
#include "laser_mapper/laser_mapper.h"
#include "laser_mapper/build_settings.hpp"

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// LaserMapper - Constructor
//   1) Sets up settings
//   2) Creates communication interfaces with other nodes - pointcloud inputs and processing service 
LaserMapper::LaserMapper()
{
	// *** Subscribers ***
	tf::TransformListener tf_listener_;
	std::string pointcloud_topic;
	if( !nh_.param<std::string>("laser_stitcher/planar_scan_topic", pointcloud_topic, "laser_stitcher/planar_scan") )
		ROS_WARN_STREAM("[LaserMapper] Failed to get laser topic from parameter server - defaulting to " << pointcloud_topic << ".");
	planar_scan_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic, 100, &LaserMapper::planarScanCallback, this);
	if( !nh_.param<std::string>("laser_stitcher/full_scan_topic", pointcloud_topic, "laser_stitcher/full_scan") )
		ROS_WARN_STREAM("[LaserMapper] Failed to get laser topic from parameter server - defaulting to " << pointcloud_topic << ".");
	full_scan_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic, 100, &LaserMapper::fullScanCallback, this);

	// *** Set Up Settings ***
	std::string yaml_file_name;
	if(!nh_.getParam("laser_stitcher/yaml_file_name", yaml_file_name))
	{
		ROS_ERROR_STREAM("[LaserMapper] Failed to get yaml file name for output cloud format.");
		return;
	}
	this->buildSettings(yaml_file_name);

	// *** Pointcloud Processor *** 
	pointcloud_processor_ = nh_.serviceClient<pointcloud_processing_server::pointcloud_process>("pointcloud_service");

	// *** Save Clouds Service ***
	cloud_saving_server_ = nh_.advertiseService("save_cloud_maps", &LaserMapper::saveClouds, this);

	ros::Duration(0.50).sleep();
	ROS_INFO_STREAM("[LaserMapper] Stitcher online and ready. Listening on topics " << planar_scan_sub_.getTopic() << " and " << full_scan_sub_.getTopic());

	ROS_INFO_STREAM(outputs_.size() << " " << outputs_[1].map.name << " " << outputs_[1].preprocessing_counter << " " << outputs_[1].map.preprocessing_throttle_max);

	ros::spin();
}

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// Planar Scan Callback
//   Receives planar scans, calls map updater 
void LaserMapper::planarScanCallback(const sensor_msgs::PointCloud2 scan)
{
	for(int i=0; i<outputs_.size(); i++)
	{
		if(outputs_[i].map.build_incrementally)
			updateMap(scan, &outputs_[i]);
	}
}

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// Full Scan Callback
//   Receives full scans, calls map updater 
void LaserMapper::fullScanCallback(const sensor_msgs::PointCloud2 scan)
{
	for(int i=0; i<outputs_.size(); i++)
	{
		if(!outputs_[i].map.build_incrementally)
			updateMap(scan, &outputs_[i]);
	}
}

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// Update Map
//   1) Processes incoming scans
//   2) Adds new data to map
//   3) Postprocesses map
//   4) Publishes outputs 
void LaserMapper::updateMap(sensor_msgs::PointCloud2 scan, LaserMapper::PointcloudMap * map)
{
	ROS_DEBUG_STREAM("[LaserMapper] " << map->map.name << " - received a new input scan of size " << scan.height*scan.width);
	
	// *** Preprocessing ***
	//   Preprocess incoming cloud
	//   Relies on external package <pointcloud_processing_server> 
	if(map->map.should_preprocess)
	{
		if(map->preprocessing_counter >= map->map.preprocessing_throttle_max)
		{
			map->pointcloud_preprocess.request.pointcloud = scan;
			if(!pointcloud_processor_.call(map->pointcloud_preprocess))
				ROS_WARN_STREAM("[LaserMapper] Pointcloud Preprocess on map " << map->map.name << " failed. Continuing using raw cloud.");
			else
				scan = map->pointcloud_preprocess.response.task_results[map->pointcloud_preprocess.response.task_results.size()-1].task_pointcloud;
			map->preprocessing_counter = 0;
			ROS_DEBUG_STREAM("[LaserMapper] " << map->map.name << " - new scan size after preprocessing: " << scan.height*scan.width << " and preprocess size " << map->pointcloud_preprocess.response.task_results.size()-1);
		}
		else map->preprocessing_counter++;
	}
	
	// *** Transform *** 
	//   Check to make sure that incoming scan frame matches target
	//   If it doesn't, transform it
	if(scan.header.frame_id.compare(map->map.pointcloud.header.frame_id) != 0) 		// string::compare returns a 0 if the strings are identical 
	{
		  // Transform pointcloud in space to align origin to Camera Frame
		if(tf_listener_.waitForTransform(scan.header.frame_id, map->map.pointcloud.header.frame_id, ros::Time::now(), ros::Duration(0.5)))  
		{
			sensor_msgs::PointCloud2 transformed_scan;
			pcl_ros::transformPointCloud (map->map.pointcloud.header.frame_id, scan, transformed_scan, tf_listener_);  	// transforms input_pc2 into process_message
			scan = transformed_scan;
		}
		else  	// if Transform request times out... Continues WITHOUT TRANSFORM
			ROS_WARN_STREAM_THROTTLE(60, "[PointcloudProcessing] listen for transformation from " << scan.header.frame_id << " to " << map->map.pointcloud.header.frame_id << " timed out. Proceeding...");
	}

	// *** Update Map ***
	//const sensor_msgs::PointCloud2 previous_cloud_state = map->map.pointcloud;
	//pcl::concatenatePointCloud(previous_cloud_state, scan, map->map.pointcloud);
	pcl::concatenatePointCloud(map->map.pointcloud, scan, map->map.pointcloud);
	ROS_DEBUG_STREAM("[LaserMapper] " << map->map.name << " - current cloud size after concatenation: " << map->map.pointcloud.height*map->map.pointcloud.width);


	// *** Postprocessing ***
	//   Process entire map cloud
	//   Relies on external package <pointcloud_processing_server> 
	if(map->map.should_postprocess)
	{
		if(map->postprocessing_counter >= map->map.postprocessing_throttle_max)
		{
			map->pointcloud_postprocess.request.pointcloud = map->map.pointcloud;
			if(!pointcloud_processor_.call(map->pointcloud_postprocess))
				ROS_WARN_STREAM("[LaserMapper] Pointcloud Postprocess on map " << map->map.name << " failed. Continuing using raw cloud.");
			else
				map->map.pointcloud = map->pointcloud_postprocess.response.task_results[map->pointcloud_postprocess.response.task_results.size()-1].task_pointcloud;
			map->postprocessing_counter = 0;
			ROS_DEBUG_STREAM("[LaserMapper] " << map->map.name << " - current full cloud size after postprocessing: " << map->map.pointcloud.height*map->map.pointcloud.width);
		}
		else map->postprocessing_counter++;
	}

	// *** Output ***
	map->new_cloud_pub.publish(scan);
	map->map_pub.publish(map->map.pointcloud);
}


// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// Save
//   Service call, saves all clouds which are set to be saved
bool LaserMapper::saveClouds(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	for(int i=0; i<outputs_.size(); i++)
	{
		// *** Postprocessing ***
		//   Postprocess map one more time before saving (just in case)
		//   Relies on external package <pointcloud_processing_server> 
		if(outputs_[i].map.should_postprocess)
		{
			if(outputs_[i].postprocessing_counter >= outputs_[i].map.postprocessing_throttle_max)
			{
				outputs_[i].pointcloud_postprocess.request.pointcloud = outputs_[i].map.pointcloud;
				if(!pointcloud_processor_.call(outputs_[i].pointcloud_postprocess))
					ROS_WARN_STREAM("[LaserMapper] Pointcloud Postprocess on map " << outputs_[i].map.name << " failed. Continuing using raw cloud.");
				else
					outputs_[i].map.pointcloud = outputs_[i].pointcloud_postprocess.response.task_results[outputs_[i].pointcloud_postprocess.response.task_results.size()-1].task_pointcloud;
				outputs_[i].postprocessing_counter = 0;
				ROS_DEBUG_STREAM("[LaserMapper] " << outputs_[i].map.name << " - current full cloud size after postprocessing: " << outputs_[i].map.pointcloud.height*outputs_[i].map.pointcloud.width);
			}
			else outputs_[i].postprocessing_counter++;
		}

		// *** Output Publishing ***
		outputs_[i].map_pub.publish(outputs_[i].map.pointcloud);

		/// *** Save To Bag ***
		if(outputs_[i].map.should_save)
		{
			rosbag::Bag bag;
			std::string bag_name = "laser_mapper_" + outputs_[i].map.name + std::to_string(ros::Time::now().toSec()) + ".bag";
			bag.open(bag_name, rosbag::bagmode::Write);
			bag.write(outputs_[i].map_pub.getTopic(), ros::Time::now(), outputs_[i].map.pointcloud);
			ROS_INFO_STREAM("[LaserStitcher] Saved a ROSBAG to the file " << bag_name);
		}
		ROS_INFO_STREAM("[LaserStitcher] Outputting a full map with name " << outputs_[i].map.name << ". Cloud size: " << outputs_[i].map.pointcloud.height * outputs_[i].map.pointcloud.width << ".");

		sensor_msgs::PointCloud2Modifier cloud_modifier_(outputs_[i].map.pointcloud);
		cloud_modifier_.resize(0);

	}
} 




// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// Main
//   Creates LaserMapper object; optional debugging output code 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_mapper");

if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();

	LaserMapper laser_mapper;
}