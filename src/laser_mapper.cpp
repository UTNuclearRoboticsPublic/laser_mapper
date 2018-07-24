
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
	std::string subscriber_topics;
	if( !nh_.param<std::string>("laser_stitcher/planar_scan_topic", subscriber_topics, "laser_stitcher/planar_scan") )
		ROS_WARN_STREAM("[LaserMapper] Failed to get planar cloud topic from parameter server - defaulting to " << subscriber_topics << ".");
	planar_scan_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(subscriber_topics, 100, &LaserMapper::planarScanCallback, this);
	if( !nh_.param<std::string>("laser_stitcher/full_scan_topic", subscriber_topics, "laser_stitcher/full_scan") )
		ROS_WARN_STREAM("[LaserMapper] Failed to get full scan topic from parameter server - defaulting to " << subscriber_topics << ".");
	full_scan_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(subscriber_topics, 100, &LaserMapper::fullScanCallback, this);
	if( !nh_.param<std::string>("laser_stitcher/scanning_state_topic", subscriber_topics, "laser_stitcher/scanning_state") )
		ROS_WARN_STREAM("[LaserMapper] Failed to get routine ending message topic from parameter server - defaulting to " << subscriber_topics << ".");
	scan_end_sub_ = nh_.subscribe<std_msgs::Bool>(subscriber_topics, 100, &LaserMapper::routineStartCallback, this);

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

	// *** Cloud Access Services ***
	cloud_saving_server_ = nh_.advertiseService("save_cloud_maps", &LaserMapper::saveClouds, this);
	cloud_resetting_server_ = nh_.advertiseService("reset_cloud_maps", &LaserMapper::resetClouds, this);

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
		{
			// If the cloud is not kept across scans, clear it before inserting the new cloud
			//   For clouds that are 'built incrementally' this is handled in the new scan callback
			//   in LaserMapper::routineStartCallback() below
			if(!outputs_[i].map.persistent)
			{
				sensor_msgs::PointCloud2Modifier cloud_modifier_(outputs_[i].map.pointcloud);
				cloud_modifier_.resize(0);
			}
			updateMap(scan, &outputs_[i]);
		}
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
			ROS_INFO_STREAM("[LaserMapper] Attempting to postprocess cloud " << outputs_[i].map.name << ", currently of size " << outputs_[i].map.pointcloud.width*outputs_[i].map.pointcloud.height);
			if(outputs_[i].postprocessing_counter >= outputs_[i].map.postprocessing_throttle_max)
			{
				int num_postprocess_tasks = 0;
				outputs_[i].pointcloud_postprocess.request.pointcloud = outputs_[i].map.pointcloud;
				if(!pointcloud_processor_.call(outputs_[i].pointcloud_postprocess))
					ROS_WARN_STREAM("[LaserMapper] Pointcloud Postprocess on map " << outputs_[i].map.name << " failed. Continuing using raw cloud.");
				else
				{
					num_postprocess_tasks = outputs_[i].pointcloud_postprocess.response.task_results.size();
					outputs_[i].map.pointcloud = outputs_[i].pointcloud_postprocess.response.task_results[num_postprocess_tasks-1].task_pointcloud;
				}
				outputs_[i].postprocessing_counter = 0;
				ROS_INFO_STREAM("[LaserMapper] " << outputs_[i].map.name << " - current full cloud size after postprocessing: " << outputs_[i].map.pointcloud.height*outputs_[i].map.pointcloud.width << ". Number of postprocess tasks: " << num_postprocess_tasks);
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
			ROS_INFO_STREAM("[LaserMapper] Saved a ROSBAG to the file " << bag_name);
		}
		ROS_INFO_STREAM("[LaserMapper] Outputting a full map with name " << outputs_[i].map.name << ". Cloud size: " << outputs_[i].map.pointcloud.height * outputs_[i].map.pointcloud.width << ".");
	}
	return true;
} 



// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// Clear Clouds
//   Service call, clears a selected set of output clouds and restarts them from scratch
bool LaserMapper::resetClouds(laser_mapper::cloud_select_service::Request &req, laser_mapper::cloud_select_service::Response &res)
{
	for(int i=0; i<req.cloud_list.size(); i++)
	{
		res.success.push_back(false);
		for(int j=0; j<outputs_.size(); j++)
		{
			if(req.cloud_list[i].compare(outputs_[j].map.name) == 0)
			{
				sensor_msgs::PointCloud2Modifier cloud_modifier_(outputs_[i].map.pointcloud);
				cloud_modifier_.resize(0);
				res.success[i] = true;
				break;
			}
		}
		if(res.success[i] == false)
			ROS_WARN_STREAM("[LaserMapper] Reset requested for cloud " << req.cloud_list[i] << ", but that that cloud is not known.");
	}
	return true;
}



// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// Scan Finished Callback
//   This listens for when laser_stitcher receives an input message to end a scanning routine. 
//   Once this occurs, the laser_mapper will reset any clouds which have 'persistent' set to false (the yaml parameter retain_after_scan)
void LaserMapper::routineStartCallback(const std_msgs::Bool::ConstPtr& new_state)
{
	// Only reset non-persistent clouds when a new scan routine is being STARTED
	//   This means that when a routine is ended laser_mapper clouds will be kept until a new one is started (useful if only running once, not in loop)
	//   Note - right now this doesn't check the state of the laser_stitcher, so if a second 'start' message is received during a running routine,
	//   the laser_mapper clouds will be reset even though the laser_stitcher clouds will not.
	// Only clouds that are 'incrementally' built are cleared at the beginning of scans. Scans that are built at the end of a full routine but are not 
	//   persistent across routines are kept through the routine following their creation and only cleared once new data comes in 
	if(!new_state->data)
		return; 			  
	for(int i=0; i<outputs_.size(); i++)
	{
		ROS_INFO_STREAM("[LaserMapper] Starting a new laser_stitcher scan routine. Cloud " << outputs_[i].map.name << " size is currently " << outputs_[i].map.pointcloud.height*outputs_[i].map.pointcloud.width);
		if(!outputs_[i].map.persistent && outputs_[i].map.build_incrementally)
		{
			sensor_msgs::PointCloud2 new_cloud;
			new_cloud.header.frame_id = outputs_[i].map.pointcloud.header.frame_id;
			new_cloud.header.stamp = ros::Time::now();
			outputs_[i].map.pointcloud = new_cloud;
		}
	}
}



// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// Main
//   Creates LaserMapper object; optional debugging output code 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_mapper");

//if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
//    ros::console::notifyLoggerLevelsChanged();

	LaserMapper laser_mapper;
}