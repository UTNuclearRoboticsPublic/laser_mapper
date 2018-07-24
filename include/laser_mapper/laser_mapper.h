// ROS Libraries
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag/bag.h>
#include <std_srvs/Trigger.h>
// Custom Libraries
#include <pointcloud_processing_server/pointcloud_process.h>
#include <pointcloud_processing_server/pointcloud_task_creation.h>
// In-package Libraries
#include "laser_mapper/cloud_map.h"
#include "laser_mapper/cloud_map_list.h"
#include "laser_mapper/cloud_select_service.h"

class LaserMapper
{
public:
	LaserMapper();

private:
	// *** Pointcloud Map *** 
	//   One of the following structs is created for each desired map
	//   Most of the information is stored in the message, for easy transport
	//   Service objects, publishers, and counters are contained in the struct
	struct PointcloudMap
	{
	public:
		laser_mapper::cloud_map map;

		int preprocessing_counter;
		int postprocessing_counter;
		pointcloud_processing_server::pointcloud_process pointcloud_preprocess;
		pointcloud_processing_server::pointcloud_process pointcloud_postprocess;
		
		int publishing_counter;
		ros::Publisher map_pub;
		ros::Publisher new_cloud_pub;
	};

	// *** Builds Settings from Parameter Server ***
	//   Requires relevant yaml files to be loaded on startup (typically by .launch)
	//   Establishers publishers for output maps and pointcloud processing settings 
	bool buildSettings(std::string yaml_file_name);
	
	// *** Callbacks and Updating Functions *** 
	//   Receives planar scans, for incrementally updated clouds
	void planarScanCallback(const sensor_msgs::PointCloud2 scan);
	//   Receives whole scans, for clouds that are not incrementally updated
	void fullScanCallback(const sensor_msgs::PointCloud2 scan);
	//   Actually performs map updates, regardless of whether planar or full 
	void updateMap(const sensor_msgs::PointCloud2 scan, PointcloudMap * map);
	//   Resets Non-persistent Clouds 
	void routineStartCallback(const std_msgs::Bool::ConstPtr& new_state);

	// *** Outputs Full Clouds ***
	bool saveClouds(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	// *** Resets a Select List of Clouds *** 
	bool resetClouds(laser_mapper::cloud_select_service::Request &req, laser_mapper::cloud_select_service::Response &res);

	ros::Subscriber planar_scan_sub_;
	ros::Subscriber full_scan_sub_;
	ros::Subscriber scan_end_sub_;

	std::vector<PointcloudMap> outputs_;
	ros::Publisher output_cloud_pub_;

	ros::ServiceServer cloud_saving_server_;
	ros::ServiceServer cloud_resetting_server_;
	ros::ServiceClient pointcloud_processor_;
	tf::TransformListener tf_listener_;
	ros::NodeHandle nh_;

};