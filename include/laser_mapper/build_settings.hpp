
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// Build Settings
//   Fills out settings in PointcloudMap object from parameter file
//   Input string is the prefix at the top of the yaml file used to load the parameters 
bool LaserMapper::buildSettings(std::string yaml_file_name)
{
	std::vector<std::string> cloud_list;
	if(!nh_.getParam(yaml_file_name + "/cloud_list", cloud_list))
	{
		ROS_ERROR_STREAM("[LaserMapper] Failed to get list of output cloud names from the parameters server. Settings initialization failed.");
		return false;
	}
	ROS_INFO_STREAM("[LaserMapper] Retrieved list of output cloud names from parameter server - " << cloud_list.size() << " entries.");

	outputs_.clear();
	for(int i=0; i<cloud_list.size(); i++)
	{
		PointcloudMap cloud_options;

		// *** Name, Publishers, and Frame *** 
		cloud_options.map.name = cloud_list[i];
		cloud_options.map_pub = nh_.advertise<sensor_msgs::PointCloud2>("laser_stitcher/" + cloud_list[i], 1, this);
		cloud_options.new_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("laser_stitcher/" + cloud_list[i] + "/new_cloud", 1, this);
		if( !nh_.param<std::string>(yaml_file_name + "/" + cloud_list[i] + "/desired_frame", cloud_options.map.pointcloud.header.frame_id, "map") )
			ROS_WARN_STREAM("[LaserMapper] Failed to get " << yaml_file_name << "/" << cloud_list[i] << "/desired_frame from parameter server - defaulting to " << cloud_options.map.pointcloud.header.frame_id);

		// *** When To Update *** 
		bool temp_bool;
		if( !nh_.param<bool>(yaml_file_name + "/" + cloud_list[i] + "/incremental_update", temp_bool, true) )
			ROS_WARN_STREAM("[LaserMapper] Failed to get " << yaml_file_name << "/" << cloud_list[i] << "/incremental_update from parameter server - defaulting to " << temp_bool);
		cloud_options.map.build_incrementally = temp_bool;
		if( !nh_.param<bool>(yaml_file_name + "/" + cloud_list[i] + "/retain_after_scan", temp_bool, true) )
			ROS_WARN_STREAM("[LaserMapper] Failed to get " << yaml_file_name << "/" << cloud_list[i] << "/retain_after_scan from parameter server - defaulting to " << temp_bool);
		cloud_options.map.persistent = temp_bool;

		// *** Output Settings *** 
		if( !nh_.param<bool>(yaml_file_name + "/" + cloud_list[i] + "/should_save", temp_bool, true) )
			ROS_WARN_STREAM("[LaserMapper] Failed to get " << yaml_file_name << "/" << cloud_list[i] << "/should_save from parameter server - defaulting to " << temp_bool);
		cloud_options.map.should_save = temp_bool;
		if( !nh_.param<bool>(yaml_file_name + "/" + cloud_list[i] + "/throttle_publish", temp_bool, false) )
			ROS_WARN_STREAM("[LaserMapper] Failed to get " << yaml_file_name << "/" << cloud_list[i] << "/throttle_publish from parameter server - defaulting to " << temp_bool);
		cloud_options.map.throttle_publishing = temp_bool;
		if(cloud_options.map.throttle_publishing)
		{
			if(!nh_.param<int>(yaml_file_name + "/" + cloud_list[i] + "/publishing_throttle_max", cloud_options.map.publishing_throttle_max, 5))
				ROS_WARN_STREAM("[LaserMapper] Publishing throttling requested for output " << cloud_list[i] << ", but throttle maximum not found in parameter server. Setting publish throttle to " << cloud_options.map.publishing_throttle_max);
			cloud_options.publishing_counter = 0;
		}

		// *** Preprocessing Settings *** 
		if( !nh_.param<bool>(yaml_file_name + "/" + cloud_list[i] + "/preprocessing/should_process", temp_bool, false) )
			ROS_WARN_STREAM("[LaserMapper] Failed to get " << yaml_file_name << "/" << cloud_list[i] << "/preprocessing/should_process from parameter server - defaulting to " << temp_bool);
		cloud_options.map.should_preprocess = temp_bool;
		if(cloud_options.map.should_preprocess)
		{
			pointcloud_processing_server::pointcloud_process pointcloud_process;
			PointcloudTaskCreation::processFromYAML(&pointcloud_process, cloud_list[i] + "/preprocessing", yaml_file_name);
			cloud_options.pointcloud_preprocess = pointcloud_process;
			cloud_options.map.preprocessing_task_list = pointcloud_process.request.tasks;
			
			if( !nh_.param<bool>(yaml_file_name + "/" + cloud_list[i] + "/preprocessing/throttle", temp_bool, false) )
				ROS_WARN_STREAM("[LaserMapper] Failed to get " << yaml_file_name << "/" << cloud_list[i] << "/preprocessing/throttle from parameter server - defaulting to " << temp_bool);
			cloud_options.map.throttle_preprocessing = temp_bool;
			if(cloud_options.map.throttle_preprocessing)
			{
				if(!nh_.param<int>(yaml_file_name + "/" + cloud_list[i] + "/preprocessing/throttle_max", cloud_options.map.preprocessing_throttle_max, 50))
					ROS_WARN_STREAM("[LaserMapper] Preprocessing throttling requested for output " << cloud_list[i] << ", but " << yaml_file_name << "/" << cloud_list[i] << "/preprocessing/throttle_max not found in parameter server. Setting processing throttle to " << cloud_options.map.preprocessing_throttle_max);
				cloud_options.preprocessing_counter = 0;
			}
			
		}

		// *** Postprocessing Settings ***
		if( !nh_.param<bool>(yaml_file_name + "/" + cloud_list[i] + "/postprocessing/should_process", temp_bool, false) )
			ROS_WARN_STREAM("[LaserMapper] Failed to get " << yaml_file_name << "/" << cloud_list[i] << "/postprocessing/should_process from parameter server - defaulting to " << temp_bool);
		cloud_options.map.should_postprocess = temp_bool;
		if(cloud_options.map.should_postprocess)
		{
			pointcloud_processing_server::pointcloud_process pointcloud_process;
			PointcloudTaskCreation::processFromYAML(&pointcloud_process, cloud_list[i] + "/postprocessing", yaml_file_name);
			cloud_options.pointcloud_postprocess = pointcloud_process;
			cloud_options.map.postprocessing_task_list = pointcloud_process.request.tasks;
			
			if( !nh_.param<bool>(yaml_file_name + "/" + cloud_list[i] + "/postprocessing/throttle", temp_bool, false) )
				ROS_WARN_STREAM("[LaserMapper] Failed to get " << yaml_file_name << "/" << cloud_list[i] << "/postprocessing/throttle from parameter server - defaulting to " << temp_bool);
			cloud_options.map.throttle_postprocessing = temp_bool;
			if(cloud_options.map.throttle_postprocessing)
			{
				if(!nh_.param<int>(yaml_file_name + "/" + cloud_list[i] + "/postprocessing/throttle_max", cloud_options.map.postprocessing_throttle_max, 50))
					ROS_WARN_STREAM("[LaserMapper] Postprocessing throttling requested for output " << cloud_list[i] << ", but " << yaml_file_name << "/" << cloud_list[i] << "/postprocessing/throttle_max not found in parameter server. Setting processing throttle to " << cloud_options.map.postprocessing_throttle_max);
				cloud_options.postprocessing_counter = 0;
			}
			
		}

		outputs_.push_back(cloud_options);
	}
}