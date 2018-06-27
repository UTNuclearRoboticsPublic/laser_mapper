# *** Basic ***
string 												name
sensor_msgs/PointCloud2 							pointcloud

# *** Building Settings ***
bool 												build_incrementally 		# Is the map updated for every new planar_scan received?
bool 												persistent 					# Does the map persist across full scan routines? If no, will be reset with each scan

# *** Preprocessing *** 
bool                                                should_preprocess			# If FALSE, then cloud will be saved as raw data 
pointcloud_processing_server/pointcloud_task[]      preprocessing_task_list		# Filtering, Transforms, Clipping, Voxelization, RANSAC Segmentation...
int32                                               min_cloud_size_pre			# If cloud falls below this size, processing will 'fail'
bool                                                throttle_preprocessing		# If TRUE, then processing only occurs once every X times a cloud is incrementally updated
int32                                               preprocessing_throttle_max 	# The X for the above

# *** Postprocessing *** 
bool                                                should_postprocess			# If FALSE, then cloud will be saved as raw data 
pointcloud_processing_server/pointcloud_task[]      postprocessing_task_list	# Filtering, Transforms, Clipping, Voxelization, RANSAC Segmentation...
int32                                               min_cloud_size_post			# If cloud falls below this size, processing will 'fail'
bool                                                throttle_postprocessing		# If TRUE, then processing only occurs once every X times a cloud is incrementally updated
int32                                               postprocessing_throttle_max # The X for the above

# *** Publishing Settings ***
bool                                                throttle_publishing			# If TRUE, then publishing only occurs once every X times a cloud is incrementally updated
int32                                               publishing_throttle_max 	# The X for the above 

# *** Saving Settings ***
bool 												should_save 				# Should the map be saved to .bag file when program finishes?
string 												bag_file_name 				# Name for saved .bag file