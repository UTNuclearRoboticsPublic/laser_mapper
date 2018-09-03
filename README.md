# Table of Contents
1. [About](#about)
2. [Parameter Setup](#parameter-setup)
3. [Use](#use)

***

## About

Laser Mapper is a utility to allow collection of planar pointclouds (from a sensor_msgs/LaserScan) and stitching into cohesive 3D clouds, with automatic postprocessing of the data using PCL. This package is intended to be as a layer on top of the [laser_stitcher](https://github.com/UTNuclearRoboticsPublic/laser_stitcher.git) package to provide postprocessing of the generated data. 

The utility allows multiple different output cloud formats to be specified in a yaml file and produced continuously and simultaneously with one another. Clouds can be updated once per 'full scan' (adding an entire 3D scan at a time) or once every time a new planar scan is input. 

## Parameter Setup

An example yaml file can be seen in /laser_mapper/param/canyon_mapper_output.yaml

The top level namespace in each yaml file should be a descriptive term relating to the particular mapping application desired (eg matching the name of the parameter file created). 

The most important parameter at the next level down is the cloud_list parameter, which should contain a list of names of clouds to be output. These cloud names MUST match the names used below.

For each cloud in the list, a definition is created at the same level in the parameter space as the cloud list. That definition includes the following parameters:

- desired_frame: the output frame in which the cloud should be compiled (this should be a static frame such as /map)
- incremental_update: if this is true, the cloud will be updated every time a new planar scan is input. Otherwise, it will only be updated every time a full_scan is created
- publish: if this is true, the mapper will publish the cloud on an output topic
- throttle_publish: if this is true, the cloud will only be published a subset of the times it is updated
- publishing_throttle_max: if throttle_publish is true, this number controls how many updates occur before a cloud is published
- retain_after_scan: if this is true, and if scans are being published continually (not just a single scan - see laser_stitcher), then the cloud will be kept and built up across subsequent scans. Otherwise, it will be discarded and restarted for each scan
- should_save: if this is true, then the cloud will be automatically saved once a full scan is finished
- preprocessing: this PCL processing is applied to INCOMING clouds (either planar or fully 3D, depending on incremental_update) and its parameters follow conventions defined in [pointcloud_processing_server](https://github.com/UTNuclearRobotics/pointcloud_processing_server.git)
- postprocessing: this PCL processing is applied to OUTGOING clouds after they've been updated based on new data, and its parameters follow conventions defined in [pointcloud_processing_server](https://github.com/UTNuclearRobotics/pointcloud_processing_server.git)
- should_process: both pre- and postprocessing have this option, which decides whether any processing of this type should occur
- throttle: if this is true, then processing of this type will ONLY occur once for every several processing opportunities. This should probably never be true for preprocessing and should always be true for postprocessing if incremental_update is true, while never true if it's not. Otherwise, either clouds will get through unprocessed, or postprocessing will happen too frequently, which gets expensive as the overall cloud gets big
- throttle_max: if throttle above is true, then processing of this type will only occur every X opportunities for processing

## Use

Laser_Mapper is intended to be used as an extra layer on top of the [laser_stitcher](https://github.com/UTNuclearRoboticsPublic/laser_stitcher.git) package. Run laser_stitcher as outlined in that package, then load a copy of the necessary laser_mapper yaml file structure and run the laser_mapper node. This can all be done in a single launch file, as given in the example /laser_mapper/launch/laser_mapper.launch
