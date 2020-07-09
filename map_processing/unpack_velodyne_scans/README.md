# unpack_velodyne_scans

## Overview

This package will take a bag file <foo.bag> containing velodyne_msgs/VelodyneScan
messages as input, process these messages to produce sensor_msgs/PointCloud2
messages per udp packet contained within velodyne_msgs/VelodyneScan, and then
write these sensor_msgs/PointCloud2 messages to a new bag file <foo_unpacked.bag>.   

## Usage

Run the main exectuble as follows:

  `cd catkin_ws/devel/lib/unpack_velodyne_scans`
	`./unpack_velodyne_scans_main /path/to/bag/file.bag velodyne_calibration.yaml`

	where velodyne_calibration.yaml can be any one of the calibration files listed
	under ~/velodyne/velodyne_pointcloud/params (ex. VLP16_hires_db.yaml)
