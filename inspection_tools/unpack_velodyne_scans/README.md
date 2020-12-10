# unpack_velodyne_scans

## Overview

This package will take a bag file <foo.bag> containing velodyne_msgs/VelodyneScan messages as input, process these messages to produce sensor_msgs/PointCloud2 messages per udp packet contained within each velodyne_msgs/VelodyneScan message, and then write these sensor_msgs/PointCloud2 messages to a new bag file <foo_unpacked.bag>. <foo.bag> and <foo_unpacked.bag> may then be merged as one bag using <merge_bags.py> within ../beam_robotics/scripts/tools

## Usage

Run the main exectuble as follows:

`cd catkin_ws/build/unpack_velodyne_scans`
`./unpack_velodyne_scans_main --bag_file_path=/path/to/bag/file.bag`

For information on required and optional program input, run: `./unpack_velodyne_scans_main --help`
