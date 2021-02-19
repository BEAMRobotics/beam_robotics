# rosbag tools

This package contains some c++ tools for performing tasks on rosbags

## unpack_velodyne_scans

### Overview

This package will take a bag file <foo.bag> containing velodyne_msgs/VelodyneScan messages as input, process these messages to produce sensor_msgs/PointCloud2 messages per udp packet contained within each velodyne_msgs/VelodyneScan message, and then write these sensor_msgs/PointCloud2 messages to a new bag file <foo_unpacked.bag> along with all other topics contained within <foo.bag>. This tool maintains the indexed times of all messages and ensures sensor_msgs/PointCloud2 messages are indexed correctly.

### Usage

Run the main exectuble as follows:

`cd catkin_ws/build/unpack_velodyne_scans`
`./unpack_velodyne_scans_main --bag_file_path=/path/to/bag/file.bag`

For information on required and optional program input, run: `./unpack_velodyne_scans_main --help`

## offset_message_timestamps

### Overview

This script can be used to add or remove a constant time offset from a specific topic. It will then resave the bag to a new name

### Usage


Run the main exectuble as follows:

`cd catkin_ws/build/offset_message_timestamps`
`./offset_message_timestamps_main [args]

For information on [args], run: `./unpack_velodyne_scans_main --help`