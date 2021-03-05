# rosbag tools

This package contains some c++ tools for performing tasks on rosbags

## unpack_velodyne_scans

### Overview

This executable will take a bag file `foo.bag` containing *velodyne_msgs/VelodyneScan* messages on topic(s) `/bar_topic_1, /bar_topic_2, ... /bar_topic_n` as input and:

  1. for all topics `/bar_topic_1, /bar_topic_2, ... /bar_topic_n`, convert velodyne packets contained within *velodyne_msgs/VelodyneScan* messages to *sensor_msgs/PointCloud2* messages
  2. write these *sensor_msgs/PointCloud2* messages to new topics `/bar_topic_1_postfix, /bar_topic_2_postfix, ... /bar_topic_n_postfix` in a new bag `foo_postfix.bag` along with all other topics contained within `foo.bag`
   
This tool maintains the indexed times of all messages and ensures *sensor_msgs/PointCloud2* messages are indexed according to their packet time.

### Usage

Run the main exectuble as follows:

`cd catkin_ws/build/unpack_velodyne_scans` \
`./unpack_velodyne_scans_main [args]`

For information on `[args]`, run: `./unpack_velodyne_scans_main --help`