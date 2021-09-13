# rosbag tools

This package contains some c++ tools for performing tasks on rosbags

## unpack_velodyne_scans

### Overview

This executable will take a bag file `foo.bag` containing *velodyne_msgs/VelodyneScan* messages on topic(s) `/bar_topic_1, /bar_topic_2, ... /bar_topic_n` as input and:

  1. for all topics `/bar_topic_1, /bar_topic_2, ... /bar_topic_n`, convert velodyne packets contained within *velodyne_msgs/VelodyneScan* messages to *sensor_msgs/PointCloud2* messages. The user may choose to aggregate packets into a common *sensor_msgs/PointCloud2* message or publish a *sensor_msgs/PointCloud2* message per packet.
  2. write these *sensor_msgs/PointCloud2* messages to new topics `/bar_topic_1_postfix, /bar_topic_2_postfix, ... /bar_topic_n_postfix` in a new bag `foo_postfix.bag` along with all other topics contained within `foo.bag`

### Usage

Run the main exectuble as follows:

`cd catkin_ws/build/rosbag_tools` \
`./unpack_velodyne_scans_main [args]`

For information on `[args]`, run: `./unpack_velodyne_scans_main --help`
