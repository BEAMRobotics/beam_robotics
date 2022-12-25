#pragma once

#include <ros/ros.h>
#include <velodyne_pointcloud/rawdata.h>

namespace rosbag_tools {

class VelodyneTools {
public:
  explicit VelodyneTools(const std::string& lidar_model);
  sensor_msgs::PointCloud2
      UnpackScan(velodyne_msgs::VelodyneScan::ConstPtr& scan);

private:
  std::string lidar_model_;

  // Default Velodyne Parameters Required for Initialization
  double min_range_ = 0.9;
  double max_range_ = 130;
  double view_direction_ = 0;
  double view_width_ = 2 * M_PI;

  // Required Velodyne Classes for Raw data processing
  std::shared_ptr<velodyne_rawdata::RawData> data_;
  std::shared_ptr<velodyne_rawdata::DataContainerBase> container_ptr_;
};

} // namespace rosbag_tools
