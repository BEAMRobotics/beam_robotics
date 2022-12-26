#pragma once

#include <string>
#include <velodyne_pointcloud/datacontainerbase.h>

namespace rosbag_tools {

class PointcloudXYZIRT : public velodyne_rawdata::DataContainerBase {
public:
  PointcloudXYZIRT(const double max_range, const double min_range,
                   const std::string& target_frame,
                   const std::string& fixed_frame,
                   const unsigned int scans_per_block);

  virtual void newLine();

  virtual void setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg);

  virtual void addPoint(float x, float y, float z, const uint16_t ring,
                        const uint16_t azimuth, const float distance,
                        const float intensity, const float time);

  virtual const sensor_msgs::PointCloud2& finishCloud();

  sensor_msgs::PointCloud2Iterator<float> iter_x, iter_y, iter_z,
      iter_intensity, iter_time;
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring;
};
} // namespace rosbag_tools
