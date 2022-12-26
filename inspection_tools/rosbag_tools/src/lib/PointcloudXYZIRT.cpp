

#include <rosbag_tools/PointcloudXYZIRT.h>

namespace rosbag_tools {

PointcloudXYZIRT::PointcloudXYZIRT(const double max_range,
                                   const double min_range,
                                   const std::string& target_frame,
                                   const std::string& fixed_frame,
                                   const unsigned int scans_per_block)
    : DataContainerBase(max_range, min_range, target_frame, fixed_frame, 0, 1,
                        true, scans_per_block, 6, "x", 1,
                        sensor_msgs::PointField::FLOAT32, "y", 1,
                        sensor_msgs::PointField::FLOAT32, "z", 1,
                        sensor_msgs::PointField::FLOAT32, "intensity", 1,
                        sensor_msgs::PointField::FLOAT32, "ring", 1,
                        sensor_msgs::PointField::UINT16, "time", 1,
                        sensor_msgs::PointField::FLOAT32),
      iter_x(cloud, "x"),
      iter_y(cloud, "y"),
      iter_z(cloud, "z"),
      iter_ring(cloud, "ring"),
      iter_intensity(cloud, "intensity"),
      iter_time(cloud, "time"){};

void PointcloudXYZIRT::setup(
    const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg) {
  sensor_frame = scan_msg->header.frame_id;
  manage_tf_buffer();

  cloud.header.stamp = scan_msg->header.stamp;
  cloud.data.resize(scan_msg->packets.size() * config_.scans_per_packet *
                    cloud.point_step);
  cloud.width = config_.init_width;
  cloud.height = config_.init_height;
  cloud.is_dense = static_cast<uint8_t>(config_.is_dense);

  iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
  iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
  iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
  iter_intensity = sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");
  iter_ring = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "ring");
  iter_time = sensor_msgs::PointCloud2Iterator<float>(cloud, "time");
}

void PointcloudXYZIRT::newLine() {}

void PointcloudXYZIRT::addPoint(float x, float y, float z, uint16_t ring,
                                uint16_t /*azimuth*/, float distance,
                                float intensity, float time) {
  if (!pointInRange(distance)) return;

  // convert polar coordinates to Euclidean XYZ

  transformPoint(x, y, z);

  *iter_x = x;
  *iter_y = y;
  *iter_z = z;
  *iter_ring = ring;
  *iter_intensity = intensity;
  *iter_time = time;

  ++cloud.width;
  ++iter_x;
  ++iter_y;
  ++iter_z;
  ++iter_ring;
  ++iter_intensity;
  ++iter_time;
}

const sensor_msgs::PointCloud2& PointcloudXYZIRT::finishCloud() {
  cloud.data.resize(cloud.point_step * cloud.width * cloud.height);
  cloud.row_step = cloud.point_step * cloud.width;
  if (!config_.target_frame.empty()) {
    cloud.header.frame_id = config_.target_frame;
  } else if (!config_.fixed_frame.empty()) {
    cloud.header.frame_id = config_.fixed_frame;
  } else {
    cloud.header.frame_id = sensor_frame;
  }
  return cloud;
}
} // namespace rosbag_tools
