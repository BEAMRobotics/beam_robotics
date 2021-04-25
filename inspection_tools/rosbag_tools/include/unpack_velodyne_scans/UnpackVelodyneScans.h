#pragma once

#include <ros/ros.h>
#include <velodyne_pointcloud/rawdata.h>

namespace unpack_velodyne_scans {

class UnpackVelodyneScans {
 public:
  explicit UnpackVelodyneScans(const std::string& bag_file_path,
                               const std::string& calibration_file,
                               const std::string& output_postfix);
  ~UnpackVelodyneScans() = default;
  void Run();

 private:
  std::string bag_file_path_;
  std::string calibration_file_;
  std::string output_postfix_;

  // Default Velodyne Parameters Required for Initialization
  double min_range_ = 0.9;
  double max_range_ = 130;
  double view_direction_ = 0;
  double view_width_ = 2 * M_PI;

  // Required Velodyne Classes for Raw data processing
  std::shared_ptr<velodyne_rawdata::RawData> data_;
  std::shared_ptr<velodyne_rawdata::DataContainerBase> container_ptr_;
};

}  // namespace unpack_velodyne_scans
