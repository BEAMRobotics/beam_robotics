#pragma once

#include <ros/ros.h>
#include <velodyne_pointcloud/rawdata.h>

namespace unpack_velodyne_scans {

class UnpackVelodyneScans {
 public:
  explicit UnpackVelodyneScans(const std::string& bag_file_path,
                               const std::string& calibration_file);
  ~UnpackVelodyneScans() = default;
  void Unpack();

 private:
  std::string bag_file_path_;
  std::string calibration_file_;
  std::string calibration_path_;

  // Default Velodyne Parameters Required for RawData Initialization
  double min_range_ = 0.9;
  double max_range_ = 130;
  double view_direction_ = 0;
  double view_width_ = 2 * M_PI;
  std::string target_frame_ = "";
  std::string fixed_frame_ = "";

  boost::shared_ptr<velodyne_rawdata::RawData> data_;
  boost::shared_ptr<velodyne_rawdata::DataContainerBase> container_ptr_;
};

}  // namespace unpack_velodyne_scans
