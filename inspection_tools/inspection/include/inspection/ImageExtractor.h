#pragma once

#include <Eigen/Dense>
#include <beam/utils/log.hpp>
#include <beam/utils/math.hpp>
#include <beam/utils/time.hpp>
#include <beam_containers/Utilities.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <rosbag/bag.h>
#include <string>
#include <vector>
// #include <rosbag/view.h>

namespace inspection {

/**
 * @brief class for extracting images from a ROS bag
 */
class ImageExtractor {
public:
  ImageExtractor(const std::string &config_file_location);

  ~ImageExtractor() = default;

  void ExtractImages();

  void GetTimeStamps();

  std::pair<double, double>
  CalculatePoseChange(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2);

  void OutputImages();

private:
  std::string bag_file_, poses_file_, save_directory_;
  std::vector<std::string> image_topics_;
  std::vector<double> distance_between_images_;
  std::vector<double> rotation_between_images_;
  std::vector<std::pair<beam::TimePoint, Eigen::Affine3d>> poses_;
  std::vector<std::vector<beam::TimePoint>> time_stamps_;
};

} // namespace inspection
