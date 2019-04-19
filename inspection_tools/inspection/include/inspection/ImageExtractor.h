#pragma once

// Gen includes
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// ROS includes
#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

// beam includes
#include <beam_utils/log.hpp>
#include <beam_utils/math.hpp>
#include <beam_utils/time.hpp>
#include <beam_containers/ImageBridge.h>
#include <beam_containers/Utilities.h>

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

  std::pair<double, double> CalculatePoseChange(const Eigen::Affine3d &p1,
                                                const Eigen::Affine3d &p2);

  void OutputImages();

  cv::Mat GetImageFromBag(const beam::TimePoint &time_point,
                          rosbag::Bag &ros_bag, std::string &image_topic,
                          bool add_frame_id);

  void OutputJSONList(const std::string &file_name,
                 const std::vector<std::string> &list);

private:
  std::string bag_file_, poses_file_, save_directory_, image_container_type_;
  std::vector<std::string> image_topics_, image_object_list_, camera_list_,
                           frame_ids_;
  std::vector<double> distance_between_images_;
  std::vector<double> rotation_between_images_;
  std::vector<bool> are_images_distorted_, is_ir_camera_;
  std::vector<std::pair<beam::TimePoint, Eigen::Affine3d>> poses_;
  std::vector<std::vector<beam::TimePoint>> time_stamps_;
};

} // namespace inspection
