#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <beam_containers/ImageBridge.h>
#include <beam_containers/Utilities.h>
#include <beam_utils/log.hpp>
#include <beam_utils/math.hpp>
#include <beam_utils/time.hpp>

namespace inspection {

/**
 * @brief class for extracting images from a ROS bag
 */
class ImageExtractor {
public:
  ImageExtractor(const std::string& bag_file, const std::string& poses_file,
                 const std::string& save_directory,
                 const std::string& config_file_location);

  ~ImageExtractor() = default;

  void ExtractImages();

  void GetTimeStamps();

  std::pair<double, double> CalculatePoseChange(const Eigen::Affine3d& p1,
                                                const Eigen::Affine3d& p2);

  void OutputImages();

  cv::Mat GetImageFromBag(const beam::TimePoint& time_point,
                          rosbag::Bag& ros_bag, const int& cam_number,
                          const bool add_frame_id);

  cv::Mat EnhanceImage(cv::Mat& input_image, int cam_number);

  cv::Mat ROSDebayer(sensor_msgs::ImageConstPtr& image_raw);

  void OutputJSONList(const std::string& file_name,
                      const std::vector<std::string>& list);

private:
  std::string bag_file_, poses_file_, save_directory_, image_container_type_;
  std::vector<std::string> image_topics_, image_object_list_, camera_list_,
      enhance_methods_, frame_ids_;
  std::vector<double> distance_between_images_, rotation_between_images_,
      alphas_, betas_;
  std::vector<bool> are_images_distorted_, is_ir_camera_;
  std::vector<std::pair<beam::TimePoint, Eigen::Affine3d>> poses_;
  std::vector<std::vector<beam::TimePoint>> time_stamps_;
};

} // namespace inspection
