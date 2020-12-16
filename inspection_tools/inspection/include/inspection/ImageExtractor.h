#pragma once

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>

#include <beam_calibration/ConvertCameraModel.h>

namespace inspection {

/**
 * @brief struct for holding image transform details
 */
struct ImageTransform {
  std::string type;
  std::vector<double> params;
};

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

private:
  std::vector<ImageTransform> GetImageTransforms(const nlohmann::json& J);

  cv::Mat ApplyImageTransforms(const cv::Mat& input_image, int cam_number);

  cv::Mat ApplyLinearTransform(const cv::Mat& image,
                               const std::vector<double>& params);

  cv::Mat ApplyHistogramTransform(const cv::Mat& image,
                                  const std::vector<double>& params);

  // https://en.wikipedia.org/wiki/Adaptive_histogram_equalization#Contrast_Limited_AHE
  cv::Mat ApplyClaheTransform(const cv::Mat& image,
                              const std::vector<double>& params);

  // NOTE: You cannot convert images that are cropped and compressed because we
  // do not know how to convert them to the camera model dimensions which is
  // required for undistorting.
  cv::Mat ApplyUndistortTransform(const cv::Mat& image,
                                  const std::vector<double>& params,
                                  int cam_number);

  void GetTimeStamps();

  bool PassedMinMotion(const Eigen::Affine3d& TA_curr_last, int cam_number);

  void OutputImages();

  cv::Mat GetImageFromBag(ros::Time& image_time, int cam_number,
                          bool first_image);

  cv::Mat ROSConvertColor(const sensor_msgs::ImageConstPtr& image_raw);

  void OutputJSONList(const std::string& file_name,
                      const std::vector<std::string>& list);

  std::string bag_file_;
  std::string poses_file_;
  std::string save_directory_;
  std::string image_container_type_;
  std::vector<std::string> image_topics_;
  std::vector<std::string> image_object_list_;
  std::vector<std::string> camera_list_;
  std::vector<std::string> frame_ids_;
  std::vector<std::string> intrinsics_;
  std::vector<double> distance_between_images_;
  std::vector<double> rotation_between_images_;
  std::vector<std::vector<ImageTransform>> image_transforms_;
  std::vector<bool> are_input_images_distorted_;
  std::vector<bool> are_output_images_distorted_;
  std::vector<bool> are_images_compressed_;
  std::vector<bool> is_ir_camera_;
  std::vector<std::vector<ros::Time>> image_time_stamps_;
  std::vector<std::shared_ptr<beam_calibration::UndistortImages>>
      undistort_images_;
  rosbag::Bag bag_;
  ros::Time last_image_time_;
};

} // namespace inspection
