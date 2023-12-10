#pragma once

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>

#include <beam_calibration/ConvertCameraModel.h>
#include <beam_utils/math.h>

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

  bool CheckRotationalVelocity(
      const std::vector<ros::Time>& timestamps,
      const std::vector<Eigen::Matrix4d, beam::AlignMat4d>& poses,
      uint16_t pose_iter, double max_rotation_rate_deg_per_s) const;

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

  void GetImageProperties();

  void GetTimeStamps();

  void OutputImages();

  cv::Mat GetImageFromBag(ros::Time& image_time, int cam_number);

  void OutputJSONList(const std::string& file_name,
                      const std::vector<std::string>& list);

  struct CameraData {
    std::string topic;
    std::string frame_id;
    double distance_between_images_m;
    double rotation_between_images_deg;
    double max_rotation_rate_deg_per_s;
    std::vector<ImageTransform> transforms;
    bool input_distorted;
    bool output_distorted;
    bool compressed;
    bool downsampled;
    bool ir_camera;
    std::shared_ptr<beam_calibration::CameraModel> model;
    std::shared_ptr<beam_calibration::UndistortImages> rectifier;
    std::vector<ros::Time> timestamps;
  };

  std::string bag_file_;
  std::string poses_file_;
  std::string save_directory_;
  std::string image_container_type_;
  rosbag::Bag bag_;
  ros::Time last_image_time_{ros::TIME_MIN};
  std::vector<CameraData> camera_data_;
};

} // namespace inspection
