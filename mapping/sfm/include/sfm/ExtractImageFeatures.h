#pragma once

#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>

namespace sfm {

/**
 * @brief class for extracting image features for a directory containing images
 */
class ExtractImageFeatures {
public:
  ExtractImageFeatures(const std::string& image_directory,
                       const std::string& output_directory,
                       bool search_recursively,
                       const std::string& file_extension,
                       const std::string& feature_type,
                       const std::string& model_path, int num_features, double downsize_image);

  ~ExtractImageFeatures() = default;

  void ExtractFeatures();

private:
  void GetFeatureDetectorDescriptor(
      std::shared_ptr<beam_cv::Detector>& detector,
      std::shared_ptr<beam_cv::Descriptor>& descriptor);

  void WriteToFile(const std::vector<cv::KeyPoint>& keypoints,
                   const cv::Mat& descriptors, const std::string& filename);

  std::string image_directory_;
  std::string output_directory_;
  bool search_recursively_;
  std::string file_extension_;
  std::string feature_type_;
  std::string model_path_;
  int num_features_;
  double downsize_image_;
  std::shared_ptr<beam_cv::SuperPointModel> model_;
};

} // namespace sfm
