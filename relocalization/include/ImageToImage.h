#pragma once

#include <vector>

#include <GlobalMatcher.h>
#include <ImageDatabase.h>

#include <beam_cv/descriptors/Descriptor.h>
#include <beam_cv/detectors/Detector.h>
#include <beam_cv/matchers/Matcher.h>
#include <beam_cv/tracker/Tracker.h>

namespace relocalization {

/**
 * @brief Representation of a global map feature matcher using Image to Image
 * matching
 */
class ImageToImage : public GlobalMatcher {
public:
  /**
   * @brief Default constructor
   */
  ImageToImage() = default;

  /**
   * @brief Initialize with a image database
   */
  ImageToImage(std::shared_ptr<relocalization::ImageDatabase>& db,
               std::shared_ptr<beam_cv::Detector> detector,
               std::shared_ptr<beam_cv::Descriptor> descriptor,
               std::shared_ptr<beam_cv::Matcher> matcher);

  /**
   * @brief Default destructor
   */
  ~ImageToImage() override = default;

  /**
   * @brief Virtual method to return 2d-3d point correspondences for query image
   * @param query_image to query against current map
   */
  std::vector<std::tuple<Eigen::Vector2i, Eigen::Vector3d>>
      Query(cv::Mat query_image, Eigen::Matrix4d& pose_estimate) override;

  /**
   * @brief Sets the descriptor used in the tracker
   * @param descriptor to set
   */
  void SetDescriptor(std::shared_ptr<beam_cv::Descriptor> descriptor);

  /**
   * @brief Sets the detector used in the tracker
   * @param detector to set
   */
  void SetDetector(std::shared_ptr<beam_cv::Detector> detector);

  /**
   * @brief Sets the matcher used in the tracker
   * @param matcher to set
   */
  void SetMatcher(std::shared_ptr<beam_cv::Matcher> matcher);

private:
  int N = 5;
  // feature descriptor, detector and matcher
  std::shared_ptr<beam_cv::Detector> detector_;
  std::shared_ptr<beam_cv::Descriptor> descriptor_;
  std::shared_ptr<beam_cv::Matcher> matcher_;
  // image database storing images, poses and method of querying
  std::shared_ptr<relocalization::ImageDatabase> database_;
  // feature tracker to use for triangulation
  std::shared_ptr<beam_cv::Tracker> tracker_;
};

} // namespace relocalization
