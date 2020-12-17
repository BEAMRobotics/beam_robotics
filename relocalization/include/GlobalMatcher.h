#pragma once

#include <beam_utils/utils.hpp>
#include <opencv2/opencv.hpp>

namespace relocalization {
/**
 * @brief Generic class for a global map matcher given a single image
 */
class GlobalMatcher {
public:
  /**
   * @brief Default constructor
   */
  GlobalMatcher() = default;

  /**
   * @brief Default destructor
   */
  virtual ~GlobalMatcher() = default;

  /**
   * @brief Virtual method to return 2d-3d point correspondences for query image
   * @param query_image to query against current map
   */
  virtual std::vector<std::tuple<Eigen::Vector2i, Eigen::Vector3d>>
      Query(cv::Mat query_image, Eigen::Matrix4d& pose_estimate) = 0;
};

} // namespace relocalization