#pragma once

#include <beam_containers/PointBridge.h>
#include <beam_utils/time.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

namespace inspection {

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;
using BridgePoint = beam_containers::PointBridge;
using DefectCloud = pcl::PointCloud<beam_containers::PointBridge>;
using PointCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZRGB>;

/**
 * @brief Class for combining labeled clouds from each image / camera labeled
 * cloud
 */
class CloudCombiner {
public:
  CloudCombiner() = default;
  ~CloudCombiner() = default;

  /**
   * @brief Main function for combining clouds
   * @param clouds Labeled clouds as 2d vec
   * @details Clouds - each element in outer vector corresponds to a different
   * camera, each element in inner vector corresponds to different image used to
   * label
   */
  void CombineClouds(std::vector<std::vector<DefectCloud::Ptr>> clouds,
                     std::vector<std::vector<Eigen::Affine3f>> transforms);

  DefectCloud::Ptr GetCombinedCloud() { return combined_cloud_; }

protected:
  DefectCloud::Ptr combined_cloud_ = boost::make_shared<DefectCloud>();
};

} // namespace inspection