#pragma once

#include <beam_containers/PointBridge.h>
#include <beam_utils/math.h>
#include <beam_utils/time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

namespace inspection {

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;
using BridgePoint = beam_containers::PointBridge;
using DefectCloud = pcl::PointCloud<beam_containers::PointBridge>;
using PointCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

// map: image timestamp in Ns -> defect cloud in map frame
using DefectCloudsMapType = std::unordered_map<int64_t, DefectCloud::Ptr>;

// map: image timestamp in Ns -> T_MAP_CAMERA
using TransformMapType = std::unordered_map<int64_t, Eigen::Affine3d>;

struct CloudStats {
  int points_total;
  int points_new;
  int points_updated;
  int points_blank;
};

// map: image timestamp in Ns -> CloudStats
using StatsMapType = std::unordered_map<int64_t, CloudStats>;

/**
 * @brief Class for combining labeled clouds from each image / camera labeled
 * cloud. Since different images may labels subsets of the same points, this
 * combiner uses the point details that captured that point from the closest
 * position. I.e., we take the closest image to label the point when multiple
 * images labeled the same point. However, for the case where one image labeled
 * the RGB and another one or more defects, we only replace the point details if
 * the new point has non-zero values. I.e., if img1 only labeled color only, and
 * img2 labeled cracks only, then we keep the col from img1 and the crack
 * details from img2.
 *
 * NOTE: defects are only labeled if detected as a defect, we do not store
 * negative detections. In other words, if a defect mask "labels" as set of
 * points, we only update the defect fields of the points that were detected as
 * being a crack. It is too hard to know for sure if the image saw that defect
 * point or not so we cannot use the false detections to say that a point does
 * not belong to a defect type. This also makes us more conservative in the
 * sense we prefer to detect more false positives
 *
 * NOTE: defect probabilities are updated using Bayesian log odds updating.
 * Therefore, probabilities of a crack being a crack will continue to increase
 * each time a point is labeled as a crack
 */
class CloudCombiner {
public:
  CloudCombiner(float crack_detector_precision,
                float corrosion_detector_precision,
                float spall_detector_precision, float delam_detector_precision);
  ~CloudCombiner() = default;

  /**
   * @brief Main function for combining clouds. This fills in the internal cloud
   * which can then be retrieved with GetCombinedCloud()
   * @param clouds: map cam name -> [map timestamp -> defect cloud]
   * @param Ts_MAP_CAMERA: map cam name -> [map timestamp -> T_MAP_CAMERA]
   */
  void CombineClouds(
      const std::unordered_map<std::string, DefectCloudsMapType>& clouds,
      const std::unordered_map<std::string, TransformMapType>& Ts_MAP_CAMERA);

  /**
   * @brief return final combined cloud
   * @return DefectCloud::Ptr
   */
  DefectCloud::Ptr GetCombinedCloud() { return combined_cloud_; }

  /**
   * @brief save stats to a json file
   * @param output_file must be .json
   */
  void OutputStatistics(const std::string& output_file);

protected:
  /**
   * @brief adds all points from the first cloud in clouds to the map
   * @param clouds
   * @param Ts_MAP_CAMERA
   * @return std::vector<float>
   */
  std::vector<float> AddFirstCloud(
      const std::unordered_map<std::string, DefectCloudsMapType>& clouds,
      const std::unordered_map<std::string, TransformMapType>& Ts_MAP_CAMERA);

  /**
   * @brief update contents of one point with new point. This takes into
   * account that some points may have different labels than others. The
   * replacement logic is as follows:
   *
   * - if the new point has no RGB, then we keep the original point's XYZ and
   * RGB. This is because we want to prioritize XYZ from the camera providing
   * the RGB
   * - for all defect fields, we only update the point probability if that field
   * in the new point isn't 0
   *
   * @param point_id_to_update
   * @param new_point
   * @param update_color if set to false, it will only update defect fields
   * @return true if any field was updated, if not then the point is probably
   * unlabeled
   */
  bool UpdatePoint(int point_id_to_update, const BridgePoint& new_point,
                   bool update_color) const;

  DefectCloud::Ptr combined_cloud_ = std::make_shared<DefectCloud>();

  // distance at which we consider two points to be identical
  float point_distance_threshold_{0.001}; // 1 mm

  // map camera name -> StatsMapType (see def above)
  std::unordered_map<std::string, StatsMapType> stats_;

  float crack_detector_precision_;
  float corrosion_detector_precision_;
  float spall_detector_precision_;
  float delam_detector_precision_;

  double l0_; // log odds of 0.5, used for Bayesian logit updates
};

} // namespace inspection