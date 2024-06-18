#pragma once

#include <beam_utils/pointclouds.h>

// Create point types of different lidars
// Velodyne
struct PointQuality {
  PCL_ADD_POINT4D
  float mean_knn_dist;
  float surface_density;
  float volume_density;
  float roughness;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

using PointCloudQuality = pcl::PointCloud<PointQuality>;

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointQuality, 
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, mean_knn_dist, mean_knn_dist)
    (float, surface_density, surface_density)
    (float, volume_density, volume_density)
    (float, roughness, roughness)
    )
// clang-format on

namespace map_quality {

double CalculateRoughness(const PointQuality& pt, const PointCloudQuality& map,
                          const std::vector<uint32_t>& point_ids,
                          double seg_dist_thresh);

class MapQuality {
public:
  MapQuality() = delete;

  MapQuality(const std::string& map_path, const std::string& output_path,
             double voxel_size_m = 0.01, int knn = 10, double radius = 0.1,
             const std::string& map_output_path = "");

  void Run();

private:
  void LoadCloud();

  void RunVoxelMapQuality();

  void RunStatisticalMapQuality();

  void SaveResults();

  int CalculateOccupiedVoxels(const PointCloudPtr& cloud) const;

  /**
   * @brief Private method for breaking up clouds.
   * @param input_cloud Reference to the cloud to be broken up.
   * @return A vector of the broken up clouds.
   */
  std::vector<PointCloudPtr>
      BreakUpPointCloud(const PointCloud& input_cloud) const;

  /**
   * @brief Private method for splitting one cloud into two
   * format.
   * @param input_cloud Cloud to be split.
   * @param max_axis The axis of greatest magnitude to split the cloud.
   * @return A pair of the split clouds.
   */
  std::pair<PointCloudPtr, PointCloudPtr>
      SplitCloudInTwo(const PointCloud& input_cloud, int max_axis) const;

  // data storage
  std::string map_path_;
  std::string output_path_;
  PointCloud map_;

  // params:
  double voxel_size_m_;
  int knn_;
  double radius_;
  std::string map_output_path_;

  // results
  pcl::PointXYZ min_;
  pcl::PointXYZ max_;
  int total_occupied_voxels_{0};
  double mean_knn_dist_;
  double mean_surface_density_;
  double mean_volume_density_;
  double mean_roughness_;
};

} // namespace map_quality