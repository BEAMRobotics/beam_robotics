#pragma once

#include <beam_utils/pointclouds.h>

namespace map_quality {

class MapQuality {
public:
  MapQuality() = delete;

  MapQuality(const std::string& map_path, const std::string& output_path,
             double voxel_size_m = 0.01, int knn = 10);

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

  // results
  pcl::PointXYZ min_;
  pcl::PointXYZ max_;
  int total_occupied_voxels_{0};
  double mean_knn_dist_;
};

} // namespace map_quality