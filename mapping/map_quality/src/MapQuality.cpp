#include <map_quality/MapQuality.h>

#include <nlohmann/json.hpp>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include <beam_utils/filesystem.h>
#include <beam_utils/log.h>

namespace map_quality {

MapQuality::MapQuality(const std::string& map_path,
                       const std::string& output_path, double voxel_size_m)
    : map_path_(map_path),
      output_path_(output_path),
      voxel_size_m_(voxel_size_m) {}

void MapQuality::Run() {
  LoadCloud();
  std::vector<PointCloudPtr> broken_clouds = BreakUpPointCloud(map_);
  for (const PointCloudPtr& cloud : broken_clouds) {
    int num_occupied = CalculateOccupiedVoxels(cloud);
    total_occupied_voxels_ += num_occupied;
  }
  SaveResults();
  BEAM_INFO("Map quality analysis finished successfully!");
}

void MapQuality::LoadCloud() {
  BEAM_INFO("Loading input map: {}", map_path_);
  pcl::io::loadPCDFile(map_path_, map_);
  if (map_.empty()) {
    BEAM_ERROR("empty input map.");
    throw std::runtime_error{"empty input map"};
  }
  pcl::getMinMax3D(map_, min_, max_);
}

void MapQuality::SaveResults() {
  BEAM_INFO("Saving results to: {}", output_path_);

  nlohmann::json J;

  double wx = max_.x - min_.x;
  double wy = max_.y - min_.y;
  double wz = max_.z - min_.z;
  double volume = wx * wy * wz;
  double voxel_volume = voxel_size_m_ * voxel_size_m_ * voxel_size_m_;
  int64_t total_voxels = static_cast<int64_t>(volume / voxel_volume);
  int64_t empty_voxels = total_voxels - total_occupied_voxels_;
  J["volume_m3"] = volume;
  J["total_voxels"] = total_voxels;
  J["occupied_voxels"] = total_occupied_voxels_;
  J["empty_voxels"] = empty_voxels;
  J["map_points"] = map_.size();
  J["voxels_per_m3"] = total_occupied_voxels_ / volume;
  J["percent_empty"] =
      static_cast<double>(empty_voxels) / static_cast<double>(total_voxels);

  std::ofstream file(output_path_);
  file << std::setw(4) << J << std::endl;
}

int MapQuality::CalculateOccupiedVoxels(const PointCloudPtr& cloud) const {
  PointCloud downsampled_points;
  pcl::VoxelGrid<pcl::PointXYZ> downsampler;
  downsampler.setLeafSize(voxel_size_m_, voxel_size_m_, voxel_size_m_);
  downsampler.setInputCloud(cloud);
  downsampler.filter(downsampled_points);
  return downsampled_points.size();
}

std::vector<PointCloudPtr>
    MapQuality::BreakUpPointCloud(const PointCloud& input_cloud) const {
  // Determine if integer overflow will occur.
  pcl::PointXYZ min;
  pcl::PointXYZ max;
  pcl::getMinMax3D(input_cloud, min, max);
  Eigen::Vector3f axis_dimensions(std::abs(max.x - min.x),
                                  std::abs(max.y - min.y),
                                  std::abs(max.z - min.z));
  uint64_t voxel_count_x =
      static_cast<uint64_t>((axis_dimensions[0] / voxel_size_m_) + 1);
  uint64_t voxel_count_y =
      static_cast<uint64_t>((axis_dimensions[1] / voxel_size_m_) + 1);
  uint64_t voxel_count_z =
      static_cast<uint64_t>((axis_dimensions[2] / voxel_size_m_) + 1);
  if ((voxel_count_x * voxel_count_y * voxel_count_z) >
      static_cast<uint64_t>(std::numeric_limits<std::int32_t>::max())) {
    // Split cloud along max axis until integer overflow does not occur.
    int max_axis;
    axis_dimensions.maxCoeff(&max_axis);
    std::pair<PointCloudPtr, PointCloudPtr> split_clouds =
        SplitCloudInTwo(input_cloud, max_axis);

    // Recursively call BreakUpPointCloud on each split.
    std::vector<PointCloudPtr> cloud_1 =
        BreakUpPointCloud(*(split_clouds.first));
    std::vector<PointCloudPtr> cloud_2 =
        BreakUpPointCloud(*(split_clouds.second));
    cloud_1.insert(cloud_1.end(), cloud_2.begin(), cloud_2.end());
    return cloud_1;
  } else {
    // No overflow, return output cloud pointers in vector.
    PointCloudPtr output_cloud = std::make_shared<PointCloud>(input_cloud);
    return std::vector<PointCloudPtr>{output_cloud};
  }
}

std::pair<PointCloudPtr, PointCloudPtr>
    MapQuality::SplitCloudInTwo(const PointCloud& input_cloud,
                                int max_axis) const {
  // Get mid point of cloud.
  pcl::PointXYZ min;
  pcl::PointXYZ max;
  pcl::getMinMax3D(input_cloud, min, max);
  Eigen::Vector3f eigen_min(min.x, min.y, min.z);
  Eigen::Vector3f eigen_max(max.x, max.y, max.z);
  float midpoint = (eigen_max[max_axis] + eigen_min[max_axis]) / 2;

  // Create clouds for points (1) < than midpoint (2) > than midpoint.
  PointCloudPtr cloud_1 = std::make_shared<PointCloud>();
  PointCloudPtr cloud_2 = std::make_shared<PointCloud>();
  for (const auto& point : input_cloud) {
    Eigen::Vector3f eigen_point(point.x, point.y, point.z);
    if (eigen_point[max_axis] < midpoint) {
      cloud_1->push_back(point);
    } else {
      cloud_2->push_back(point);
    }
  }
  return std::pair<PointCloudPtr, PointCloudPtr>(cloud_1, cloud_2);
}

} // namespace map_quality