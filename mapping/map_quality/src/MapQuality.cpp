#include <map_quality/MapQuality.h>

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include <beam_utils/log.h>

namespace map_quality {

MapQuality::MapQuality(const std::string& map_path,
                       const std::string& output_path)
    : map_path_(map_path), output_path_(output_path) {}

void MapQuality::Run() {
  LoadCloud();
  pcl::getMinMax3D(map_, min_, max_);
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
}

void MapQuality::SaveResults() {
  BEAM_INFO("Saving results to: {}", output_path_);

  double wx = max_.x - min_.x;
  double wy = max_.y - min_.y;
  double wz = max_.z - min_.z;
  double volume = wx * wy * wz;

  std::cout << "total_occupied_voxels_: " << total_occupied_voxels_ << "\n";
  std::cout << "number of map points: " << map_.size() << "\n";
  std::cout << "volume: " << volume << " m^3 \n";
  std::cout << "occupied voxels per cubic meter: "
            << total_occupied_voxels_ / volume << "\n";

  // todo output to json
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