#include <map_quality/MapQuality.h>

#include <math.h>

#include <nlohmann/json.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <beam_utils/filesystem.h>
#include <beam_utils/kdtree.h>
#include <beam_utils/log.h>

namespace map_quality {

MapQuality::MapQuality(const std::string& map_path,
                       const std::string& output_path, double voxel_size_m,
                       int knn, double radius,
                       const std::string& map_output_path)
    : map_path_(map_path),
      output_path_(output_path),
      voxel_size_m_(voxel_size_m),
      knn_(knn),
      radius_(radius),
      map_output_path_(map_output_path) {}

void MapQuality::Run() {
  LoadCloud();
  RunVoxelMapQuality();
  RunStatisticalMapQuality();
  SaveResults();
  BEAM_INFO("Map quality analysis finished successfully!");
}

void MapQuality::RunStatisticalMapQuality() {
  BEAM_INFO("Running statistical map quality analysis");

  PointCloudQuality map_quality;
  pcl::copyPointCloud(map_, map_quality);

  // build kdtree
  beam::KdTree<PointQuality> kdtree(map_quality);

  // iterate through all points, computing statistics and saving to cloud
#pragma omp parallel
#pragma omp for
  for (int pid = 0; pid < map_quality.points.size(); pid++) {
    auto& pt = map_quality.points.at(pid);

    // get mean distances to k nearest neighbors
    std::vector<uint32_t> point_ids;
    std::vector<float> point_distances;
    int num_results =
        kdtree.nearestKSearch(pt, knn_, point_ids, point_distances);
    if (num_results > 0) {
      float sum = 0;
      for (int i = 0; i < num_results; i++) { sum += point_distances[i]; }
      pt.mean_knn_dist = sum / num_results;
    } else {
      pt.mean_knn_dist = -1;
    }

    // calculate densities as described here:
    // cloudcompare.org/doc/wiki/index.php/density
    point_ids.clear();
    point_distances.clear();
    num_results = kdtree.radiusSearch(pt, radius_, point_ids, point_distances);
    if (num_results > 0) {
      float sum = 0;
      for (int i = 0; i < num_results; i++) { sum += point_distances[i]; }
      pt.surface_density = num_results / (M_PI * radius_ * radius_);
      pt.volume_density =
          num_results / (4 / 3 * M_PI * radius_ * radius_ * radius_);
    } else {
      // std::cout << "TEST0\n";
      pt.surface_density = -1;
      pt.volume_density = -1;
    }

    // calculate roughness as described here:
    // cloudcompare.org/doc/wiki/index.php/roughness
    if (num_results > 2) {
      pt.roughness = CalculateRoughness(pt, map_quality, point_ids);
    } else {
      pt.roughness = -1;
    }
  }

  // iterate through cloud and calculate combined statistics
  double sum_mean_knn_dist = 0;
  double sum_surface_density = 0;
  double sum_volume_density = 0;
  double sum_roughness = 0;
  int count_mean_knn_dist = 0;
  int count_surface_density = 0;
  int count_volume_density = 0;
  int count_roughness = 0;
  for (int pid = 0; pid < map_quality.points.size(); pid++) {
    auto& pt = map_quality.points.at(pid);
    if (pt.mean_knn_dist != -1) {
      sum_mean_knn_dist += pt.mean_knn_dist;
      count_mean_knn_dist++;
    }
    if (pt.surface_density != -1) {
      sum_surface_density += pt.surface_density;
      count_surface_density++;
    }
    if (pt.volume_density != -1) {
      sum_volume_density += pt.volume_density;
      count_volume_density++;
    }
    if (pt.roughness != -1) {
      sum_roughness += pt.roughness;
      count_roughness++;
    }
  }
  mean_knn_dist_ = sum_mean_knn_dist / count_mean_knn_dist;
  mean_surface_density_ = sum_surface_density / count_surface_density;
  mean_volume_density_ = sum_volume_density / count_volume_density;
  mean_roughness_ = sum_roughness / count_roughness;

  if (!map_output_path_.empty()) {
    beam::SavePointCloud(map_output_path_, map_quality);
  }
}

double MapQuality::CalculateRoughness(
    const PointQuality& pt, const PointCloudQuality& map,
    const std::vector<uint32_t>& point_ids) const {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // Fill in the cloud data
  for (const uint32_t pid : point_ids) {
    pcl::PointXYZ p_new;
    const auto& p = map.at(pid);
    p_new.x = p.x;
    p_new.y = p.y;
    p_new.z = p.z;
    cloud->push_back(p_new);
  }

  auto coefficients = std::make_shared<pcl::ModelCoefficients>();
  auto inliers = std::make_shared<pcl::PointIndices>();

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(radius_ * 2);
  // seg.setOptimizeCoefficients(true); // Optional

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) { return -1; }

  Eigen::Vector4f coef(coefficients->values[0], coefficients->values[1],
                       coefficients->values[2], coefficients->values[3]);
  Eigen::Vector4f pp(pt.x, pt.y, pt.z, 1);
  double distance_to_plane = pp.dot(coef);
  return distance_to_plane;
}

void MapQuality::RunVoxelMapQuality() {
  BEAM_INFO("Running voxel map quality analysis");
  std::vector<PointCloudPtr> broken_clouds = BreakUpPointCloud(map_);
  for (const PointCloudPtr& cloud : broken_clouds) {
    int num_occupied = CalculateOccupiedVoxels(cloud);
    total_occupied_voxels_ += num_occupied;
  }
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
  J["occupied_voxels_per_m3"] = total_occupied_voxels_ / volume;
  J["percent_empty"] =
      static_cast<double>(empty_voxels) / static_cast<double>(total_voxels);
  J["mean_pts_per_voxel"] = static_cast<double>(map_.size()) /
                            static_cast<double>(total_occupied_voxels_);
  J["mean_knn_dist_mm"] = mean_knn_dist_ * 1000;
  J["mean_surface_density"] = mean_surface_density_;
  J["mean_volume_density"] = mean_volume_density_;
  J["mean_roughness"] = mean_roughness_;

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