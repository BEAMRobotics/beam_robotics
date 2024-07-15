#include <map_quality/MapQuality.h>

#include <filesystem>
#include <math.h>

#include <nlohmann/json.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <beam_utils/angles.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/kdtree.h>
#include <beam_utils/log.h>

namespace map_quality {

double CalculateRoughness(const PointQuality& pt, const PointCloudQuality& map,
                          const std::vector<uint32_t>& point_ids,
                          double seg_dist_thresh) {
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
  seg.setDistanceThreshold(seg_dist_thresh);
  // seg.setOptimizeCoefficients(true); // Optional

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) { return -1; }

  Eigen::Vector4f coef(coefficients->values[0], coefficients->values[1],
                       coefficients->values[2], coefficients->values[3]);
  Eigen::Vector4f pp(pt.x, pt.y, pt.z, 1);
  double distance_to_plane = pp.dot(coef);
  return std::abs(distance_to_plane);
}

MapQuality::MapQuality(const std::string& map_path,
                       const std::string& output_path,
                       const std::string& config_path,
                       const std::string& map_output_path,
                       const std::string& planes_output_path)
    : map_path_(map_path),
      output_path_(output_path),
      map_output_path_(map_output_path),
      planes_output_path_(planes_output_path) {
  LoadConfig(config_path);
  LoadCloud();
}

void MapQuality::LoadConfig(const std::string& config_path) {
  params_ = Params();
  if (config_path.empty()) {
    BEAM_INFO("No config path provided, using default parameters.");
    return;
  }

  BEAM_INFO("Loading MapQuality config file: {}", config_path);
  nlohmann::json J;
  if (!beam::ReadJson(config_path, J)) {
    throw std::runtime_error{"Invalid json"};
  }

  beam::ValidateJsonKeysOrThrow(
      {"voxel_size_m", "knn", "radius", "use_euc_clustering", "max_clusters_h",
       "max_clusters_v", "min_cluster_size", "clustering_dist_m",
       "max_horz_planes", "max_vert_planes", "plane_seg_dist_thresh",
       "plane_ang_thres_deg"},
      J);

  params_.voxel_size_m = J["voxel_size_m"];
  params_.knn = J["knn"];
  params_.radius = J["radius"];
  params_.use_euc_clustering = J["use_euc_clustering"];
  params_.max_clusters_h = J["max_clusters_h"];
  params_.max_clusters_v = J["max_clusters_v"];
  params_.min_cluster_size = J["min_cluster_size"];
  params_.clustering_dist_m = J["clustering_dist_m"];
  params_.max_horz_planes = J["max_horz_planes"];
  params_.max_vert_planes = J["max_vert_planes"];
  params_.plane_seg_dist_thresh = J["plane_seg_dist_thresh"];
  params_.plane_ang_thres_deg = J["plane_ang_thres_deg"];
}

void MapQuality::RunAll() {
  ComputeVoxelStatistics();
  ComputerNeighborhoodStatistics();
  ComputePlaneStatistics();
  SaveResults();
  BEAM_INFO("Map quality analysis finished successfully!");
}

void MapQuality::ComputePlaneStatistics() {
  BEAM_INFO("Running plane fitting statistics");

  Eigen::Vector3f z_axis(0, 0, 1);

  // setup plane fitter perpendicular to  z-axis
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(params_.plane_seg_dist_thresh);
  seg.setAxis(z_axis); // we look for planes perpendicular to z
  seg.setEpsAngle(beam::Deg2Rad(params_.plane_ang_thres_deg));
  seg.setOptimizeCoefficients(true); // Optional

  // first get all horizontal planes
  auto current_map = std::make_shared<PointCloud>(map_);
  BEAM_INFO("Fitting horizontal planes");
  std::vector<PointCloudPtr> planes_h;
  for (int i = 0; i < params_.max_horz_planes; i++) {
    BEAM_INFO("Working on plane {}", i);
    // fit plane
    auto coefficients = std::make_shared<pcl::ModelCoefficients>();
    auto inliers = std::make_shared<pcl::PointIndices>();
    seg.setInputCloud(current_map);
    seg.segment(*inliers, *coefficients);

    // filter out points, then repeat
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(current_map);
    extract.setIndices(inliers);
    planes_h.push_back(std::make_shared<PointCloud>());
    extract.setNegative(false);
    extract.filter(*planes_h.back()); // add to set of planes
    extract.setNegative(true);
    extract.filter(*current_map); // reduce original map
  }
  if (!planes_output_path_.empty()) {
    BEAM_INFO("Clearing planes output path: {}", planes_output_path_);
    std::filesystem::remove_all(planes_output_path_);
    std::filesystem::create_directory(planes_output_path_);
    BEAM_INFO("Saving {} horizontal planes to {}", planes_h.size(),
              planes_output_path_);
    for (int i = 0; i < planes_h.size(); i++) {
      std::string filepath =
          beam::CombinePaths(planes_output_path_,
                             "plane_horizontal_" + std::to_string(i) + ".pcd");
      beam::SavePointCloud(filepath, *planes_h.at(i));
    }
  }

  // setup segmentation model that segments plane parallel to z axis
  pcl::SACSegmentation<pcl::PointXYZ> segv;
  segv.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  segv.setMethodType(pcl::SAC_RANSAC);
  segv.setDistanceThreshold(params_.plane_seg_dist_thresh);
  segv.setAxis(z_axis); // we look for planes parallel to z
  segv.setEpsAngle(beam::Deg2Rad(params_.plane_ang_thres_deg));
  seg.setOptimizeCoefficients(true); // Optional

  // get all vertical planes
  BEAM_INFO("Fitting vertical planes");
  std::vector<PointCloudPtr> planes_v;
  for (int i = 0; i < params_.max_vert_planes; i++) {
    BEAM_INFO("Working on plane {}", i);

    // fit plane
    auto coefficients = std::make_shared<pcl::ModelCoefficients>();
    auto inliers = std::make_shared<pcl::PointIndices>();
    segv.setInputCloud(current_map);
    segv.segment(*inliers, *coefficients);

    // filter out points, then repeat
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(current_map);
    extract.setIndices(inliers);
    planes_v.push_back(std::make_shared<PointCloud>());
    extract.setNegative(false);
    extract.filter(*planes_v.back()); // add to set of planes
    extract.setNegative(true);
    extract.filter(*current_map); // reduce original map
  }
  if (!planes_output_path_.empty()) {
    BEAM_INFO("Saving {} vertical planes to {}", planes_v.size(),
              planes_output_path_);
    for (int i = 0; i < planes_v.size(); i++) {
      std::string filepath = beam::CombinePaths(
          planes_output_path_, "plane_vertical_" + std::to_string(i) + ".pcd");
      beam::SavePointCloud(filepath, *planes_v.at(i));
    }
  }

  std::vector<PointCloudPtr> planes;
  if (params_.use_euc_clustering) {
    auto planes_v_filtered =
        FilterPlanesByClustering(planes_v, params_.max_clusters_v);
    for (const auto& pc : planes_v_filtered) { planes.push_back(pc); }
    auto planes_h_filtered =
        FilterPlanesByClustering(planes_h, params_.max_clusters_h);
    for (const auto& pc : planes_h_filtered) { planes.push_back(pc); }
  }

  if (!planes_output_path_.empty()) {
    BEAM_INFO("Saving {} filtered planes to {}", planes.size(),
              planes_output_path_);
    for (int i = 0; i < planes.size(); i++) {
      std::string filepath = beam::CombinePaths(
          planes_output_path_, "plane_filtered_" + std::to_string(i) + ".pcd");
      beam::SavePointCloud(filepath, *planes.at(i));
    }
  }

  CalculateMeanDistanceToPlanes(planes);
}

void MapQuality::CalculateMeanDistanceToPlanes(
    const std::vector<PointCloudPtr>& planes) {
  double distance_sum{0};
  int count{0};
  for (const auto& plane : planes) {
    auto coefficients = std::make_shared<pcl::ModelCoefficients>();
    auto inliers = std::make_shared<pcl::PointIndices>();

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(params_.plane_seg_dist_thresh);
    // seg.setOptimizeCoefficients(true); // Optional

    seg.setInputCloud(plane);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) { continue; }

    Eigen::Vector4f coef(coefficients->values[0], coefficients->values[1],
                         coefficients->values[2], coefficients->values[3]);

    // iterate through all points in plane and calculate distance to plane
    for (const auto& pt : plane->points) {
      Eigen::Vector4f pp(pt.x, pt.y, pt.z, 1);
      double distance_to_plane = pp.dot(coef);
      distance_sum += std::abs(distance_to_plane);
      count++;
    }
  }
  mean_distance_to_planes_ = distance_sum / count;
  BEAM_INFO("Calculated a mean point distance to plane of {}, using {} points "
            "from {} planes.",
            mean_distance_to_planes_, count, planes.size());
}

std::vector<PointCloudPtr> MapQuality::FilterPlanesByClustering(
    const std::vector<PointCloudPtr>& clouds_in, int max_clusters) const {
  // cluster size -> pair <cloud id, point ids>
  std::map<size_t, std::pair<int, pcl::PointIndices>, std::greater<int>>
      clusters;
  for (int i = 0; i < clouds_in.size(); i++) {
    const auto& cloud = clouds_in.at(i);
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(params_.clustering_dist_m);
    ec.setMinClusterSize(params_.min_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto& ids : cluster_indices) {
      auto cloud_id_to_point_ids = std::make_pair(i, ids);
      clusters.emplace(ids.indices.size(), cloud_id_to_point_ids);
    }
  }

  std::vector<PointCloudPtr> clouds_out;
  for (const auto& [cluster_size, cloud_id_to_point_ids] : clusters) {
    if (clouds_out.size() >= max_clusters) { break; }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    const PointCloudPtr& cloud = clouds_in.at(cloud_id_to_point_ids.first);
    const auto indices =
        std::make_shared<pcl::PointIndices>(cloud_id_to_point_ids.second);
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    clouds_out.push_back(std::make_shared<PointCloud>());
    extract.filter(*clouds_out.back());
  }
  return clouds_out;
}

void MapQuality::ComputerNeighborhoodStatistics() {
  BEAM_INFO("Running neighborhood statistical map quality analysis");
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
        kdtree.nearestKSearch(pt, params_.knn, point_ids, point_distances);
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
    num_results =
        kdtree.radiusSearch(pt, params_.radius, point_ids, point_distances);
    if (num_results > 0) {
      float sum = 0;
      for (int i = 0; i < num_results; i++) { sum += point_distances[i]; }
      pt.surface_density =
          num_results / (M_PI * params_.radius * params_.radius);
      pt.volume_density = num_results / (4 / 3 * M_PI * params_.radius *
                                         params_.radius * params_.radius);
    } else {
      // std::cout << "TEST0\n";
      pt.surface_density = -1;
      pt.volume_density = -1;
    }

    // calculate roughness as described here:
    // cloudcompare.org/doc/wiki/index.php/roughness
    if (num_results > 2) {
      pt.roughness =
          CalculateRoughness(pt, map_quality, point_ids, params_.radius * 2);
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

void MapQuality::ComputeVoxelStatistics() {
  BEAM_INFO("Running voxel statistics map quality analysis");
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
  BEAM_INFO("Done loading input map of size: {}", map_.size());
}

void MapQuality::SaveResults() {
  BEAM_INFO("Saving results to: {}", output_path_);

  nlohmann::json J;
  if (total_occupied_voxels_ != 0) {
    pcl::getMinMax3D(map_, min_, max_);
    double wx = max_.x - min_.x;
    double wy = max_.y - min_.y;
    double wz = max_.z - min_.z;
    double volume = wx * wy * wz;
    double voxel_volume =
        params_.voxel_size_m * params_.voxel_size_m * params_.voxel_size_m;
    int64_t total_voxels = static_cast<int64_t>(volume / voxel_volume);
    int64_t empty_voxels = total_voxels - total_occupied_voxels_;
    J["voxel-volume_m3"] = volume;
    J["voxel-total_voxels"] = total_voxels;
    J["voxel-occupied_voxels"] = total_occupied_voxels_;
    J["voxel-empty_voxels"] = empty_voxels;
    J["voxel-map_points"] = map_.size();
    J["voxel-occupied_voxels_per_m3"] = total_occupied_voxels_ / volume;
    J["voxel-percent_empty"] =
        static_cast<double>(empty_voxels) / static_cast<double>(total_voxels);
    J["voxel-mean_pts_per_voxel"] = static_cast<double>(map_.size()) /
                                    static_cast<double>(total_occupied_voxels_);
  }
  if (mean_knn_dist_ != 0) {
    J["statistics-mean_knn_dist_mm"] = mean_knn_dist_ * 1000;
    J["statistics-mean_surface_density"] = mean_surface_density_;
    J["statistics-mean_volume_density"] = mean_volume_density_;
    J["statistics-mean_roughness"] = mean_roughness_;
  }

  if (mean_distance_to_planes_ != 0) {
    J["planes-mean_dist_to_planes"] = mean_distance_to_planes_;
  }

  std::ofstream file(output_path_);
  file << std::setw(4) << J << std::endl;
  BEAM_INFO("Done saving results.");
}

int MapQuality::CalculateOccupiedVoxels(const PointCloudPtr& cloud) const {
  PointCloud downsampled_points;
  pcl::VoxelGrid<pcl::PointXYZ> downsampler;
  downsampler.setLeafSize(params_.voxel_size_m, params_.voxel_size_m,
                          params_.voxel_size_m);
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
      static_cast<uint64_t>((axis_dimensions[0] / params_.voxel_size_m) + 1);
  uint64_t voxel_count_y =
      static_cast<uint64_t>((axis_dimensions[1] / params_.voxel_size_m) + 1);
  uint64_t voxel_count_z =
      static_cast<uint64_t>((axis_dimensions[2] / params_.voxel_size_m) + 1);
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