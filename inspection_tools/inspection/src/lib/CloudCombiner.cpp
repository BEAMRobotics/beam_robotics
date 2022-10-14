#define PCL_NO_PRECOMPILE

#include <fstream>

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/kdtree.h>

/* must include after kdtree include
   explanation: OpenCV headers must be included after flann headers. kdtree uses
   flann and CloudCombiner has OpenCV somewhere in the chain. */
#include "inspection/CloudCombiner.h"

#include <beam_utils/filesystem.h>
#include <beam_utils/math.h>

namespace inspection {

CloudCombiner::CloudCombiner(float crack_detector_precision,
                             float corrosion_detector_precision,
                             float spall_detector_precision,
                             float delam_detector_precision)
    : crack_detector_precision_(crack_detector_precision),
      corrosion_detector_precision_(corrosion_detector_precision),
      spall_detector_precision_(spall_detector_precision),
      delam_detector_precision_(delam_detector_precision) {
  l0_ = beam::Logit(0.5);
}

void CloudCombiner::CombineClouds(
    const std::unordered_map<std::string, DefectCloudsMapType>& clouds_in_cam,
    const std::unordered_map<std::string, TransformMapType>& Ts_MAP_CAMERA) {
  combined_cloud_ = std::make_shared<DefectCloud>();
  pcl::search::KdTree<BridgePoint> kdtree;

  // store distance from each point its camera used to label
  std::vector<float> distances_to_cam =
      AddFirstCloud(clouds_in_cam, Ts_MAP_CAMERA);

  // iterate through all cameras
  for (const auto& [cam_name, cam_defects_in_cam] : clouds_in_cam) {
    // get current camera transforms and check it exists
    auto cam_Ts_iter = Ts_MAP_CAMERA.find(cam_name);
    if (cam_Ts_iter == Ts_MAP_CAMERA.end()) {
      BEAM_ERROR("cannot find image transforms for camera name {}, not "
                 "combining clouds",
                 cam_name);
      continue;
    }

    // iterate through all images for this camera
    StatsMapType camera_stats;
    for (const auto& [timestamp, img_cloud_in_cam] : cam_defects_in_cam) {
      // skip first cloud that we've already added
      if (cam_name == clouds_in_cam.begin()->first) {
        if (timestamp == clouds_in_cam.begin()->second.begin()->first) {
          CloudStats first_stats;
          first_stats.points_total = img_cloud_in_cam->size();
          first_stats.points_new = img_cloud_in_cam->size();
          first_stats.points_updated = 0;
          first_stats.points_blank = 0;
          camera_stats.emplace(timestamp, first_stats);
          continue;
        }
      }

      // get current image transform and check it exists
      auto img_T_iter = cam_Ts_iter->second.find(timestamp);
      if (img_T_iter == cam_Ts_iter->second.end()) {
        BEAM_ERROR("cannot find image timestamp {} for camera name {}, not "
                   "adding image cloud",
                   timestamp, cam_name);
        continue;
      }

      // convert image cloud to map frame
      const Eigen::Affine3d& T_MAP_CAMERA = img_T_iter->second;
      DefectCloud img_cloud_in_map;
      pcl::transformPointCloud(*img_cloud_in_cam, img_cloud_in_map,
                               T_MAP_CAMERA);

      Eigen::Vector3d img_origin = T_MAP_CAMERA.translation();

      kdtree.setInputCloud(combined_cloud_);
      int points_updated = 0;
      int points_new = 0;
      int points_blank = 0;

      // iterate through all points in the cloud
      for (const auto& search_point : img_cloud_in_map) {
        Eigen::Vector3d search_point_eig(search_point.x, search_point.y,
                                         search_point.z);
        float cur_distance = beam::distance(search_point_eig, img_origin);

        // find closest point (k = 1) in the combined cloud to each point in the
        // current image cloud
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        if (kdtree.nearestKSearch(search_point, 1, pointIdxNKNSearch,
                                  pointNKNSquaredDistance) == 0) {
          // if none found, go add point and move to next point
          combined_cloud_->push_back(search_point);
          distances_to_cam.push_back(cur_distance);
          points_new++;
          continue;
        }

        // else, add the point if it's not in the combined map
        if (pointNKNSquaredDistance[0] > point_distance_threshold_) {
          combined_cloud_->push_back(search_point);
          distances_to_cam.push_back(cur_distance);
          points_new++;
          continue;
        }

        // if the point is closer to the current camera, then update color and
        // defects, otherwise update defects only
        bool point_updated;
        if (cur_distance < distances_to_cam[pointIdxNKNSearch[0]]) {
          distances_to_cam[pointIdxNKNSearch[0]] = cur_distance;
          point_updated = UpdatePoint(pointIdxNKNSearch[0], search_point, true);

        } else {
          point_updated =
              UpdatePoint(pointIdxNKNSearch[0], search_point, false);
        }

        if (point_updated) {
          points_updated++;
        } else {
          points_blank++;
        }
      }

      // add stats
      CloudStats stats;
      stats.points_total = img_cloud_in_map.size();
      stats.points_new = points_new;
      stats.points_updated = points_updated;
      stats.points_blank = points_blank;
      camera_stats.emplace(timestamp, stats);
    }

    stats_.emplace(cam_name, camera_stats);
  }

  BEAM_INFO("Finished combining point clouds - final map is: {} points",
            combined_cloud_->points.size());
}

std::vector<float> CloudCombiner::AddFirstCloud(
    const std::unordered_map<std::string, DefectCloudsMapType>& clouds_in_cam,
    const std::unordered_map<std::string, TransformMapType>& Ts_MAP_CAMERA) {
  std::vector<float> distances_to_cam;
  std::string first_cam_name = clouds_in_cam.begin()->first;
  const DefectCloudsMapType& first_cam_map = clouds_in_cam.begin()->second;
  DefectCloud::Ptr first_cloud_in_cam = first_cam_map.begin()->second;
  int64_t first_cloud_time = first_cam_map.begin()->first;

  if (Ts_MAP_CAMERA.find(first_cam_name) == Ts_MAP_CAMERA.end()) {
    BEAM_CRITICAL("cannot find camera {} in transform map, exiting",
                  first_cam_name);
    throw std::runtime_error{"cannot find camera"};
  }

  if (Ts_MAP_CAMERA.at(first_cam_name).find(first_cloud_time) ==
      Ts_MAP_CAMERA.at(first_cam_name).end()) {
    BEAM_CRITICAL("cannot find image timestamp {} for camera {} in transform "
                  "map, exiting",
                  first_cloud_time, first_cam_name);
    throw std::runtime_error{"cannot find timestamp"};
  }

  const Eigen::Affine3d& T_MAP_IMG1 =
      Ts_MAP_CAMERA.at(first_cam_name).at(first_cloud_time);
  Eigen::Vector3d t_MAP_IMG1 = T_MAP_IMG1.translation();

  // iterate through map
  for (const BridgePoint& p : *first_cloud_in_cam) {
    combined_cloud_->push_back(pcl::transformPoint<BridgePoint>(p, T_MAP_IMG1));
    Eigen::Vector3d search_point(p.x, p.y, p.z);
    distances_to_cam.push_back(beam::distance(search_point, t_MAP_IMG1));
  }

  return distances_to_cam;
}

bool CloudCombiner::UpdatePoint(int point_id_to_update,
                                const BridgePoint& new_point,
                                bool update_color) const {
  bool point_updated = false;
  BridgePoint& p_to_update = combined_cloud_->points.at(point_id_to_update);

  if (new_point.crack != 0) {
    double p_prev = (p_to_update.crack == 0) ? 0.5 : p_to_update.crack;
    p_to_update.crack =
        beam::BayesianLogitUpdate(crack_detector_precision_, l0_, p_prev);
    point_updated = true;
  }
  if (new_point.spall != 0) {
    double p_prev = (p_to_update.spall == 0) ? 0.5 : p_to_update.spall;
    p_to_update.spall =
        beam::BayesianLogitUpdate(spall_detector_precision_, l0_, p_prev);
    point_updated = true;
  }
  if (new_point.corrosion != 0) {
    double p_prev = (p_to_update.corrosion == 0) ? 0.5 : p_to_update.corrosion;
    p_to_update.corrosion =
        beam::BayesianLogitUpdate(corrosion_detector_precision_, l0_, p_prev);
    point_updated = true;
  }
  if (new_point.delam != 0) {
    double p_prev = (p_to_update.delam == 0) ? 0.5 : p_to_update.delam;
    p_to_update.delam =
        beam::BayesianLogitUpdate(delam_detector_precision_, l0_, p_prev);
    point_updated = true;
  }

  if (!update_color) { return point_updated; }

  if (new_point.x + new_point.y + new_point.z != 0) {
    p_to_update.x = new_point.x;
    p_to_update.y = new_point.y;
    p_to_update.z = new_point.z;
    p_to_update.r = new_point.r;
    p_to_update.g = new_point.g;
    p_to_update.b = new_point.b;
    point_updated = true;
  }

  if (new_point.thermal != 0) {
    p_to_update.thermal = new_point.thermal;
    point_updated = true;
  }

  return point_updated;
}

void CloudCombiner::OutputStatistics(const std::string& output_file) {
  if (output_file.empty()) {
    BEAM_INFO("no output directory input, not saving statistics");
    return;
  }

  BEAM_INFO("Saving cloud combiner summary to: {}", output_file);

  if (!beam::HasExtension(output_file, ".json")) {
    BEAM_ERROR("cannot save stats, file must be a json");
    return;
  }

  std::unordered_map<std::string, std::unordered_map<int, nlohmann::json>>
      cam_jsons;
  for (const auto& [cam_name, cam_stats] : stats_) {
    std::unordered_map<int, nlohmann::json> img_jsons;
    for (const auto& [timestamp, img_stats] : cam_stats) {
      nlohmann::json stats_json;
      stats_json["points_total"] = img_stats.points_total;
      stats_json["points_new"] = img_stats.points_new;
      stats_json["points_updated"] = img_stats.points_updated;
      stats_json["points_blank"] = img_stats.points_blank;
      img_jsons.emplace(timestamp, stats_json);
    }
    cam_jsons.emplace(cam_name, img_jsons);
  }

  std::ofstream filejson(output_file);
  filejson << std::setw(4) << cam_jsons << std::endl;
}

} // end namespace inspection
