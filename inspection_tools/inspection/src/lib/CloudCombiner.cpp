#include "inspection/CloudCombiner.h"

#include <fstream>

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>

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

  // store each point id added back to the final map and its associated distance to camera
  std::unordered_map<uint64_t, double> mapIdToCamDistance;
  
  // store link between original map ID, and new combined map ID
  std::unordered_map<uint64_t, double> origMapIdToCombMapId;

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
      // get current image transform and check it exists
      auto img_T_iter = cam_Ts_iter->second.find(timestamp);
      if (img_T_iter == cam_Ts_iter->second.end()) {
        BEAM_ERROR("cannot find image timestamp {} for camera name {}, not "
                   "adding image cloud",
                   timestamp, cam_name);
        continue;
      }

      const Eigen::Affine3d& T_MAP_CAMERA = img_T_iter->second;

      int points_updated = 0;
      int points_new = 0;
      int points_ignored = 0;
      for (const auto& pt_in_cam : *img_cloud_in_cam) {
        Eigen::Vector3d pt_in_cam_eig(pt_in_cam.x, pt_in_cam.y, pt_in_cam.z);
        double dist_cam_to_pt = pt_in_cam_eig.norm();
        auto iter = mapIdToCamDistance.find(pt_in_cam.map_point_id); 
        
        // if point ID hasn't been added to map, then add it
        if(iter == mapIdToCamDistance.end()){
          combined_cloud_->push_back(pcl::transformPoint<BridgePoint>(pt_in_cam, T_MAP_CAMERA));
          mapIdToCamDistance.emplace(pt_in_cam.map_point_id, dist_cam_to_pt);
          origMapIdToCombMapId.emplace(pt_in_cam.map_point_id, combined_cloud_->size() - 1);
          points_new++;
          continue;
        } 
        
        // else, check if new distance to camera is less than the previous stored point
        bool point_updated;
        if(dist_cam_to_pt < iter->second){
          // if the point is closer to the current camera, then update color and
          // defects
          point_updated = UpdatePoint(origMapIdToCombMapId.at(pt_in_cam.map_point_id), pt_in_cam, true);
          iter->second = dist_cam_to_pt;
        } else {
          // if point is farther from this camera, update defects only
          point_updated = UpdatePoint(origMapIdToCombMapId.at(pt_in_cam.map_point_id), pt_in_cam, false);
        }

        if (point_updated) {
          points_updated++;
        } else {
          points_ignored++;
        }
      }

      // add stats
      CloudStats stats;
      stats.points_total = img_cloud_in_cam->size();
      stats.points_new = points_new;
      stats.points_updated = points_updated;
      stats.points_ignored = points_ignored;
      camera_stats.emplace(timestamp, stats);
    }

    stats_.emplace(cam_name, camera_stats);
  }

  BEAM_INFO("Finished combining point clouds - final map is: {} points",
            combined_cloud_->points.size());
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

  p_to_update.r = new_point.r;
  p_to_update.g = new_point.g;
  p_to_update.b = new_point.b;

  if (new_point.thermal != 0) {
    p_to_update.thermal = new_point.thermal;
  }

  return true;
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

  std::unordered_map<std::string, std::unordered_map<std::string, nlohmann::json>>
      cam_jsons;
  for (const auto& [cam_name, cam_stats] : stats_) {
    std::unordered_map<std::string, nlohmann::json> img_jsons;
    for (const auto& [timestamp, img_stats] : cam_stats) {
      nlohmann::json stats_json;
      stats_json["points_total"] = img_stats.points_total;
      stats_json["points_new"] = img_stats.points_new;
      stats_json["points_updated"] = img_stats.points_updated;
      stats_json["points_ignored"] = img_stats.points_ignored;
      img_jsons.emplace(std::to_string(timestamp), stats_json);
    }
    cam_jsons.emplace(cam_name, img_jsons);
  }

  std::ofstream filejson(output_file);
  filejson << std::setw(4) << cam_jsons << std::endl;
  BEAM_INFO("Done saving summary");
}

} // end namespace inspection
