#define PCL_NO_PRECOMPILE

#include "inspection/CloudCombiner.h"
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/kdtree.h>

namespace inspection {

void CloudCombiner::CombineClouds(
    std::vector<std::vector<pcl::PointCloud<beam_containers::PointBridge>::Ptr>>
        clouds,
    std::vector<std::vector<Eigen::Affine3f>> transforms) {
  pcl::search::KdTree<BridgePoint> kdtree;
  BridgePoint bp = {0, 0, 0};
  combined_cloud_->points.push_back(bp);

  // Iterate through and successfully add point clouds to final
  int num_cams = clouds.size();
  std::vector<float> closest_camera_pose;
  for (int cam = 0; cam < num_cams; cam++) {
    std::vector<Eigen::Affine3f> camera_tfs = transforms[cam];
    int i = 0;
    for (const auto& pc : clouds[cam]) {
      Eigen::Affine3f img_to_map = camera_tfs[i++];
      Eigen::Vector3f tf_origin = img_to_map.translation();
      int replaced_points = 0;
      kdtree.setInputCloud(combined_cloud_);
      for (const auto& search_point : *pc) {
        int K = 1;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        if (kdtree.nearestKSearch(search_point, K, pointIdxNKNSearch,
                                  pointNKNSquaredDistance) > 0) {
          // distance from point to the current camera
          float cur_distance = beam::distance(search_point, tf_origin);
          // If this point isn't already in our cloud, add it
          if (pointNKNSquaredDistance[0] > 0.001) {
            combined_cloud_->push_back(search_point);
            closest_camera_pose.push_back(cur_distance);
          } else {
            // if the point is closer to the current camera, then replace
            if (cur_distance < closest_camera_pose[pointIdxNKNSearch[0]]) {
              replaced_points++;
              closest_camera_pose[pointIdxNKNSearch[0]] = cur_distance;
              combined_cloud_->points[pointIdxNKNSearch[0]] = search_point;
            }
          }
        }
      }
      std::stringstream ss;
      ss << "[" << tf_origin[0] << "," << tf_origin[1] << "," << tf_origin[2]
         << "]";
      std::string camera_pose = ss.str();
      BEAM_INFO("Camera #{} Pose: {}, Number of points replaced: {}", cam + 1,
                camera_pose, replaced_points);
    }
  }
  BEAM_INFO("Finished combining point clouds - final map is: {} points",
            combined_cloud_->points.size());
}

} // end namespace inspection
