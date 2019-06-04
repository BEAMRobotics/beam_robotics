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
  for (int cam = 0; cam < num_cams; cam++) {
    std::vector<Eigen::Affine3f> camera_tfs = transforms[cam];
    int i = 1;
    Eigen::Vector3f prev_origin(0, 0, 0);
    for (const auto& pc : clouds[cam]) {
      Eigen::Affine3f img_to_map = camera_tfs[i++];
      Eigen::Vector4f origin(0, 0, 0, 1);
      Eigen::Vector3f tf_origin = (img_to_map * origin).head<3>();

      kdtree.setInputCloud(combined_cloud_);
      for (const auto& search_point : *pc) {
        int K = 1;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        if (kdtree.nearestKSearch(search_point, K, pointIdxNKNSearch,
                                  pointNKNSquaredDistance) > 0) {
          // If this point isn't already in our cloud, add it
          if (pointNKNSquaredDistance[0] > 0.001) {
            combined_cloud_->push_back(search_point);
          } else {
            /*
            BridgePoint prev_point =
                combined_cloud_->points[pointIdxNKNSearch[0]];
            float prev_distance = sqrt(((prev_point.x - prev_origin[0]) *
                                        (prev_point.x - prev_origin[0])) +
                                       ((prev_point.y - prev_origin[1]) *
                                        (prev_point.y - prev_origin[1])) +
                                       ((prev_point.z - prev_origin[2]) *
                                        (prev_point.z - prev_origin[2])));

            float cur_distance = sqrt(((search_point.x - tf_origin[0]) *
                                       (search_point.x - tf_origin[0])) +
                                      ((search_point.y - tf_origin[1]) *
                                       (search_point.y - tf_origin[1])) +
                                      ((search_point.z - tf_origin[2]) *
                                       (search_point.z - tf_origin[2])));
            if (cur_distance < prev_distance) {*/
            combined_cloud_->points[pointIdxNKNSearch[0]] = search_point;
            //}
          }
        }
      }
      // prev_origin = tf_origin;
    }
  }
  BEAM_INFO("Finished combining point clouds - final map is: {} points",
            combined_cloud_->points.size());
}

} // end namespace inspection
