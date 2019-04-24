#define PCL_NO_PRECOMPILE

#include "inspection/CloudCombiner.h"
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/kdtree.h>

namespace inspection {

void CloudCombiner::CombineClouds(
    std::vector<std::vector<pcl::PointCloud<beam_containers::PointBridge>::Ptr>>
        clouds) {
  pcl::search::KdTree<BridgePoint> kdtree;
  BridgePoint bp = {0,0,0};
  combined_cloud_->points.push_back(bp);

  // Iterate through and successfully add point clouds to final
  int num_cams = clouds.size();
  for (int cam = 0; cam < num_cams; cam++) {
    for (const auto& pc : clouds[cam]) {
      //      std::cout << "Cam : " << cam << std::endl;
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
            // Else : If the point is already in the cloud, we will replace it
            // with the most recently observed version of the point
            combined_cloud_->points[pointIdxNKNSearch[0]] = search_point;
          }

/*          for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
            std::cout << "    " << pc->points[pointIdxNKNSearch[i]].x << " "
                      << pc->points[pointIdxNKNSearch[i]].y << " "
                      << pc->points[pointIdxNKNSearch[i]].z
                      << " (squared distance: " << pointNKNSquaredDistance[i]
                      << ")" << std::endl;*/
        }
      }
    }
  }

  std::cout << "Combined point cloud size: " << combined_cloud_->points.size()
            << std::endl;
}

} // end namespace inspection
