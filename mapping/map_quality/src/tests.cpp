#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <beam_utils/filesystem.h>
#include <beam_utils/kdtree.h>
#include <beam_utils/log.h>
#include <beam_utils/pointclouds.h>

#include <map_quality/MapQuality.h>

bool values_equal(double v1, double v2) {
  if (abs(v1 - v2) < 0.001) { return true; }
  return false;
}

void run_roughness_tests() {
  // generate plane
  PointCloudQuality cloud;
  double dx = 1;
  double dy = 1;
  double increment = 0.01;
  double z_plane = 0;

  double x = -1 * dx;
  double y = -1 * dy;
  std::vector<uint32_t> pt_ids_all;
  while (x < dx) {
    while (y < dy) {
      PointQuality pt;
      pt.x = x;
      pt.y = y;
      pt.z = z_plane;
      y += increment;
      pt_ids_all.push_back(pt_ids_all.size());
      cloud.push_back(pt);
    }
    x += increment;
  }

  // create points and check that the roughness is as expected
  double r_expected = -1.1;
  PointQuality pt_query;
  pt_query.x = 0;
  pt_query.y = 0;
  pt_query.z = r_expected;
  double roughness =
      map_quality::CalculateRoughness(pt_query, cloud, pt_ids_all, 5);
  if (values_equal(roughness, r_expected)) {
    std::cout << "Roughness not what is expected. \nExpected: " << r_expected
              << "\nCalculated: " << r_expected << "\n";
    throw std::runtime_error("TEST FAILED");
  }
  std::cout << "Roughness tests succeeded!\n";
}

int main(int argc, char* argv[]) {
  run_roughness_tests();
  return 0;
}
