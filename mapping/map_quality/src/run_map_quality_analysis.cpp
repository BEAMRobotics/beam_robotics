#include <gflags/gflags.h>

#include <map_quality/ScanPoseGtGeneration.h>

#include <beam_utils/gflags.h>

DEFINE_string(cloud, "", "Full file path to map pointcloud");
DEFINE_validator(gt_cloud, &beam::gflags::ValidateFileMustExist);

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  map_quality::MapQuality map_quality;

  return 0;
}
