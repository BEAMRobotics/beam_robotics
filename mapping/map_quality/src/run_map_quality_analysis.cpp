#include <gflags/gflags.h>

#include <map_quality/MapQuality.h>

#include <beam_utils/gflags.h>

DEFINE_string(cloud, "", "Full file path to map pointcloud");
DEFINE_validator(cloud, &beam::gflags::ValidateFileMustExist);
DEFINE_string(output, "", "Output path to save results");
DEFINE_validator(output, &beam::gflags::ValidateDirMustExist);

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  map_quality::MapQuality map_quality(FLAGS_cloud, FLAGS_output);
  map_quality.Run();

  return 0;
}
