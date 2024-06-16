#include <gflags/gflags.h>

#include <map_quality/MapQuality.h>

#include <beam_utils/gflags.h>

DEFINE_string(cloud, "", "Full file path to map pointcloud");
DEFINE_validator(cloud, &beam::gflags::ValidateFileMustExist);
DEFINE_string(output, "",
              "Output path to save results. Must be a json file output");
DEFINE_string(pointcloud_output, "",
              "If set, this will output the metrics into a pointcloud for "
              "viewing metrics that apply to each point.");
DEFINE_validator(output, &beam::gflags::ValidateDirMustExist);
DEFINE_double(voxel_size, 0.01, "voxel size in meters");
DEFINE_double(radius, 0.1,
              "search radius for density and roughness calculations.");
DEFINE_double(knn, 10,
              "number of neighbors to use for average distance metric");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  map_quality::MapQuality map_quality(FLAGS_cloud, FLAGS_output,
                                      FLAGS_voxel_size, FLAGS_knn, FLAGS_radius,
                                      FLAGS_pointcloud_output);
  map_quality.Run();

  return 0;
}
