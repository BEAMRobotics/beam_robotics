#include <gflags/gflags.h>

#include <map_quality/MapQuality.h>

#include <beam_utils/gflags.h>

DEFINE_string(cloud, "", "Full file path to map pointcloud");
DEFINE_validator(cloud, &beam::gflags::ValidateFileMustExist);
DEFINE_string(output, "",
              "Output path to save results. Must be a json file output");
DEFINE_validator(output, &beam::gflags::ValidateMustBeJson);
DEFINE_string(pointcloud_output, "",
              "If set, this will output the metrics into a pointcloud for "
              "viewing metrics that apply to each point.");
DEFINE_double(voxel_size, 0.01, "voxel size in meters");
DEFINE_double(radius, 0.1,
              "search radius for density and roughness calculations.");
DEFINE_double(knn, 10,
              "number of neighbors to use for average distance metric");
DEFINE_bool(voxel, true, "whether or not to run voxel statistics");
DEFINE_bool(neighborhood, true,
            "whether or not to run neighborhood statistics");
DEFINE_bool(plane, true, "whether or not to run plane statistics");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  map_quality::MapQuality map_quality(FLAGS_cloud, FLAGS_output,
                                      FLAGS_pointcloud_output);
  if (FLAGS_voxel) {
    map_quality.ComputeVoxelStatistics(FLAGS_voxel_size, FLAGS_knn);
  }
  if (FLAGS_neighborhood) {
    map_quality.ComputerNeighborhoodStatistics(FLAGS_radius);
  }
  if (FLAGS_plane) { map_quality.ComputePlaneStatistics(); }
  map_quality.SaveResults();
  return 0;
}
