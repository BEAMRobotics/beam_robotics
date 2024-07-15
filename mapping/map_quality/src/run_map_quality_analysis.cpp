#include <gflags/gflags.h>

#include <map_quality/MapQuality.h>

#include <beam_utils/gflags.h>

DEFINE_string(cloud, "", "Full file path to map pointcloud");
DEFINE_validator(cloud, &beam::gflags::ValidateFileMustExist);
DEFINE_string(
    config, "",
    "Full file path to config json file. Will use default if not provided.");
DEFINE_string(output, "",
              "Output path to save results. Must be a json file output");
DEFINE_validator(output, &beam::gflags::ValidateMustBeJson);
DEFINE_string(pointcloud_output, "",
              "If set, this will output the metrics into a pointcloud for "
              "viewing metrics that apply to each point.");
DEFINE_string(planes_output, "",
              "If set, this will output the extracted planes to this folder");
DEFINE_bool(voxel, true, "whether or not to run voxel statistics");
DEFINE_bool(neighborhood, false,
            "whether or not to run neighborhood statistics");
DEFINE_bool(plane, true, "whether or not to run plane statistics");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  map_quality::MapQuality map_quality(FLAGS_cloud, FLAGS_output, FLAGS_config,
                                      FLAGS_pointcloud_output,
                                      FLAGS_planes_output);
  if (FLAGS_voxel) { map_quality.ComputeVoxelStatistics(); }
  if (FLAGS_neighborhood) { map_quality.ComputerNeighborhoodStatistics(); }
  if (FLAGS_plane) { map_quality.ComputePlaneStatistics(); }
  map_quality.SaveResults();
  return 0;
}
