#include <gflags/gflags.h>

#include <beam_utils/gflags.h>
#include <lidar_trajectory_validation/LidarTrajectoryValidation.h>

DEFINE_string(
    poses, "",
    "Full path poses file (Required). See pose file types in "
    "libbeam/beam_mapping/poses.h. Or extract poses using 3d_map_builder");
DEFINE_validator(poses, &beam::gflags::ValidateJsonFileMustExist);

DEFINE_string(extrinsics, "",
              "Full path to extrinsics json (Required). See "
              "beam_robotics/calibration for file formats");
DEFINE_validator(extrinsics, &beam::gflags::ValidateJsonFileMustExist);

DEFINE_string(config, "",
              "Full path to config file. For example config, see "
              "lidar_to_lidar_calibrator/config/config.json");
DEFINE_validator(config, &beam::gflags::ValidateJsonFileMustExist);

DEFINE_string(timestamps, "",
              "Full json file containing loop closure timestamps. For example "
              "config, see "
              "lidar_to_lidar_calibrator/config/example_loop_timestamps.json");
DEFINE_validator(timestamps, &beam::gflags::ValidateJsonFileMustExist);

DEFINE_string(matcher_config, "",
              "Full path to matcher config file. For example config, see "
              "lidar_to_lidar_calibrator/config/icp.json, or see "
              "libbeam/beam_matching/icp.json");
DEFINE_validator(matcher_config, &beam::gflags::ValidateJsonFileMustExist);

DEFINE_string(bag, "", "Full path to bag file which contains the lidar data");
DEFINE_validator(bag, &beam::gflags::ValidateBagFileMustExist);

DEFINE_string(output, "",
              "Full path to output directory (Required). Directory must exist");
DEFINE_validator(output, &beam::gflags::ValidateDirMustExist);

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  LidarTrajectoryValidation::Inputs inputs{
      .poses = FLAGS_poses,
      .extrinsics = FLAGS_extrinsics,
      .config = FLAGS_config,
      .matcher_config = FLAGS_matcher_config,
      .bag = FLAGS_bag,
      .output = FLAGS_output,
      .timestamps = FLAGS_timestamps,
  };
  LidarTrajectoryValidation validator(inputs);
  validator.Run();
  return 0;
}