#include <thread>

#include <gflags/gflags.h>

#include <beam_utils/gflags.h>
#include <inspection/InspectionUtils.h>
#include <inspection/MapLabeler.h>

using namespace std::literals::chrono_literals;

DEFINE_string(images, "", "Full path to root folder of images. (Required)");
DEFINE_validator(images, &beam::gflags::ValidateDirMustExist);
DEFINE_string(map, "", "Full path to unlabeled pcd map. (Required)");
DEFINE_validator(map, &beam::gflags::ValidateFileMustExist);
DEFINE_string(poses, "", "Full path to poses file (Required).");
DEFINE_validator(poses, &beam::gflags::ValidateJsonFileMustExist);
DEFINE_string(intrinsics, "",
              "Full path to root folder of intrinsics. (Required)");
DEFINE_validator(intrinsics, &beam::gflags::ValidateDirMustExist);
DEFINE_string(extrinsics, "", "Full path to json extrinsics file. (Required)");
DEFINE_validator(extrinsics, &beam::gflags::ValidateJsonFileMustExist);
DEFINE_string(config, "",
              "Full path to json config file. If none provided, it will use "
              "the file inspection/config/MapLabeler.json");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::string config_path;
  if (FLAGS_config.empty()) {
    config_path = inspection::utils::GetConfigFilePath("MapLabeler.json");
  } else {
    config_path = FLAGS_config;
  }

  inspection::MapLabeler mapper(FLAGS_images, FLAGS_map, FLAGS_poses,
                                FLAGS_intrinsics, FLAGS_extrinsics,
                                config_path);

  mapper.PrintConfiguration();
  mapper.Run();

  // TODO: put these into a json
  mapper.PlotFrames("hvlp_link", mapper.viewer);
  mapper.PlotFrames("F1_link", mapper.viewer);
  mapper.PlotFrames("F2_link", mapper.viewer);
  mapper.DrawFinalMap();
  mapper.SaveLabeledClouds();

  while (!mapper.viewer->wasStopped()) {
    mapper.viewer->spinOnce(100);
    std::this_thread::sleep_for(100ms);
  }
  return 0;
}