#include <thread>

#include <gflags/gflags.h>
#include <boost/filesystem.hpp>

#include <beam_utils/gflags.h>
#include <inspection/InspectionUtils.h>
#include <inspection/MapLabeler.h>

using namespace std::literals::chrono_literals;

DEFINE_string(images, "", "Full path to root folder of images. (Required)");
DEFINE_validator(images, &beam::gflags::ValidateDirMustExist);
DEFINE_string(output, "", "Full path to output folder of labeled. (Required)");
DEFINE_validator(output, &beam::gflags::ValidateDirMustExist);
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
DEFINE_string(frame_override, "",
              "Set this to override the moving frame id in the poses file.");
DEFINE_bool(output_individual_clouds, true,
            "set to true to output the cloud portion labeled by each image as "
            "a separate pcd file");
DEFINE_bool(
    output_camera_poses, true,
    "set to true to output camera pose (in baselink and camera frame) for each "
    "image used in labeling. It also outputs camera frustums.");
DEFINE_bool(
    output_images, true,
    "set to true to output all image containers used in the labeling process");
DEFINE_bool(draw_final_map, false,
            "set to true to draw final map in a PCL viewer");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::string config_path;
  if (FLAGS_config.empty()) {
    config_path = inspection::utils::GetConfigFilePath("MapLabelerConfig.json");
  } else {
    config_path = FLAGS_config;
  }

  inspection::MapLabeler::Inputs inputs{
      .images_directory = FLAGS_images,
      .map = FLAGS_map,
      .poses = FLAGS_poses,
      .intrinsics_directory = FLAGS_intrinsics,
      .extrinsics = FLAGS_extrinsics,
      .config_file_location = config_path,
      .poses_moving_frame_override = FLAGS_frame_override};
  inspection::MapLabeler mapper(inputs);

  mapper.PrintConfiguration();
  mapper.Run();
  mapper.SaveFinalMap(FLAGS_output);

  if (FLAGS_output_individual_clouds) {
    std::string dir = FLAGS_output + "/labelled_clouds/";
    boost::filesystem::create_directories(dir);
    mapper.SaveLabeledClouds(dir);
  }

  if (FLAGS_output_camera_poses) {
    std::string dir = FLAGS_output + "/camera_poses/";
    boost::filesystem::create_directories(dir);
    mapper.SaveCameraPoses(dir);
  }

  if (FLAGS_output_images) {
    std::string dir = FLAGS_output + "/images/";
    boost::filesystem::create_directories(dir);
    mapper.SaveImages(dir);
  }

  if (FLAGS_draw_final_map) {
    mapper.DrawFinalMap();
    while (!mapper.viewer->wasStopped()) {
      mapper.viewer->spinOnce(100);
      std::this_thread::sleep_for(100ms);
    }
  }

  return 0;
}