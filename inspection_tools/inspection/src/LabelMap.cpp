#include <boost/filesystem.hpp>
#include <gflags/gflags.h>

#include <beam_utils/gflags.h>
#include <beam_utils/log.h>
#include <inspection/InspectionUtils.h>
#include <inspection/MapLabeler.h>

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
DEFINE_bool(draw_final_map, true,
            "set to true to draw final map in a PCL viewer");

using namespace beam;
using namespace inspection;

int main(int argc, char* argv[]) {
  ::gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::string config_path;
  if (FLAGS_config.empty()) {
    config_path = utils::GetConfigFilePath("MapLabelerConfig.json");
  } else {
    config_path = FLAGS_config;
  }

  MapLabeler::Inputs inputs{
      .images_directory = FLAGS_images,
      .map = FLAGS_map,
      .poses = FLAGS_poses,
      .intrinsics_directory = FLAGS_intrinsics,
      .extrinsics = FLAGS_extrinsics,
      .config_file_location = config_path,
      .poses_moving_frame_override = FLAGS_frame_override};
  MapLabeler mapper(inputs);
  mapper.PrintConfiguration();
  DefectCloud::Ptr final_map = mapper.RunFullPipeline(
      FLAGS_output, FLAGS_output_individual_clouds, true);
  mapper.SaveFinalMap(final_map, FLAGS_output);
  mapper.OutputConfig(FLAGS_output);

  if (FLAGS_output_camera_poses) {
    std::string dir = beam::CombinePaths(FLAGS_output, "camera_poses");
    boost::filesystem::create_directories(dir);
    mapper.SaveCameraPoses(dir);
  }

  if (FLAGS_output_images) {
    std::string dir = beam::CombinePaths(FLAGS_output, "images");
    boost::filesystem::create_directories(dir);
    mapper.SaveImages(dir);
  }

  if (FLAGS_draw_final_map) { mapper.DrawFinalMap(final_map); }

  BEAM_INFO("LabelMap binary finished successfully!");
  return 0;
}