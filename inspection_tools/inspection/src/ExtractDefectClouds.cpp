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
DEFINE_string(frame_override, "",
              "Set this to override the moving frame id in the poses file.");

using namespace beam;
using namespace inspection;

int main(int argc, char* argv[]) {
  ::gflags::ParseCommandLineFlags(&argc, &argv, true);

  MapLabeler::Inputs inputs{.images_directory = FLAGS_images,
                            .map = FLAGS_map,
                            .poses = FLAGS_poses,
                            .intrinsics_directory = FLAGS_intrinsics,
                            .extrinsics = FLAGS_extrinsics,
                            .config_file_location = "",
                            .poses_moving_frame_override =
                                FLAGS_frame_override};
  MapLabeler mapper(inputs);
  std::unordered_map<std::string, DefectCloudsMapType> defect_clouds =
      mapper.GetDefectClouds();
  mapper.SaveLabeledClouds(defect_clouds, FLAGS_output);
  BEAM_INFO("ExtractDefectClouds binary finished successfully!");
  return 0;
}