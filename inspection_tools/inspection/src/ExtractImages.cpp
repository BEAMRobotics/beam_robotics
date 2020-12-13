#include <gflags/gflags.h>

#include <beam_utils/gflags.h>
#include <inspection/ImageExtractor.h>
#include <inspection/InspectionUtils.h>

DEFINE_string(bag, "", "Full path to bag file (Required).");
DEFINE_validator(bag, &beam::gflags::ValidateBagFileMustExist);
DEFINE_string(poses, "", "Full path to poses file (Required).");
DEFINE_validator(poses, &beam::gflags::ValidateJsonFileMustExist);
DEFINE_string(output, "/tmp/image_extractor", "Full path to output directory");
DEFINE_string(config, "",
              "Full path to json config file. If none provided, it will use "
              "the file inspection/config/ImageExtractorConfig.json");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::string config_path;
  if (FLAGS_config.empty()) {
    config_path =
        inspection::utils::GetConfigFilePath("ImageExtractorConfig.json");
  } else {
    config_path = FLAGS_config;
  }

  inspection::ImageExtractor extractor(FLAGS_bag, FLAGS_poses, FLAGS_output,
                                       config_path);
  extractor.ExtractImages();

  return 0;
}
