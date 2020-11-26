#include <gflags/gflags.h>

#include <beam_utils/gflags.h>
#include <inspection/InspectionUtils.h>
#include <inspection/QuantifyDefects.h>

DEFINE_string(cloud, "", "Full path to labeled point cloud (Required).");
DEFINE_validator(cloud, &beam::gflags::ValidateFileMustExist);
DEFINE_string(output, "/tmp/quantify_defects", "Full path to output directory");
DEFINE_string(config, "",
              "Full path to json config file. If none provided, it will use "
              "the file inspection/config/QuantifyDefectsConfig.json");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::string config_path;
  if (FLAGS_config.empty()) {
    config_path =
        inspection::utils::GetConfigFilePath("QuantifyDefectsConfig.json");
  } else {
    config_path = FLAGS_config;
  }

  inspection::QuantifyDefects quant_defects(FLAGS_cloud, FLAGS_output,
                                            config_path);
  quant_defects.OutputCorrosionInfo();
  quant_defects.SaveCorrosionOnlyCloud();

  quant_defects.OutputCrackInfo();
  quant_defects.SaveCrackOnlyCloud();

  quant_defects.OutputSpallInfo();
  quant_defects.SaveSpallOnlyCloud();

  return 0;
}
