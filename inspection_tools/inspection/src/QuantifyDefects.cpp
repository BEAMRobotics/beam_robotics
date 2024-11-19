#include <gflags/gflags.h>

#include <beam_utils/gflags.h>
#include <beam_utils/log.h>
#include <inspection/DefectQuantification.h>
#include <inspection/InspectionUtils.h>

DEFINE_string(cloud, "", "Full path to labeled point cloud (Required).");
DEFINE_validator(cloud, &beam::gflags::ValidateFileMustExist);
DEFINE_string(output, "/tmp/quantify_defects", "Full path to output directory");
DEFINE_string(config, "",
              "Full path to json config file. If none provided, it will use "
              "the file inspection/config/QuantifyDefectsConfig.json");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::string config_path = FLAGS_config;
  if (config_path.empty()) {
    config_path =
        inspection::utils::GetConfigFilePath("QuantifyDefectsConfig.json");
    BEAM_INFO("No config provided, using default at {}", config_path);
  }
  inspection::DefectQuantification quant_defects(FLAGS_cloud, FLAGS_output,
                                                 config_path);
  quant_defects.ProcessDefects();
  quant_defects.SaveResults();
  return 0;
}
