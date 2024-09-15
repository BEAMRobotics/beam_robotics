#include <gflags/gflags.h>

#include <beam_calibration/TfTree.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/gflags.h>
#include <beam_utils/se3.h>

DEFINE_string(extrinsics, "", "Full path to extrinsics json");
DEFINE_validator(extrinsics, &beam::gflags::ValidateMustBeJson);
DEFINE_string(baselink, "", "Baselink frame ID in extrinsics");
DEFINE_string(output, "", "Full path to output json");
DEFINE_validator(output, &beam::gflags::ValidateMustBeJson);

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  nlohmann::json J;
  BEAM_INFO("reading extrinsics from {}", FLAGS_extrinsics);
  if (!beam::ReadJson(FLAGS_extrinsics, J)) {
    throw std::runtime_error{"cannot load json"};
  }

  // get list of all frames
  std::set<std::string> frames;
  for (const auto& calibration : J["calibrations"]) {
    std::string to_frame = calibration["to_frame"];
    std::string from_frame = calibration["from_frame"];
    if (to_frame != FLAGS_baselink) { frames.insert(to_frame); }
    if (from_frame != FLAGS_baselink) { frames.insert(from_frame); }
  }

  nlohmann::json J_out;
  J_out["type"] = J["type"];
  J_out["date"] = J["date"];
  J_out["method"] = J["method"];
  std::vector<nlohmann::json> calibrations_out;

  beam_calibration::TfTree tf_tree;
  tf_tree.LoadJSON(FLAGS_extrinsics);
  for (const std::string& from_frame : frames) {
    nlohmann::json J_calib;
    J_calib["from_frame"] = from_frame;
    J_calib["to_frame"] = FLAGS_baselink;

    Eigen::Matrix4d T =
        tf_tree.GetTransformEigen(FLAGS_baselink, from_frame).matrix();
    std::vector<double> T_vec = beam::EigenTransformToVector(T);
    J_calib["transform"] = T_vec;
    calibrations_out.push_back(J_calib);
  }

  J_out["calibrations"] = calibrations_out;
  std::cout << "Saving final json to FLAGS_output: " << FLAGS_output << "\n";
  std::ofstream filejson(FLAGS_output);
  filejson << std::setw(4) << J_out << std::endl;
  std::cout << "Binary completed successfully\n";
  return 0;
}