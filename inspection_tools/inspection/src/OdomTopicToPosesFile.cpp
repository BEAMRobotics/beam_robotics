#include <gflags/gflags.h>

#include <beam_mapping/Poses.h>
#include <beam_utils/gflags.h>

DEFINE_string(bag, "", "Full path to bag file (Required).");
DEFINE_validator(bag, &beam::gflags::ValidateBagFileMustExist);
DEFINE_string(topic, "", "Odometry topic to get poses from (Required).");
DEFINE_validator(topic, &beam::gflags::ValidateCannotBeEmpty);
DEFINE_string(
    output, "",
    "Full path to output file (Required). Example: /home/user/pose_file.json");
DEFINE_validator(output, &beam::gflags::ValidateMustBeJson);

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // create and export poses file
  beam_mapping::Poses poses;
  poses.LoadFromBAG(FLAGS_bag, FLAGS_topic);
  poses.WriteToJSON(FLAGS_output);

  return 0;
}