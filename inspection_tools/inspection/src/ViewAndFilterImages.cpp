#include <gflags/gflags.h>

#include <beam_utils/gflags.h>
#include <inspection/ImageDatabase.h>

DEFINE_string(cameras_list, "",
              "Full path to cameras list json file (Required).");
DEFINE_validator(cameras_list, &beam::gflags::ValidateJsonFileMustExist);

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // TODO

  return 0;
}
