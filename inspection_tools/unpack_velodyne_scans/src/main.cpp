#include <unpack_velodyne_scans/UnpackVelodyneScans.h>

#include <gflags/gflags.h>
#include <beam_utils/gflags.h>

DEFINE_string(bag_file_path, "", "Full path to bag file (Required).");
DEFINE_validator(bag_file_path, &beam::gflags::ValidateBagFileMustExist);
DEFINE_string(calibration_file, "VLP16_hires_db.yaml", 
							"velodyne calibration file (Optional).");
DEFINE_string(output_postfix, "_unpacked",
              "topic postfix for unpacked velodyne scans (Optional).");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  unpack_velodyne_scans::UnpackVelodyneScans unpack_velodyne_scans(
      FLAGS_bag_file_path, FLAGS_calibration_file,
      FLAGS_output_postfix);
  unpack_velodyne_scans.Run();

  return 0;
}
