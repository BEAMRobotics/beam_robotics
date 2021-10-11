#include <unpack_velodyne_scans/UnpackVelodyneScans.h>

#include <beam_utils/gflags.h>
#include <gflags/gflags.h>

DEFINE_bool(
    aggregate_packets, true,
    "Set 'true' if packets contained in velodyne_msgs/VelodyneScan are to "
    "be aggregated into one sensor_msgs/PointCloud2 message");
DEFINE_string(bag_file_path, "", "Full path to bag file (Required)");
DEFINE_validator(bag_file_path, &beam::gflags::ValidateBagFileMustExist);
DEFINE_string(calibration_file, "VLP16_hires_db.yaml",
              "velodyne calibration file (Optional)");
DEFINE_string(output_postfix, "_unpacked",
              "topic postfix for unpacked velodyne scans (Optional)");
DEFINE_string(lidar_model, "VLP16",
              "Options: VLP16, 32C, 32E, VLS128. (Optional)");              

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  try {
    unpack_velodyne_scans::UnpackVelodyneScans unpack_velodyne_scans(
        FLAGS_aggregate_packets, FLAGS_bag_file_path, FLAGS_calibration_file,
        FLAGS_output_postfix, FLAGS_lidar_model);
    unpack_velodyne_scans.Run();
  } catch (std::exception& e) {
    std::cout << e.what() << std::endl;
  }

  return 0;
}
