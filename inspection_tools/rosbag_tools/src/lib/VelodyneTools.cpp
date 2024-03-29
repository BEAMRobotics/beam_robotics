#include <rosbag_tools/VelodyneTools.h>

#include <boost/filesystem.hpp>
#include <ros/console.h>
#include <ros/package.h>

#include <beam_utils/filesystem.h>
#include <beam_utils/log.h>

#include <rosbag_tools/PointcloudXYZIRT.h>

namespace rosbag_tools {

VelodyneTools::VelodyneTools(const std::string& lidar_model)
    : lidar_model_(lidar_model),
      data_(std::make_shared<velodyne_rawdata::RawData>()) {
  std::string velodyne_params_path = beam::CombinePaths(
      ros::package::getPath("velodyne_pointcloud"), "params");
  std::string calibration_full_path;
  if (lidar_model == "VLP16") {
    calibration_full_path =
        beam::CombinePaths(velodyne_params_path, "VLP16db.yaml");
  } else if (lidar_model == "32C") {
    calibration_full_path =
        beam::CombinePaths(velodyne_params_path, "32db.yaml");
    calibration_full_path += ".yaml";
  } else if (lidar_model == "32E") {
    calibration_full_path =
        beam::CombinePaths(velodyne_params_path, "VeloView-VLP-32C.yaml");
    calibration_full_path += ".yaml";
  } else if (lidar_model == "VLS128") {
    calibration_full_path =
        beam::CombinePaths(velodyne_params_path, "VLS128.yaml");
  } else {
    BEAM_CRITICAL("Ensure lidar model is supported by BEAM");
    throw std::invalid_argument{"Invalid lidar model."};
  }

  if (!boost::filesystem::exists(calibration_full_path)) {
    BEAM_CRITICAL("Cannot find velodyne calibration file at {}",
                  calibration_full_path);
    throw std::invalid_argument{"Invalid file path"};
  }
  BEAM_INFO("Using velodyne calibration file: {}", calibration_full_path);

  BEAM_INFO("Setting up offline data processing...");
  int setup = data_->setupOffline(calibration_full_path, lidar_model,
                                  max_range_, min_range_);
  if (setup == -1) {
    BEAM_CRITICAL("Unable to set up offline data processing");
    throw std::invalid_argument{"Invalid offline set up."};
  }

  BEAM_INFO("Initializing Velodyne::RawData class for unpacking");
  data_->setParameters(min_range_, max_range_, view_direction_, view_width_);
  container_ptr_ = std::make_shared<rosbag_tools::PointcloudXYZIRT>(
      max_range_, min_range_, "", "", data_->scansPerPacket());
  BEAM_INFO("Done initializing");

  ros::Time::init();
}

sensor_msgs::PointCloud2 VelodyneTools::UnpackScan(
    const velodyne_msgs::VelodyneScan::ConstPtr& scan) {
  container_ptr_->setup(scan);
  for (size_t i = 0; i < scan->packets.size(); ++i) {
    data_->unpack(scan->packets[i], *container_ptr_, scan->header.stamp);
  }
  return container_ptr_->finishCloud();
}

} // namespace rosbag_tools
