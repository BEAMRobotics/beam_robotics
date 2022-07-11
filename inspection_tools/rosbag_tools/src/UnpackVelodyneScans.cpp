#include <unpack_velodyne_scans/UnpackVelodyneScans.h>

#include <ros/console.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <velodyne_pointcloud/pointcloudXYZIRT.h>

#include <beam_utils/log.h>
#include <boost/foreach.hpp>

namespace unpack_velodyne_scans {

UnpackVelodyneScans::UnpackVelodyneScans(const std::string& bag_file_path,
                                         const std::string& output_postfix,
                                         const std::string& lidar_model)
    : bag_file_path_(bag_file_path),
      output_postfix_(output_postfix),
      data_(std::make_shared<velodyne_rawdata::RawData>()) {
  ros::Time::init();

  BEAM_INFO("Loading velodyne calibration file for unpacking...");
  std::string calibration_full_path =
      ros::package::getPath("velodyne_pointcloud") + "/params/";
  if (lidar_model == "VLP16") {
    calibration_full_path += "VLP16db.yaml";
  } else if (lidar_model == "32C") {
    calibration_full_path += "32db.yaml";
  } else if (lidar_model == "32E") {
    calibration_full_path += "VeloView-VLP-32C.yaml";
  } else if (lidar_model == "VLS128") {
    calibration_full_path += "VLS128.yaml";
  } else {
    BEAM_CRITICAL("Ensure lidar model is supported by BEAM");
    throw std::invalid_argument{"Invalid lidar model."};
  }

  BEAM_INFO("Setting up offline data processing...");
  int setup = data_->setupOffline(calibration_full_path, lidar_model,
                                  max_range_, min_range_);
  if (setup == -1) {
    BEAM_CRITICAL("Unable to set up offline data processing");
    throw std::invalid_argument{"Invalid offline set up."};
  }

  BEAM_INFO("Initializing Velodyne::RawData class for unpacking...");
  data_->setParameters(min_range_, max_range_, view_direction_, view_width_);
  container_ptr_ = std::make_shared<velodyne_pointcloud::PointcloudXYZIRT>(
      max_range_, min_range_, "", "", data_->scansPerPacket());
}

void UnpackVelodyneScans::Run() {
  rosbag::Bag bag_in;
  rosbag::Bag bag_out;
  bag_in.open(bag_file_path_, rosbag::bagmode::Read);
  bag_file_path_.replace(bag_file_path_.end() - 4, bag_file_path_.end(),
                         output_postfix_ + ".bag");
  bag_out.open(bag_file_path_, rosbag::bagmode::Write);

  rosbag::View view(bag_in);
  BEAM_INFO("Unpacking...");
  BOOST_FOREACH (rosbag::MessageInstance const msg, view) {
    bag_out.write(msg.getTopic(), msg.getTime(), msg);

    auto vel_scan_msg = msg.instantiate<velodyne_msgs::VelodyneScan>();
    if (vel_scan_msg == NULL) {
      continue;
    }

    std::string out_topic = msg.getTopic();
    out_topic += output_postfix_;

    container_ptr_->setup(vel_scan_msg);
    for (size_t i = 0; i < vel_scan_msg->packets.size(); ++i) {
      data_->unpack(vel_scan_msg->packets[i], *container_ptr_,
                    vel_scan_msg->header.stamp);
    }
    sensor_msgs::PointCloud2 cloud_packet = container_ptr_->finishCloud();
    bag_out.write(out_topic, msg.getTime(), cloud_packet);
  }

  BEAM_INFO("Unpacking completed. Results have been written to: {}",
            bag_out.getFileName());
  bag_in.close();
  bag_out.close();
}

}  // namespace unpack_velodyne_scans
