#include <unpack_velodyne_scans/UnpackVelodyneScans.h>

#include <ros/console.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <velodyne_pointcloud/pointcloudXYZIRT.h>

#include <beam_utils/log.h>
#include <boost/foreach.hpp>

namespace unpack_velodyne_scans {

UnpackVelodyneScans::UnpackVelodyneScans(bool aggregate_packets,
                                         const std::string& bag_file_path,
                                         const std::string& calibration_file,
                                         const std::string& output_postfix,
                                         const std::string& lidar_model)
    : aggregate_packets_(aggregate_packets),
      bag_file_path_(bag_file_path),
      calibration_file_(calibration_file),
      output_postfix_(output_postfix),
      data_(std::make_shared<velodyne_rawdata::RawData>()) {
  ros::Time::init();

  BEAM_INFO("Loading velodyne calibration file for unpacking...");
  std::string calibration_path;
  std::string calibration_full_path;
  calibration_path = ros::package::getPath("velodyne_pointcloud") + "/params/";
  calibration_full_path += calibration_path + calibration_file_;

  int setup =
      data_->setupOffline(calibration_full_path, lidar_model, max_range_, min_range_);
  if (setup == -1) {
    BEAM_CRITICAL("Ensure calibration file is included in " + calibration_path +
                  ". Exiting Program");
    throw std::invalid_argument{""};
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

    if (aggregate_packets_) {
      container_ptr_->setup(vel_scan_msg);
      for (size_t i = 0; i < vel_scan_msg->packets.size(); ++i) {
        data_->unpack(vel_scan_msg->packets[i], *container_ptr_,
                      vel_scan_msg->header.stamp);
      }
      sensor_msgs::PointCloud2 cloud_packet = container_ptr_->finishCloud();
      bag_out.write(out_topic, msg.getTime(), cloud_packet);
    } else {
      for (size_t i = 0; i < vel_scan_msg->packets.size(); ++i) {
        container_ptr_->setup(vel_scan_msg);
        data_->unpack(vel_scan_msg->packets[i], *container_ptr_,
                      vel_scan_msg->header.stamp);
        container_ptr_->modify_packet_time(vel_scan_msg->packets[i]);
        sensor_msgs::PointCloud2 cloud_packet = container_ptr_->finishCloud();
        ros::Time packet_indexed_time =
            msg.getTime() +
            (cloud_packet.header.stamp - vel_scan_msg->header.stamp);
        bag_out.write(out_topic, packet_indexed_time, cloud_packet);
      }
    }
  }

  BEAM_INFO("Unpacking completed. Results have been written to: {}",
            bag_out.getFileName());
  bag_in.close();
  bag_out.close();
}

}  // namespace unpack_velodyne_scans
