#include <unpack_velodyne_scans/UnpackVelodyneScans.h>

#include <ros/console.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h>

#include <beam_utils/log.hpp>

namespace unpack_velodyne_scans {

UnpackVelodyneScans::UnpackVelodyneScans(const std::string& bag_file_path,
                                         const std::string& calibration_file)
    : bag_file_path_(bag_file_path),
      calibration_file_(calibration_file),
      data_(new velodyne_rawdata::RawData()) {
  ros::Time::init();

  BEAM_INFO("Initializing Velodyne calibration...");
  calibration_path_ = ros::package::getPath("velodyne_pointcloud");
  calibration_path_ += "/params/" + calibration_file_;
  int set_up = data_->setupOffline(calibration_path_, max_range_, min_range_);

  if (set_up == 0) {
    BEAM_INFO("Initializing Velodyne raw data class...");
    data_->setParameters(min_range_, max_range_, view_direction_, view_width_);
    container_ptr_ = boost::shared_ptr<velodyne_pointcloud::PointcloudXYZIR>(
        new velodyne_pointcloud::PointcloudXYZIR(max_range_, min_range_,
                                                 target_frame_, fixed_frame_,
                                                 data_->scansPerPacket()));
    container_ptr_->configure(max_range_, min_range_, fixed_frame_,
                              target_frame_);
    Unpack();
  } else {
    BEAM_CRITICAL(
        "Unsure calibration file is inlcuded in "
        "/velodyne/velodyne_pointcloud/params. Exiting Program");
  }
}

void UnpackVelodyneScans::Unpack() {
  rosbag::Bag bag_in;
  try {
    bag_in.open(bag_file_path_, rosbag::bagmode::Read);
  } catch (rosbag::BagException& ex) {
    BEAM_CRITICAL("Bag exception: {}. Exiting Program.", ex.what());
    return;
  }

  rosbag::View view(bag_in, ros::TIME_MIN, ros::TIME_MAX, true);
  rosbag::Bag bag_out;
  std::string str = bag_file_path_;
  str.replace(str.end() - 4, str.end(), "_unpacked.bag");
  bag_out.open(str, rosbag::bagmode::Write);

  int total_messages = view.size();
  int message_counter = 0;
  std::string output_message = "Processing bag for velodyne unpacking...";
  for (auto iter = view.begin(); iter != view.end(); iter++) {
    message_counter++;
    auto vel_scan_msg = iter->instantiate<velodyne_msgs::VelodyneScan>();
    std::string out_topic = iter->getTopic();
    out_topic += "_unpacked";

    if (vel_scan_msg != NULL) {
      for (size_t i = 0; i < vel_scan_msg->packets.size(); ++i) {
        container_ptr_->setup(vel_scan_msg);
        data_->unpack(vel_scan_msg->packets[i], *container_ptr_,
                      vel_scan_msg->header.stamp);
        container_ptr_->modify_pkt_time(vel_scan_msg->packets[i]);
        sensor_msgs::PointCloud2 cloud_packet = container_ptr_->finishCloud();
        bag_out.write(out_topic, cloud_packet.header.stamp, cloud_packet);
      }
    }
    beam::OutputPercentComplete(message_counter, total_messages,
                                output_message);
  }

  bag_in.close();
  bag_out.close();
  BEAM_INFO("Done processessing Bag");
}

}  // namespace unpack_velodyne_scans
