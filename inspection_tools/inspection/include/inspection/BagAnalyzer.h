#pragma once
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
//#include <message_filters/simple_filter.h>
#include <message_filters/subscriber.h>
//#include <message_filters/time_sequencer.h>
#include <tf2/buffer_core.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>

#include <chrono>
#include <memory>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>

namespace inspection {

/**
 * @brief Class for loading a bag
 */
class BagAnalyzer {
public:
  BagAnalyzer(std::string& bag_file_path);

  ~BagAnalyzer() = default;

  void LoadBag(const std::string& bag_file);

  void GetAggregateScans();

  void GetTFBuffer();
  std::unique_ptr<tf2::BufferCore> GetTF2Buffer();

  std::vector<std::vector<rosbag::MessageInstance>> GetPointClouds(
      std::vector<std::pair<ros::Time, ros::Time>> region_of_interest);

  ros::Time GetStartTime();
  ros::Time GetEndTime();
  std::pair<ros::Time, ros::Time> GetTimeframe();

  std::vector<std::pair<ros::Time, ros::Time>> GetROIs(float threshold);

private:
  rosbag::Bag bag_file_;
  std::vector<rosbag::MessageInstance> messages_ = {};
};

/**
 * @brief Simple timer object
 */
struct HighResolutionTimer {
  HighResolutionTimer() : start_time(take_time_stamp()) {}

  /**
   * @brief Restart timer
   */
  void restart() { start_time = take_time_stamp(); }

  /**
   * @brief Return elapsed time in seconds.
   * @return
   */
  double elapsed() const {
    return double(take_time_stamp() - start_time) * 1e-9;
  }

  std::uint64_t elapsed_nanoseconds() const {
    return take_time_stamp() - start_time;
  }

protected:
  static std::uint64_t take_time_stamp() {
    return std::uint64_t(
        std::chrono::high_resolution_clock::now().time_since_epoch().count());
  }

private:
  std::uint64_t start_time;
};
}