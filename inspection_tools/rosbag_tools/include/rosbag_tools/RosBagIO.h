#pragma once

#include <ros/ros.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_msgs/VelodyneScan.h>

namespace rosbag_tools {

class RosBagReader {
public:
  RosBagReader(
      const std::string& bag_file_path,
      const std::vector<std::string>& topics = std::vector<std::string>(),
      const ros::Time& start_time = ros::TIME_MIN,
      const ros::Time& end_time = ros::TIME_MAX);

  /**
   * @brief Get the Next Msg iterator object
   *
   * @param iter reference to view iterator to populate
   * @return true if successful
   * @return false  if unsuccessful
   */
  bool GetNextMsg(rosbag::View::iterator& iter);

  /**
   * @brief Get the Next Image Msg object
   *
   * @return boost::shared_ptr<sensor_msgs::Image> will be null if unsuccessful.
   * Therefore you can check using if(msg_ptr)
   */
  boost::shared_ptr<sensor_msgs::Image> GetNextImageMsg();

  /**
   * @brief Get the Next Velodyne Msg object
   *
   * @return boost::shared_ptr<velodyne_msgs::VelodyneScan>  will be null if
   * unsuccessful. Therefore you can check using if(msg_ptr)
   */
  boost::shared_ptr<velodyne_msgs::VelodyneScan> GetNextVelodyneMsg();

private:
  std::string bag_file_path_;
  rosbag::Bag bag_;
  std::unique_ptr<rosbag::View> view_{nullptr};
  rosbag::View::iterator iter_;
  rosbag::View::iterator iter_velodyne_;
  rosbag::View::iterator iter_image_;
};

class RosBagWriter {
public:
  explicit RosBagWriter(const std::string& bag_file_path);

  void CloseBag();

  void AddMsg(const rosbag::MessageInstance& msg);

  void AddMsg(const std::string& topic, const ros::Time& stamp,
              const sensor_msgs::PointCloud2& cloud);

  void AddMsg(const std::string& topic, const ros::Time& stamp,
              const sensor_msgs::Image& image);

  void AddMsg(const std::string& topic, const ros::Time& stamp,
              const sensor_msgs::CompressedImage& image);

private:
  std::string bag_file_path_;
  rosbag::Bag bag_;
};

} // namespace rosbag_tools
