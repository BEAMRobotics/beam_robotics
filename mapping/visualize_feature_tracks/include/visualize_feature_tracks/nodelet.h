#pragma once

#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>

#include <visualize_feature_tracks/feature_tracker.h>

namespace visualize_feature_tracks {

class TrackerNodelet : public nodelet::Nodelet {
 public:
  TrackerNodelet();

 private:
  void onInit();

  void LoadParams();

  void ImageCallback(const sensor_msgs::Image::ConstPtr img_msg);

  ros::Subscriber image_subscriber_;
  ros::Publisher image_publisher_;

  std::string image_in_topic_;
  std::string image_out_topic_;
  std::string tracker_type_;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  FeatureTracker tracker_;
};

}  // namespace visualize_feature_tracks
