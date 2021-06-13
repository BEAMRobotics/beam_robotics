#pragma once

#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>

#include <visualize_feature_tracks/feature_tracker.h>
#include <visualize_feature_tracks/LandmarkMeasurementMsg.h>

namespace visualize_feature_tracks {

class TrackerNodelet : public nodelet::Nodelet {
 public:
  TrackerNodelet();

 private:
  void onInit();

  void LoadParams();

  void ImageCallback(const sensor_msgs::Image::ConstPtr img_msg);

  ros::Subscriber image_subscriber_;
  ros::Publisher tracks_publisher_;

  std::string image_topic_;
  std::string tracks_topic_;
  std::string tracker_type_;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  FeatureTracker tracker_;
};

}  // namespace visualize_feature_tracks
