#include <visualize_feature_tracks/tracker_nodelet.h>

#include <stdlib.h>
#include <string>

#include <cv_bridge/cv_bridge.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

namespace visualize_feature_tracks {

TrackerNodelet::TrackerNodelet() {}

void TrackerNodelet::onInit() {
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();
  LoadParams();
  
  image_subscriber_ = private_nh_.subscribe(
      image_topic_, 1000, &TrackerNodelet::ImageCallback, this);
}

void TrackerNodelet::ImageCallback(
    const sensor_msgs::Image::ConstPtr img_msg) {
  // extract image
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat image = cv_ptr->image;
}

void TrackerNodelet::LoadParams() {
  if (private_nh_.getParam("image_topic", image_topic_)) {
    ROS_INFO("Loaded image_topic: %f", image_topic_);
  } else {
    ROS_INFO("Could not load image_topic");
  }

  if (private_nh_.getParam("tracks_topic", tracks_topic_)) {
    ROS_INFO("Loaded tracks_topic: %f", tracks_topic_);
  } else {
    ROS_INFO("Could not load tracks_topic");
  }

  if (private_nh_.getParam("tracker_type", tracker_type_)) {
    ROS_INFO("Loaded tracker_type: %f", tracker_type_);
  } else {
    ROS_INFO("Could not load tracker_type_");
  }
}

}  // namespace visualize_feature_tracks

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(visualize_feature_tracks::TrackerNodelet,
                       nodelet::Nodelet);
