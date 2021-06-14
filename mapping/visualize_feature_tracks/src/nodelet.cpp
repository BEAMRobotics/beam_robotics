#include <visualize_feature_tracks/nodelet.h>

#include <stdlib.h>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <pluginlib/class_list_macros.h>

namespace visualize_feature_tracks {

TrackerNodelet::TrackerNodelet() {}

void TrackerNodelet::onInit() {
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();
  LoadParams();

  image_subscriber_ = private_nh_.subscribe(
      image_in_topic_, 1000, &TrackerNodelet::ImageCallback, this);

  image_publisher_ =
      private_nh_.advertise<sensor_msgs::Image>(image_out_topic_, 10);
}

void TrackerNodelet::ImageCallback(const sensor_msgs::Image::ConstPtr img_msg) {
  // extract image
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat img_col = cv_ptr->image;
  cv::Mat img_mono;
  cv::cvtColor(img_col, img_mono, CV_BGR2GRAY);

  // get image with tracks drawn
  std::vector<beam_cv::FeatureTrack> tracks = tracker_.GetTracks(img_mono, img_msg->header.stamp);
  cv::Mat image_out = tracker_.DrawTracks(tracks, img_col);

  cv_bridge::CvImage out_tmp;
  out_tmp.header = img_msg->header;
  out_tmp.encoding = sensor_msgs::image_encodings::RGB8;
  out_tmp.image = image_out;

  sensor_msgs::Image out_msg;
  out_tmp.toImageMsg(out_msg);
  image_publisher_.publish(out_msg);
}

void TrackerNodelet::LoadParams() {
  if (private_nh_.getParam("image_in_topic", image_in_topic_)) {
    ROS_INFO("Loaded image_in_topic: %s", image_in_topic_.c_str());
  } else {
    ROS_INFO("Could not load image_in_topic");
  }

  if (private_nh_.getParam("image_out_topic", image_out_topic_)) {
    ROS_INFO("Loaded image_out_topic: %s", image_out_topic_.c_str());
  } else {
    ROS_INFO("Could not load image_out_topic");
  }

  if (private_nh_.getParam("tracker_type", tracker_type_)) {
    ROS_INFO("Loaded tracker_type: %s", tracker_type_.c_str());
  } else {
    ROS_INFO("Could not load tracker_type");
  }
}

}  // namespace visualize_feature_tracks

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(visualize_feature_tracks::TrackerNodelet,
                       nodelet::Nodelet);
