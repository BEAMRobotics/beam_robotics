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
  feature_tracker_ = std::make_unique<FeatureTracker>(feature_tracker_params_);

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
  std::vector<beam_cv::FeatureTrack> tracks =
      feature_tracker_->GetTracks(img_mono, img_msg->header.stamp);
  cv::Mat image_out = feature_tracker_->DrawTracks(tracks, img_col);

  cv_bridge::CvImage out_tmp;
  out_tmp.header = img_msg->header;
  out_tmp.encoding = sensor_msgs::image_encodings::RGB8;
  out_tmp.image = image_out;

  sensor_msgs::Image out_msg;
  out_tmp.toImageMsg(out_msg);
  image_publisher_.publish(out_msg);
}

void TrackerNodelet::LoadParams() {
  // load topics
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

  // load types
  if (private_nh_.getParam("tracker_type",
                           feature_tracker_params_.tracker_type)) {
    ROS_INFO("Loaded tracker_type: %s",
             feature_tracker_params_.tracker_type.c_str());
  } else {
    ROS_INFO("Could not load tracker_type");
  }

  if (private_nh_.getParam("detector_type",
                           feature_tracker_params_.detector_type)) {
    ROS_INFO("Loaded detector_type: %s",
             feature_tracker_params_.detector_type.c_str());
  } else {
    ROS_INFO("Could not load detector_type");
  }

  if (private_nh_.getParam("descriptor_type",
                           feature_tracker_params_.descriptor_type)) {
    ROS_INFO("Loaded descriptor_type: %s",
             feature_tracker_params_.descriptor_type.c_str());
  } else {
    ROS_INFO("Could not load descriptor_type");
  }

  if (private_nh_.getParam("matcher_type",
                           feature_tracker_params_.matcher_type)) {
    ROS_INFO("Loaded matcher_type: %s",
             feature_tracker_params_.matcher_type.c_str());
  } else {
    ROS_INFO("Could not load matcher_type");
  }

  // load config files
  private_nh_.getParam("tracker_config",
                       feature_tracker_params_.tracker_config);
  private_nh_.getParam("detector_config",
                       feature_tracker_params_.detector_config);
  private_nh_.getParam("descriptor_config",
                       feature_tracker_params_.descriptor_config);
  private_nh_.getParam("matcher_config",
                       feature_tracker_params_.matcher_config);
}

}  // namespace visualize_feature_tracks

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(visualize_feature_tracks::TrackerNodelet,
                       nodelet::Nodelet);
