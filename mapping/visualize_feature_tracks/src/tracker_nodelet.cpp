#include <visualize_feature_tracks/tracker_nodelet.h>

#include <stdlib.h>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <pluginlib/class_list_macros.h>

#include <visualize_feature_tracks/CameraMeasurementMsg.h>

namespace visualize_feature_tracks {

TrackerNodelet::TrackerNodelet() {}

void TrackerNodelet::onInit() {
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();
  LoadParams();
  
  image_subscriber_ = private_nh_.subscribe(
      image_topic_, 1000, &TrackerNodelet::ImageCallback, this);

  tracks_publisher_ = private_nh_.advertise<CameraMeasurementMsg>(tracks_topic_, 10);    
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
  
  // get tracks & build output message
  std::vector<beam_containers::LandmarkMeasurement> tracks = tracker_.AddImage(image);
  CameraMeasurementMsg camera_measurement_msg;

  // add general info
  camera_measurement_msg.sensor_id = 0;
  camera_measurement_msg.measurement_id = img_msg->header.seq;
  camera_measurement_msg.stamp = img_msg->header.stamp;
  camera_measurement_msg.size = tracks.size();

  // convert data to vector of ros messages & add to output message
  std::vector<LandmarkMeasurementMsg> landmark_measurements;
  for (const beam_containers::LandmarkMeasurement& measurement : tracks){
    LandmarkMeasurementMsg measurement_msg;
    measurement_msg.landmark_id = measurement.landmark_id;
    measurement_msg.pixel_u = measurement.value[0];
    measurement_msg.pixel_v = measurement.value[1];
    landmark_measurements.push_back(measurement_msg);
  }
  camera_measurement_msg.landmarks = landmark_measurements;

  // publish result
  tracks_publisher_.publish(camera_measurement_msg);
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
    ROS_INFO("Could not load tracker_type");
  }
}

}  // namespace visualize_feature_tracks

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(visualize_feature_tracks::TrackerNodelet,
                       nodelet::Nodelet);
