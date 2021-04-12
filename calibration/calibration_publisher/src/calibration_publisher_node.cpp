#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <boost/filesystem.hpp>

#include <beam_calibration/TfTree.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "calibration_publisher");

  ros::NodeHandle node_handle("~");

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  std::string robot_name;
  std::string extrinsics_file_path;
  if (!node_handle.getParam("robot_name", robot_name)) {
    ROS_ERROR("Failed to load parameter");
    return -1;
  }
  if (!node_handle.getParam("extrinsics_file_path", extrinsics_file_path)) {
    ROS_ERROR("Failed to load parameter");
    return -1;
  }
  ROS_INFO("Finished loading parameters");

  std::string calibration_filepath;
  if (extrinsics_file_path.empty()) {
    std::string current_filepath =
        "calibration_publisher/src/calibration_publisher_node.cpp";

    calibration_filepath = __FILE__;
    calibration_filepath.erase(calibration_filepath.end() -
                                   current_filepath.size(),
                               calibration_filepath.end());
    calibration_filepath += "results/";
    calibration_filepath += robot_name;
    calibration_filepath += "/current/extrinsics/extrinsics.json";
  } else {
    calibration_filepath = extrinsics_file_path;
  }

  // check file exists
  if(!boost::filesystem::exists(calibration_filepath)){
    ROS_ERROR("Extrinsics file path does not exist: %s", calibration_filepath.c_str());
    throw std::invalid_argument{"Extrinsics file path does not exist."};
  }

  beam_calibration::TfTree tf_tree;
  tf_tree.LoadJSON(calibration_filepath);

  auto frames = tf_tree.GetAllFrames();
  std::string child_frame;
  for (auto frame : frames) {
    child_frame = frame.first;
    for (auto parent_frame : frame.second) {
      ros::Time time_now = ros::Time::now();
      auto tf_msg =
          tf_tree.GetTransformROS(parent_frame, child_frame, time_now);
      static_broadcaster.sendTransform(tf_msg);
    }
  }

  ROS_INFO("Successfully calibration publisher launched node.");
  ros::spin();

  return 0;
}
