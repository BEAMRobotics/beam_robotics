#include <boost/filesystem.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <beam_calibration/TfTree.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "calibration_publisher");

  ros::NodeHandle node_handle("~");
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  std::string extrinsics_file_path;
  node_handle.getParam("extrinsics_file_path", extrinsics_file_path);

  std::string robot_name;
  node_handle.getParam("robot_name", robot_name);

  if (extrinsics_file_path.empty() && robot_name.empty()) {
    BEAM_ERROR("Invalid parameters, robot_name and extrinsics_file_path cannot "
               "both be empty");
  }

  std::string calibration_filepath;
  if (extrinsics_file_path.empty()) {
    ROS_INFO("Extrinsics filepath empty, loading default calibration file for "
             "robot name: %s",
             robot_name.c_str());
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
  if (!boost::filesystem::exists(calibration_filepath)) {
    ROS_ERROR("Extrinsics file path does not exist: %s",
              calibration_filepath.c_str());
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
