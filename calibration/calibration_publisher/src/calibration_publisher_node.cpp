#include <ros/ros.h>
#include <beam_calibration/TfTree.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "calibration_publisher");

  ros::NodeHandle node_handle("~");

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  std::string robot_name;
  if (!node_handle.getParam("robot_name", robot_name)) {
    ROS_ERROR("Failed to load parameter");
  }

  ROS_INFO("Finished loading parameters");
  robot_name = "inspector_gadget";

  std::string current_filepath = "calibration_publisher/src/calibration_publisher_node.cpp";

  std::string calibration_filepath = __FILE__;
  std::cout << calibration_filepath << std::endl;
  calibration_filepath.erase(calibration_filepath.end() -
                                 current_filepath.size(),
                             calibration_filepath.end());
  calibration_filepath += "results/";
  calibration_filepath += robot_name;
  calibration_filepath += "/current/extrinsics/extrinsics.json";
  std::cout << calibration_filepath << std::endl;

  beam_calibration::TfTree tf_tree;

  tf_tree.LoadJSON(calibration_filepath);

  auto frames = tf_tree.GetAllFrames();
  std::string parent_frame;
  for (auto frame : frames) {
    parent_frame = frame.first;
    for (auto child_frame : frame.second) {
      auto tf_msg = tf_tree.GetTransform(parent_frame, child_frame, ros::Time::now());
      static_broadcaster.sendTransform(tf_msg);
    }
  }

  ROS_INFO("Successfully launched node.");
  ros::spin();

  return 0;
}
