#include <ros/ros.h>
#include <beam_calibration/TfTree.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "calibration_publisher");

  ros::NodeHandle node_handle("~");

  std::string robot_name;
  if (!node_handle.getParam("robot_name", robot_name)) {
    ROS_ERROR("Failed to load parameter");
  }
  ROS_INFO("Finished loading parameters");

  std::string current_filepath = "calibration_util/src/calibration_publisher_node.cpp";

  std::string calibration_filepath = __FILE__;
  calibration_filepath.erase(calibration_filepath.end() -
                                 current_filepath.size(),
                             calibration_filepath.end());
  calibration_filepath += "results";
  calibration_filepath += robot_name;
  calibration_filepath += "/current/extrinsics/extrinsics.json";

  beam_calibration::TfTree tf_tree;

  tf_tree.LoadJSON(calibration_filepath);

  std::cout << tf_tree.GetAllFrames() << std::endl;

  ROS_INFO("Successfully launched node.");

  return 0;
}
