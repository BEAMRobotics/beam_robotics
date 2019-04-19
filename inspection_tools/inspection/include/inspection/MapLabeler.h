#pragma once

#include <Eigen/Eigen>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>
#include <fstream>
#include <beam_utils/log.hpp>

#include <beam_containers/ImageBridge.h>
#include <beam_containers/PointBridge.h>
#include <beam_calibration/TfTree.h>
#include <beam_calibration/Intrinsics.h>
#include <beam_calibration/Pinhole.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/make_shared.hpp>
#include <nlohmann/json.hpp>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>

#include <rosbag/bag.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <beam_utils/time.hpp>

namespace inspection {


using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;
using BridgePoint = beam_containers::PointBridge;
using DefectCloud = pcl::PointCloud<beam_containers::PointBridge>;
using json = nlohmann::json;
using PCLViewer = pcl::visualization::PCLVisualizer::Ptr;
/**
 * @brief class for labeling/coloring a SLAM map given beam image containers
 */
class MapLabeler {
public:
  MapLabeler(const std::string config_file_location);

  MapLabeler() = default;

  ~MapLabeler() = default;
  // Load JSON file containing image information
  // For each image, create image container
  //
  void LoadPrevPoses();


  std::vector<std::pair<uint64_t, Eigen::Matrix4d>>
      ReadPoseFile(const std::string filename);

  DefectCloud::Ptr TransformMapToImage(ros::Time tf_time);

  DefectCloud::Ptr ProjectImgToMap(beam_containers::ImageBridge img);

  pcl::visualization::PCLVisualizer::Ptr viewer;

  void PlotFrames(std::string frame_id, PCLViewer viewer);

private:

  beam_calibration::TfTree tf_tree;

  std::string json_file_path_ = {};
  json json_config_ = {};

  std::string poses_file_name_ = {};
  std::string map_file_name_ = {};
  std::string images_file_name_ = {};
  std::string extrinsics_file_name_ = {};
  std::string path_to_camera_calib_ = {};

  std::vector<std::pair<TimePoint, Eigen::Affine3d>> final_poses_;
  DefectCloud::Ptr defect_pointcloud_ = boost::make_shared<DefectCloud>();

  std::vector<DefectCloud::Ptr> defect_clouds_ = {};
  beam_containers::ImageBridge img_bridge_;
  tf2::BufferCore tf2_buffer_{ros::Duration(1000)};

  tf::Transform tf_temp_;
};


} // namespace inspection
