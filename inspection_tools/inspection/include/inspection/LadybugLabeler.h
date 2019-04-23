#pragma once

#include <Eigen/Eigen>
#include <beam_utils/log.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <beam_calibration/Intrinsics.h>
#include <beam_colorize/Colorizer.h>

#include <beam_calibration/TfTree.h>
#include <beam_containers/ImageBridge.h>
#include <beam_containers/PointBridge.h>
#include <beam_utils/time.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/make_shared.hpp>
#include <nlohmann/json.hpp>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include <rosbag/bag.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/transforms.h>

#include "inspection/CloudCombiner.h"
#include <beam_calibration/Ladybug.h>
#include <beam_utils/time.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace inspection {

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;
using BridgePoint = beam_containers::PointBridge;
using DefectCloud = pcl::PointCloud<beam_containers::PointBridge>;
using json = nlohmann::json;
using PCLViewer = pcl::visualization::PCLVisualizer::Ptr;
using PointCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZRGB>;

/**
 * @brief class for labeling/coloring a SLAM map given beam image containers
 */
class LadybugLabeler {
  struct Camera {
    /**
     * @brief Constructor
     * @param folder_path Path to root folder containing CamerasList.json (e.g.,
     * .../inspection/images)
     * @param cam_id ID of camera being instantiated (e.g., "F1", corresponding
     * to a folder such as .../inspection/images/F1)
     * @param intrinsics_path Path to folder containing all camera intrinsics
     * files (e.g., .../calibrations/)
     */
    Camera(int cam_id, std::string intrinsics_path) : camera_id(cam_id) {
      using namespace beam_calibration;

      // Check camera type from JSON and use intrinsics Factory method to
      // instantiate correct type
      cam_intrinsics = Intrinsics::Create(IntrinsicsType::LADYBUG, cam_id);

      cam_intrinsics->LoadJSON(intrinsics_path);

      /**
       * @todo add_colorizer_param
       * @body In one of the config .json files we need to specify what type of
       * colorizer we want to use for each camera - then we can read that in
       * here and call the factory method with the appropriate enum type.
       */
      colorizer = beam_colorize::Colorizer::Create(
          beam_colorize::ColorizerType::PROJECTION);
      colorizer->SetIntrinsics(cam_intrinsics.get());
      colorizer->SetDistortion(true);
    }
    Camera() = default;
    int camera_id = {};
    std::string folder_path_ = {};
    std::vector<std::string> img_paths = {};
    std::unique_ptr<beam_calibration::Intrinsics> cam_intrinsics;
    std::unique_ptr<beam_colorize::Colorizer> colorizer;
  };

public:
  explicit LadybugLabeler(std::string config_file_location);

  LadybugLabeler() = default;

  ~LadybugLabeler() = default;

  DefectCloud::Ptr TransformMapToImageFrame(ros::Time tf_time,
                                            std::string frame_id);

  DefectCloud::Ptr ProjectImgToMap(beam_containers::ImageBridge img_container,
                                   Camera* camera);

  pcl::visualization::PCLVisualizer::Ptr viewer =
      boost::make_shared<pcl::visualization::PCLVisualizer>();

  void PlotFrames(std::string frame_id, PCLViewer viewer);

  void DrawColoredClouds();

  void SaveLabeledClouds();

  void FillTFTree();

  void ProcessJSONConfig();

  void DrawFinalMap();

private:
  beam_calibration::TfTree tf_tree;

  std::string json_labeler_filepath_ = {};
  json json_config_ = {};

  std::string poses_file_name_ = {};
  std::string map_file_name_ = {};
  std::string images_file_name_ = {};
  std::string extrinsics_file_name_ = {};
  std::string path_to_camera_calib_ = {};

  std::vector<std::string> img_container_paths = {};
  std::vector<std::string> camera_list_ = {};

  std::vector<std::pair<TimePoint, Eigen::Affine3d>> final_poses_;
  DefectCloud::Ptr defect_pointcloud_ = boost::make_shared<DefectCloud>();

  std::vector<DefectCloud::Ptr> scan_clouds = {};

  std::vector<std::vector<DefectCloud::Ptr>> defect_clouds_ = {};
  //  std::vector<DefectCloud::Ptr> defect_clouds_ = {};
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> rgb_clouds = {};
  beam_containers::ImageBridge img_bridge_;
  tf2::BufferCore tf2_buffer_{ros::Duration(1000)};

  std::vector<Camera> cameras_;
  tf::Transform tf_temp_;
  inspection::CloudCombiner cloud_combiner_;
}; // namespace inspection

} // namespace inspection
