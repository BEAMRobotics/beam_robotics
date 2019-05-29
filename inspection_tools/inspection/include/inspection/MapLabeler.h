#pragma once

#include <beam_calibration/CameraModel.h>
#include <beam_calibration/TfTree.h>
#include <beam_colorize/Colorizer.h>
#include <beam_containers/ImageBridge.h>
#include <beam_containers/PointBridge.h>
#include <beam_containers/Utilities.h>
#include <beam_utils/log.hpp>
#include <beam_utils/time.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/transforms.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include "inspection/CloudCombiner.h"

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
class MapLabeler {
  /**
   * @brief Struct for holding information relevant for every camera being used
   * for labeling
   */
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
    Camera(std::string root_folder_path, std::string cam_id,
           std::string intrinsics_path)
        : camera_id(cam_id), folder_path(root_folder_path) {
      using namespace beam_calibration;

      // Get the .json intrinsics file for the specified camera
      std::string camera_intrinsics_path = intrinsics_path + cam_id + ".json";
      BEAM_DEBUG("Camera intrinsics path: {}", camera_intrinsics_path);

      json intrinsics_json;
      std::ifstream json_config_stream(camera_intrinsics_path);
      json_config_stream >> intrinsics_json;

      cam_model = CameraModel::LoadJSON(camera_intrinsics_path);

      // Next we create/fill in a string vector which will store the path to
      // each image folder for our camera (this is used for instantiating image
      // container objects)
      json json_images_list;
      std::ifstream i(folder_path + "/ImagesList.json");
      i >> json_images_list;
      BEAM_DEBUG("Loading {} images from: {}", json_images_list["Items"].size(),
                 folder_path);
      for (const auto& image_folder : json_images_list["Items"]) {
        img_paths.emplace_back(folder_path + "/" + std::string(image_folder));
      }

      /**
       * @todo add_colorizer_param
       * @body In one of the config .json files we need to specify what type of
       * colorizer we want to use for each camera - then we can read that in
       * here and call the factory method with the appropriate enum type.
       */
      BEAM_DEBUG("Creating colorizer object");
      colorizer = beam_colorize::Colorizer::Create(
          beam_colorize::ColorizerType::PROJECTION);
      colorizer->SetIntrinsics(cam_model);
      colorizer->SetDistortion(true);
      BEAM_DEBUG("Sucessfully constructed camera");
    }
    Camera() = default;
    std::string camera_id = {};
    std::string folder_path = {};
    std::vector<std::string> img_paths = {};
    std::shared_ptr<beam_calibration::CameraModel> cam_model;
    std::unique_ptr<beam_colorize::Colorizer> colorizer;
  };

public:
  explicit MapLabeler(std::string config_file_location);

  MapLabeler() = default;

  ~MapLabeler() = default;

  /**
   * @brief Adds the frame_id to viewer for all times specified in the poses
   * file
   * @param frame_id ID of frame being plotted
   * @param viewer Viewer that frames should be added to
   */
  void PlotFrames(std::string frame_id, PCLViewer viewer);

  /**
   * @brief Main method to kick off execution
   */
  void Run();

  /**
   * @brief Print current configuration
   */
  void PrintConfiguration();

  /**
   * @brief Saves each of the labeled defect clouds in a clouds folder inside
   * the images folder
   */
  void SaveLabeledClouds();


  /**
   * @brief Draw the final labeled map in the viewer
   */
  void DrawFinalMap();

  pcl::visualization::PCLVisualizer::Ptr viewer =
      boost::make_shared<pcl::visualization::PCLVisualizer>();

private:
  /**
   * @brief Populate TF tree with dynamic transforms from poses file and with
   * static transforms frome extrinsic calibrations
   */
  void FillTFTree();

  /**
   * @brief Loads the MapLabeler.json config file and sets up data parameters /
   * cameras
   */
  void ProcessJSONConfig();

  /**
   * @brief Transforms map point cloud into an image frame
   * @param tf_time Time to lookup map->frame_id transform
   * @param frame_id Frame that map is being transformed into
   * @return Map cloud in image frame
   */
  DefectCloud::Ptr TransformMapToImageFrame(ros::Time tf_time,
                                            std::string frame_id);

  /**
   * @brief Labels point cloud map with image specified
   * @param img_container Image container used for labeling
   * @param camera Camera corresponding to image container
   * @return Labeled point cloud map
   */
  DefectCloud::Ptr ProjectImgToMap(beam_containers::ImageBridge img_container,
                                   Camera* camera);

  beam_calibration::TfTree tf_tree_;

  std::string json_labeler_filepath_ = {};
  json json_config_ = {};

  std::string poses_file_name_ = {};
  std::string map_file_name_ = {};
  std::string images_file_name_ = {};
  std::string extrinsics_file_name_ = {};
  std::string path_to_camera_calib_ = {};

  std::vector<std::string> camera_list_ = {};

  std::vector<std::pair<TimePoint, Eigen::Affine3d>> final_poses_;
  DefectCloud::Ptr defect_pointcloud_ = boost::make_shared<DefectCloud>();

  std::vector<std::vector<DefectCloud::Ptr>> defect_clouds_ = {};

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> rgb_clouds_ = {};
  beam_containers::ImageBridge img_bridge_;

  std::vector<Camera> cameras_;
  tf::Transform tf_temp_;
  inspection::CloudCombiner cloud_combiner_;

}; // namespace inspection

} // namespace inspection
