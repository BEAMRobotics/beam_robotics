#pragma once

#ifndef NDEBUG
#  define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
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
     * @param cam_name ID of camera being instantiated (e.g., "F1",
     * corresponding to a folder such as .../inspection/images/F1)
     * @param cam_intrinsics_path Path to folder containing all camera
     * intrinsics files (e.g., .../calibrations/)
     */
    Camera(json camera_config_json, std::string cam_imgs_folder,
           std::string intrin_folder)
        : camera_name_(camera_config_json.at("Name")),
          cam_imgs_folder_(cam_imgs_folder),
          cam_intrinsics_path_(intrin_folder + camera_name_ + ".json") {
      BEAM_DEBUG("Creating camera: {}", camera_name_);

      using namespace beam_calibration;
      using namespace beam_colorize;
      using namespace boost::filesystem;

      cam_model_ = CameraModel::LoadJSON(cam_intrinsics_path_);
      frame_id_ = cam_model_->GetFrameID();

      // Next we create/fill in a string vector which will store the path to
      // each image folder for our camera (this is used for instantiating image
      // container objects)
      path p{cam_imgs_folder_};
      BEAM_DEBUG("    Getting image paths for camera...");
      for (const auto& imgs : camera_config_json.at("Images")) {
        std::string img_type = imgs.at("Type");
        for (const auto& ids : imgs.at("IDs")) {
          if (ids == "All") {
            for (auto& entry :
                 boost::make_iterator_range(directory_iterator(p), {})) {
              std::string path = entry.path().string();
              img_paths_.emplace_back(path);
              BEAM_DEBUG("      Adding path: {}", path);
            }
          } else {
            img_paths_.emplace_back(cam_imgs_folder_ + "/" + img_type +
                                    std::string(ids));
            camera_pose_ids_.push_back(std::stoi(std::string(ids)));
            BEAM_DEBUG("      Adding path: {}", img_paths_.back());
          }
        }
      }
      std::sort(img_paths_.begin(), img_paths_.end());
      BEAM_DEBUG("    Total image paths: {}", img_paths_.size());

      if (camera_config_json.at("Colorizer") == "Projection") {
        colorizer_ = Colorizer::Create(ColorizerType::PROJECTION);
        colorizer_type_ = "Projection";
      } else if (camera_config_json.at("Colorizer") == "RayTrace") {
        colorizer_ = Colorizer::Create(ColorizerType::RAY_TRACE);
        colorizer_type_ = "RayTrace";
      }
      BEAM_DEBUG("    Creating {} colorizer object", colorizer_type_);

      colorizer_->SetIntrinsics(cam_model_);
      colorizer_->SetDistortion(true);
      BEAM_DEBUG("    Sucessfully constructed camera: {}!", camera_name_);
    }
    Camera() = default;

    std::string colorizer_type_ = {};
    std::string camera_name_ = {};
    std::string cam_imgs_folder_ = {};
    std::string cam_intrinsics_path_ = {};
    std::vector<std::string> img_paths_ = {};
    std::shared_ptr<beam_calibration::CameraModel> cam_model_;
    std::unique_ptr<beam_colorize::Colorizer> colorizer_;
    std::vector<Eigen::Affine3f> transforms_;
    std::vector<uint32_t> camera_pose_ids_;
    std::string frame_id_;
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
   * @brief Fill each camera with a list of its transformations
   */
  void FillCameraPoses();

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

  std::string images_folder_ = {};
  std::string intrinsics_folder_ = {};
  std::string map_path_ = {};
  std::string poses_path_ = {};
  std::string extrinsics_path_ = {};
  std::string final_map_name_ = "_final_map.pcd";
  std::string cloud_combiner_type_ = "Override";
  bool output_individual_clouds_ = false;

  std::vector<std::pair<TimePoint, Eigen::Affine3d>> final_poses_;
  DefectCloud::Ptr defect_pointcloud_ = boost::make_shared<DefectCloud>();

  std::vector<std::vector<DefectCloud::Ptr>> defect_clouds_ = {};

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> rgb_clouds_ = {};
  beam_containers::ImageBridge img_bridge_;

  std::vector<Camera> cameras_;
  tf::Transform tf_temp_;
  inspection::CloudCombiner cloud_combiner_;
};

} // namespace inspection
