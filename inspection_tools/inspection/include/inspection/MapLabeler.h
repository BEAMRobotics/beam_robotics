#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/transforms.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include <beam_containers/PointBridge.h>
#include <beam_utils/pointclouds.h>

#include <inspection/Camera.h>
#include <inspection/CloudCombiner.h>

namespace inspection {

using BridgePoint = beam_containers::PointBridge;
using DefectCloud = pcl::PointCloud<beam_containers::PointBridge>;
using PCLViewer = pcl::visualization::PCLVisualizer::Ptr;
using PointCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;

struct DefectCloudStats {
  int size;
  int cracks;
  int delams;
  int spalls;
  int corrosions;
};

/**
 * @brief class for labeling/coloring a SLAM map given beam image containers
 */
class MapLabeler {
public:
  struct Inputs {
    std::string images_directory;
    std::string map;
    std::string poses;
    std::string intrinsics_directory;
    std::string extrinsics;
    std::string config_file_location;
    std::string poses_moving_frame_override;

    void Print();
  };

  explicit MapLabeler(const Inputs& inputs);

  MapLabeler() = default;

  ~MapLabeler() = default;

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
   * the images folder. This also saves trajectories and camera poses for each
   * image drawn
   */
  void SaveLabeledClouds(const std::string& output_folder);

  /**
   * @brief save  the final labeled map as pcd
   */
  void SaveFinalMap(const std::string& output_folder);

  /**
   * @brief save calculated camera poses for each images used in the labeling.
   * This saves the RGB frame for the camera frame and baselink frame for each
   * image, it also draws the camera frustums and outputs them all as PCD files
   */
  void SaveCameraPoses(const std::string& output_folder);

  /**
   * @brief save images used for map labeling
   */
  void SaveImages(const std::string& output_folder);

  /**
   * @brief save a summary json file including camera info and their labeled
   * clouds
   */
  void OutputSummary(const std::string& output_folder);

  /**
   * @brief copy config to output directory
   */
  void OutputConfig(const std::string& output_folder);

  /**
   * @brief Draw the final labeled map in the PCL viewer
   */
  void DrawFinalMap();

private:
  /**
   * @brief Loads the MapLabeler.json config file and sets up data parameters /
   * cameras
   */
  void ProcessJSONConfig();

  /**
   * @brief Populate TF trees, one for the poses (used for interpolation) and
   * one for the extrinsics. Note we don't combine these because it could break
   * the tree depending on which frames are used in the pose measurements
   */
  void FillTFTrees();

  /**
   * @brief Iterate through each camera and fill the image poses using the
   * current tf trees and the image timestamps from the image container
   */
  void FillCameraPoses();

  /**
   * @brief Labels point cloud map with image specified
   * @param image
   * @param camera
   * @return Labeled point cloud map in map frame containing only map points
   * that were labeled with this image
   */
  DefectCloud::Ptr ProjectImgToMap(const Image& image, const Camera& camera);

  /**
   * @brief Get the stats (count of features) of a defect cloud
   * @param cloud input defect cloud
   * @return DefectCloudStats
   */
  DefectCloudStats GetDefectCloudStats(const DefectCloud::Ptr& cloud);

  Inputs inputs_;

  // from config file
  std::string colorizer_type_;
  bool depth_enhancement_{false};
  std::string final_map_name_{"_final_map.pcd"};
  std::string cloud_combiner_type_{"Override"};

  // params only tunable here
  double frustum_lengh_{1};
  double draw_points_increment_{0.01};
  float depth_map_extraction_thresh_{0.025};

  // data
  beam_calibration::TfTree extinsics_tree_;
  beam_calibration::TfTree poses_tree_;
  std::string poses_moving_frame_;
  std::string poses_fixed_frame_;
  PointCloud::Ptr map_ = std::make_shared<PointCloud>();
  std::vector<std::vector<DefectCloud::Ptr>> defect_clouds_;
  std::vector<Camera> cameras_;
  inspection::CloudCombiner cloud_combiner_;
};

} // namespace inspection
