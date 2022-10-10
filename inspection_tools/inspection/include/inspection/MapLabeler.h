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

#include <beam_colorize/Projection.h>
#include <beam_containers/PointBridge.h>
#include <beam_utils/pointclouds.h>

#include <inspection/Camera.h>

namespace inspection {

using BridgePoint = beam_containers::PointBridge;
using DefectCloud = pcl::PointCloud<beam_containers::PointBridge>;
using PCLViewer = pcl::visualization::PCLVisualizer::Ptr;
using PointCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;

// map: image timestamp in Ns -> defect cloud in map frame
using DefectCloudsMapType = std::unordered_map<int64_t, DefectCloud::Ptr>;

// map: image timestamp in Ns -> T_MAP_CAMERA
using TransformMapType = std::unordered_map<int64_t, Eigen::Affine3d>;

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
    float crack_detector_precision{0.7};
    float corrosion_detector_precision{0.7};
    float spall_detector_precision{0.7};
    float delam_detector_precision{0.7};

    void Print() const;
  };

  explicit MapLabeler(const Inputs& inputs);

  MapLabeler() = default;

  ~MapLabeler() = default;

  /**
   * @brief calls LabelColor -> LabelDefects -> CombineClouds and then outputs
   * all information
   * @param output_folder where to save labeled clouds and summary if not set to
   * false
   * @param save_labeled_clouds save the individual clouds produced by each
   * image
   * @param output_summary output summary of defect clouds
   */
  DefectCloud::Ptr RunFullPipeline(const std::string& output_folder = "",
                                   bool save_labeled_clouds = false,
                                   bool output_summary = false) const;

  /**
   * @brief get the defect clouds by projecting all map points into each image
   * and keeping only the points that land in the image plane.
   * @param defect_clouds reference to defect cloud maps: camera name -> [image
   * timestamp -> cloud]
   */
  std::unordered_map<std::string, DefectCloudsMapType> GetDefectClouds() const;

  /**
   * @brief Adds color information from RGB image in container. Each defect
   * cloud needs to contain only points that project into that image. These
   * clouds can be retrieved using GetDefectClouds function
   * @param defect_clouds map cameral_name -> DefectCloudsMap, where
   * DefectCloudsMap: map image timestamp -> defect cloud
   * @param remove_unlabeled will remove all points that have no label on any of
   * the point fields
   */
  void LabelColor(
      std::unordered_map<std::string, DefectCloudsMapType>& defect_clouds,
      bool remove_unlabeled) const;

  /**
   * @brief Adds defect mask information from RGB Masks in container. Each
   * defect cloud needs to contain only points that project into that image
   * @param defect_clouds map cameral_name -> DefectCloudsMap, where
   * DefectCloudsMap: map image timestamp -> defect cloud
   * @param remove_unlabeled will remove all points that have no label on any of
   * the point fields
   */
  void LabelDefects(
      std::unordered_map<std::string, DefectCloudsMapType>& defect_clouds,
      bool remove_unlabeled) const;

  /**
   * @brief Combine clouds into a final map
   * @param defect_clouds map cameral_name -> DefectCloudsMap, where
   * DefectCloudsMap: map image timestamp -> defect cloud
   */
  DefectCloud::Ptr CombineClouds(
      const std::unordered_map<std::string, DefectCloudsMapType>& defect_clouds,
      const std::string& output_dir = "") const;

  /**
   * @brief Print current configuration
   */
  void PrintConfiguration() const;

  /**
   * @brief Draw the final labeled map in the PCL viewer
   */
  void DrawFinalMap(const DefectCloud::Ptr& map) const;

  /**
   * @brief Saves each of the labeled defect clouds in a clouds folder inside
   * the images folder. This also saves trajectories and camera poses for each
   * image drawn
   */
  void SaveLabeledClouds(
      const std::unordered_map<std::string, DefectCloudsMapType>& defect_clouds,
      const std::string& output_folder) const;

  /**
   * @brief save  the final labeled map as pcd
   */
  void SaveFinalMap(const DefectCloud::Ptr& map,
                    const std::string& output_folder) const;

  /**
   * @brief save calculated camera poses for each images used in the labeling.
   * This saves the RGB frame for the camera frame and baselink frame for each
   * image, it also draws the camera frustums and outputs them all as PCD files
   */
  void SaveCameraPoses(const std::string& output_folder) const;

  /**
   * @brief save images used for map labeling
   */
  void SaveImages(const std::string& output_folder) const;

  /**
   * @brief save a summary json file including camera info and their labeled
   * clouds
   */
  void OutputSummary(
      const std::unordered_map<std::string, DefectCloudsMapType>& defect_clouds,
      const std::string& output_folder) const;

  /**
   * @brief copy config to output directory
   */
  void OutputConfig(const std::string& output_folder) const;

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
   * @brief Get the points in the map that are visible to a camera using the
   * standard projection method. Note this does not take occlusions into account
   * @param projection dummy projection used to determine which points project
   * into image plane
   * @param T_CAMERA_MAP
   * @return DefectCloud::Ptr visible points in camera frame
   */
  DefectCloud::Ptr
      GetMapVisibleByCam(const beam_colorize::Projection& projection,
                         const Eigen::Affine3d& T_CAMERA_MAP) const;

  /**
   * @brief Labels point cloud map with image specified
   * @param defect_cloud cloud to add results to
   * @param image
   * @param camera
   * @return Labeled point cloud map in map frame containing only map points
   * that were labeled with this image
   */
  void ProjectImgRGBToMap(DefectCloud::Ptr& defect_cloud, const Image& image,
                          const Camera& camera, bool remove_unlabeled) const;

  /**
   * @brief Labels point cloud map with image specified
   * @param defect_cloud cloud to add results to
   * @param image
   * @param camera
   * @return Labeled point cloud map in map frame containing only map points
   * that were labeled with this image
   */
  void ProjectImgRGBMaskToMap(DefectCloud::Ptr& defect_cloud,
                              const Image& image, const Camera& camera,
                              bool remove_unlabeled) const;

  /**
   * @brief Get the stats (count of features) of a defect cloud
   * @param cloud input defect cloud
   * @return DefectCloudStats
   */
  DefectCloudStats GetDefectCloudStats(const DefectCloud::Ptr& cloud) const;

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
  DefectCloud::Ptr input_map_ = std::make_shared<DefectCloud>();
  std::vector<Camera> cameras_;
};

} // namespace inspection
