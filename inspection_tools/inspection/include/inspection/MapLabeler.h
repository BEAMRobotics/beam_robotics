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

// map: image timestamp in Ns -> defect cloud in map frame
using DefectCloudsMapType = std::unordered_map<int64_t, DefectCloud::Ptr>;

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
   * @brief calls LabelColor -> LabelDefects -> CombineClouds and then outputs
   * all information
   */
  RunFullPipeline() const;

  /**
   * @brief Adds RGB color from RGB image in container. If a defect cloud
   * doesn't exist for any image of any camera, this will create one. Otherwise
   * it will fill RGB data of that defect cloud
   * IMPORTANT: if the defect cloud already exists and isn't empty, we will only
   * try to colorize the points in this defect cloud. So make sure the cloud
   * wasn't colored with a different image, or else the kept colored points
   * won't project to the image. This is meant to be used in combination with
   * LabelDefects which are both called on the same image for each defect cloud
   * @param defect_clouds map cameral_name -> DefectCloudsMap, where
   * DefectCloudsMap: map image timestamp -> defect cloud
   */
  void LabelColor(std::unordered_map<std::string, DefectCloudsMapType>&
                      defect_clouds) const;

  /**
   * @brief Adds defect mask information color from RGB Masks in container. If a
   * defect cloud doesn't exist for any image of any camera, this will create
   * one. Otherwise it will fill defect data of that defect cloud
   * IMPORTANT: if the defect cloud already exists and isn't empty, we will only
   * try to label the points in this defect cloud. So make sure the cloud wasn't
   * colored with a different image, or else the kept colored points won't
   * project to the image. This is meant to be used in combination with
   * LabelColor which are both called on the same image for each defect cloud
   * @param defect_clouds map cameral_name -> DefectCloudsMap, where
   * DefectCloudsMap: map image timestamp -> defect cloud
   */
  void LabelDefects(std::unordered_map<std::string, DefectCloudsMapType>&
                        defect_clouds) const;

  /**
   * @brief Combine clouds into a final map
   * @param defect_clouds map cameral_name -> DefectCloudsMap, where
   * DefectCloudsMap: map image timestamp -> defect cloud
   */
  void CombineClouds(const std::unordered_map<std::string, DefectCloudsMapType>&
                         defect_clouds) const;

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
   * @param defect_cloud cloud to add results to
   * @param image
   * @param camera
   * @return Labeled point cloud map in map frame containing only map points
   * that were labeled with this image
   */
  ProjectImgRGBToMap(DefectCloud::Ptr& defect_cloud, const Image& image,
                     const Camera& camera) const;

  /**
   * @brief Labels point cloud map with image specified
   * @param defect_cloud cloud to add results to
   * @param image
   * @param camera
   * @return Labeled point cloud map in map frame containing only map points
   * that were labeled with this image
   */
  ProjectImgRGBMaskToMap(DefectCloud::Ptr& defect_cloud, const Image& image,
                         const Camera& camera) const;

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
  std::vector<Camera> cameras_;
  inspection::CloudCombiner cloud_combiner_;
};

} // namespace inspection
