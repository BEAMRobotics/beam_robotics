#pragma once

#include <pcl/visualization/pcl_visualizer.h>

#include <beam_calibration/CameraModel.h>
#include <beam_calibration/TfTree.h>
#include <beam_colorize/Colorizer.h>
#include <beam_containers/ImageBridge.h>
#include <beam_utils/pointclouds.h>

namespace inspection {

using PCLViewer = pcl::visualization::PCLVisualizer::Ptr;

class CameraToMapAligner {
public:
  struct Inputs {
    std::string map;
    std::string poses;
    std::string poses_moving_frame_override;
    std::string image_container_root;
    std::string extrinsics;
    std::string intrinsics;
    std::string reference_frame;
    std::string output;
  };

  explicit CameraToMapAligner(const Inputs& inputs);

  ~CameraToMapAligner() = default;

  /**
   * @brief Main method to kick off execution
   */
  void Run();

private:
  void LoadImageContainer();

  void FillTfTrees();

  void AddFixedCoordinateSystems();

  void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event);

  void UpdateExtrinsics(const Eigen::Vector3d& trans,
                        const Eigen::Vector3d& rot);

  void PrintIntructions();

  void UpdateMap();

  void UpdateViewer();

  void OutputUpdatedTransform();

  Inputs inputs_;

  pcl::visualization::PCLVisualizer::Ptr viewer_;
  std::shared_ptr<beam_calibration::CameraModel> camera_model_;
  beam_calibration::TfTree poses_tree_;
  beam_containers::ImageBridge image_container_;
  std::string poses_moving_frame_;
  std::string poses_fixed_frame_;
  PointCloud::Ptr map_ = std::make_shared<PointCloud>();
  PointCloudCol::Ptr map_colored_ = std::make_shared<PointCloudCol>();
  Eigen::Matrix4d T_map_reference_;
  bool quit_{false};

  /** @brief this is the transform that we are going to be editing. Note that
   * this transform may break the tree if the user doesn't input a valid
   * reference_frame */
  Eigen::Matrix4d T_reference_camera_;
  Eigen::Matrix4d T_reference_camera_original_;

  // params
  double coordinateFrameScale_{0.5};
  double sensitivity_r_ = 3; // can be tuned by user
  double sensitivity_t_ = 5; // can be tuned by user
  int point_size_{3};
  double text_scale_{0.05};
  std::vector<double> backgound_rgb_{0.8, 0.8, 0.8};
  std::vector<double> text_rgb_{0, 0, 1};
};

} // namespace inspection
