#pragma once

#include <pcl/visualization/pcl_visualizer.h>

#include <beam_calibration/CameraModel.h>
#include <beam_calibration/TfTree.h>
#include <beam_utils/pointclouds.h>

namespace inspection {

using PCLViewer = pcl::visualization::PCLVisualizer::Ptr;

class CameraToMapAligner {
public:
  struct Inputs {
    std::string map;
    std::string poses_moving_frame_override;
    std::string image_container;
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
  /**
   * @brief Populate TF trees, one for the poses (used for interpolation) and
   * one for the extrinsics. Note we don't combine these because it could break
   * the tree depending on which frames are used in the pose measurements
   */
  void FillTFTrees();

  Inputs inputs_;

  pcl::visualization::PCLVisualizer::Ptr viewer_ =
      std::make_shared<pcl::visualization::PCLVisualizer>();
  beam_calibration::TfTree extinsics_original_;
  beam_calibration::TfTree extinsics_edited_;
  beam_calibration::TfTree poses_tree_;
  std::unique_ptr<beam_calibration::CameraModel> camera_model_;
  std::string poses_moving_frame_;
  std::string poses_fixed_frame_;
  PointCloud::Ptr map_ = std::make_shared<PointCloud>();
  PointCloudCol::Ptr map_colored_ = std::make_shared<PointCloudCol>();
  Eigen::Matrix4d T_map_moving_;
  Eigen::Matrix4d T_moving_reference_;

  /** @brief this is the transform that we are going to be editing. Note that
   * this transform may break the tree if the user doesn't input a valid
   * reference_frame */
  Eigen::Matrix4d T_reference_camera_;

  // params tunable here
  double coordinateFrameScale_{0.5};
};

} // namespace inspection
