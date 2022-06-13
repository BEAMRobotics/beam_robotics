#pragma once

#include <beam_calibration/CameraModel.h>
#include <beam_calibration/TfTree.h>
#include <beam_colorize/Colorizer.h>
#include <beam_containers/ImageBridge.h>
#include <beam_utils/log.h>

namespace inspection {

using PoseType = std::pair<ros::Time, Eigen::Affine3d>;

struct Image {
  explicit Image(const std::string& image_directory);

  Eigen::Affine3d T_MAP_CAMERA;
  std::string image_info_path;
  beam_containers::ImageBridge image_container;
};

/**
 * @brief Struct for holding information relevant for every camera being used
 * for labeling
 */
struct Camera {
  /**
   * @brief Constructor
   * @param camera_name ID of camera being instantiated (e.g., "F1",
   * @param intrinsics_filename Path to folder containing all camera
   * intrinsics files (e.g., .../calibrations/)
   * @param images_directory Path to root folder containing CamerasList.json
   * (e.g.,
   * .../inspection/images)
   * @param colorizer_type type of colorizer to use (e.g., Override)
   * @param images_distorted set to true if your images are distorted (i.e.,
   * have not been undistorted)
   * @param selected_images vector of image names to use for map labeling. These
   * filenames must match the image container folder name. (e.g., ImageBridge1,
   * ImageBridge2, ...). If none are provided, it wll use all images contained
   * in the ImagesList.json
   */
  Camera(const std::string& camera_name, const std::string& intrinsics_filename,
         const std::string& images_directory, const std::string& colorizer_type,
         bool images_distorted,
         const std::vector<std::string>& selected_images =
             std::vector<std::string>());

  Camera() = default;

  /**
   * @brief fill the images poses using the time stamps in the image container
   * and the tf trees for poses and extrinsics
   *
   * @param extinsics_tree
   * @param poses_tree
   * @param poses_moving_frame
   * @param poses_fixed_frame
   */
  void FillPoses(const beam_calibration::TfTree& extinsics_tree,
                 const beam_calibration::TfTree& poses_tree,
                 const std::string& poses_moving_frame,
                 const std::string& poses_fixed_frame = "Map");

  std::string name;
  std::string images_metadata_path;
  std::string intrinsics_path;
  std::shared_ptr<beam_calibration::CameraModel> cam_model;
  std::vector<Image> images;
  std::unique_ptr<beam_colorize::Colorizer> colorizer;
};

/**
 * @brief Load a vector of cameras given a json config which is the part of the
 * MapLabeler config under the field "cameras"
 *
 * @param camera_config_json list of json objects having the fields: Name,
 * Intrinsics, Enabled, ImagesDistorted, SelectedImages
 * @param images_directory
 * @param intrinsics_directory
 * @param colorizer_type
 * @return std::vector<Camera>
 */
std::vector<Camera> LoadCameras(const nlohmann::json& camera_config_json,
                                const std::string& images_directory,
                                const std::string& intrinsics_directory,
                                const std::string& colorizer_type);

} // namespace inspection
