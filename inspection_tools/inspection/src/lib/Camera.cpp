#include <inspection/Camera.h>

#include <beam_utils/filesystem.h>

namespace inspection {

Image::Image(const std::string& image_directory) {
  image_container.LoadFromJSON(image_directory);
  image_info_path = image_directory;
}

Camera::Camera(const std::string& camera_name,
               const std::string& intrinsics_filename,
               const std::string& images_directory,
               const std::string& colorizer_type,
               const std::vector<std::string>& selected_images)
    : name(camera_name), intrinsics_path(intrinsics_filename) {
  BEAM_DEBUG("Creating camera: {}", name);
  cam_model = beam_calibration::CameraModel::Create(intrinsics_path);

  nlohmann::json J;
  if (!beam::ReadJson(images_directory + "/" + camera_name + "/ImagesList.json",
                      J)) {
    throw std::runtime_error{"invalid images list metadata file path"};
  }
  std::vector<std::string> images_list;
  if (selected_images.empty()) {
    std::vector<std::string> tmp = J.at("Items");
    images_list = tmp;
    BEAM_INFO("loading all {} images in metadata file", images_list.size());
  } else {
    images_list = selected_images;
    BEAM_INFO("loading selected {} image IDs", images_list.size());
  }

  for (const std::string& image_name : images_list) {
    images.push_back(
        Image(images_directory + "/" + camera_name + "/" + image_name));
  }

  BEAM_DEBUG("Successfully constructed camera: {}!", name);

  BEAM_DEBUG("Creating {} colorizer object", colorizer_type);
  if (colorizer_type == "Projection") {
    colorizer = beam_colorize::Colorizer::Create(
        beam_colorize::ColorizerType::PROJECTION);
  } else if (colorizer_type == "RayTrace") {
    colorizer = beam_colorize::Colorizer::Create(
        beam_colorize::ColorizerType::RAY_TRACE);
  } else if (colorizer_type == "Projection2") {
    colorizer = beam_colorize::Colorizer::Create(
        beam_colorize::ColorizerType::PROJECTION_OCCLUSION_SAFE);
  }
  colorizer->SetIntrinsics(cam_model);
}

void Camera::FillPoses(const beam_calibration::TfTree& extinsics_tree,
                       const beam_calibration::TfTree& poses_tree,
                       const std::string& poses_moving_frame,
                       const std::string& poses_fixed_frame) {
  for (Image& image : images) {
    Eigen::Affine3d T_MAP_BASELINK =
        poses_tree.GetTransformEigen(poses_fixed_frame, poses_moving_frame,
                                     image.image_container.GetRosTime());
    Eigen::Affine3d T_BASELINK_CAM = extinsics_tree.GetTransformEigen(
        poses_moving_frame, cam_model->GetFrameID());
    image.T_MAP_CAMERA = T_MAP_BASELINK * T_BASELINK_CAM;
  }
}

std::vector<Camera> LoadCameras(const nlohmann::json& camera_config_json_list,
                                const std::string& images_directory,
                                const std::string& intrinsics_directory,
                                const std::string& colorizer_type) {
  std::vector<Camera> cameras;
  for (const auto& camera_config_json : camera_config_json_list) {
    bool camera_enabled;
    std::string instrinsics_filename;
    std::vector<std::string> selected_images;
    std::string camera_name;

    try {
      bool tmp = camera_config_json.at("Enabled");
      camera_enabled = tmp;
      std::string tmp2 = camera_config_json.at("Intrinsics");
      instrinsics_filename = tmp2;
      std::vector<std::string> tmp3 = camera_config_json.at("SelectedImages");
      selected_images = tmp3;
      std::string tmp5 = camera_config_json.at("Name");
      camera_name = tmp5;
    } catch (nlohmann::json::exception& e) {
      BEAM_CRITICAL("Error processing JSON file: Message {}, ID: {}", e.what(),
                    e.id);
    }

    std::string instrinsics_path =
        intrinsics_directory + "/" + instrinsics_filename;

    if (!camera_enabled) {
      BEAM_INFO("skipping camera with name {} as it's set to disabled.",
                camera_name);
      continue;
    }

    if (selected_images.empty() || selected_images[0] == "All") {
      selected_images = std::vector<std::string>();
    }
    cameras.push_back(Camera(camera_name, instrinsics_path, images_directory,
                             colorizer_type, selected_images));
  }

  return cameras;
}

} // end namespace inspection
