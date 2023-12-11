#include <inspection/Camera.h>

#include <filesystem>

#include <beam_utils/filesystem.h>

namespace inspection {

Image::Image(const std::string& image_directory) {
  image_container.LoadFromJSON(image_directory);
  image_info_path = image_directory;
}

Camera::Camera(const std::string& camera_name,
               const std::string& intrinsics_filename,
               const std::string& images_filepath,
               const std::string& colorizer_type,
               const std::vector<std::string>& selected_images)
    : name(camera_name), intrinsics_path(intrinsics_filename) {
  BEAM_INFO("Creating camera: {}", name);
  cam_model = beam_calibration::CameraModel::Create(intrinsics_path);

  nlohmann::json J;
  BEAM_INFO("reading images from {}", images_filepath);
  if (!beam::ReadJson(images_filepath, J)) {
    BEAM_CRITICAL("cannot find images list file: {}", images_filepath);
    throw std::runtime_error{"invalid images list metadata file path"};
  }

  std::vector<std::string> image_list;
  if (selected_images.empty()) {
    std::vector<std::string> tmp = J.at("Images");
    image_list = tmp;
    BEAM_INFO("loading all {} images in metadata file", image_list.size());
  } else {
    image_list = selected_images;
    BEAM_INFO("loading selected {} image IDs", image_list.size());
  }

  std::filesystem::path p(images_filepath);
  std::string save_path = p.parent_path().string();
  for (const std::string& image_name : image_list) {
    images.push_back(Image(beam::CombinePaths(save_path, image_name)));
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

std::vector<Camera>
    LoadCameras(const std::vector<nlohmann::json>& camera_config_json_list,
                const std::string& cameras_json_path,
                const std::string& intrinsics_directory,
                const std::string& colorizer_type) {
  std::string cameras_directory =
      std::filesystem::path(cameras_json_path).parent_path().string();

  BEAM_INFO("reading  cameras list from: {} ", cameras_json_path);
  nlohmann::json J;
  if (!beam::ReadJson(cameras_json_path, J)) {
    throw std::runtime_error{"invalid json file path"};
  }
  beam::ValidateJsonKeysOrThrow({"Cameras", "ImagesFilename"}, J);
  std::set<std::string> camera_names_from_json;
  std::vector<std::string> camera_names_from_json_v = J["Cameras"];
  for (const auto& v : camera_names_from_json_v) {
    camera_names_from_json.emplace(v);
  }
  std::string images_filename = J["ImagesFilename"];

  std::vector<Camera> cameras;
  for (const auto& camera_config_json : camera_config_json_list) {
    beam::ValidateJsonKeysOrThrow(
        {"Enabled", "Intrinsics", "SelectedImages", "Name"},
        camera_config_json);

    std::string camera_name = camera_config_json.at("Name");

    if (camera_names_from_json.find(camera_name) ==
        camera_names_from_json.end()) {
      BEAM_ERROR("Camera {} from map labeler config is not available in "
                 "cameras list file: {}",
                 camera_name, cameras_json_path);
      throw std::runtime_error{"inconsistent json files"};
    }
    bool camera_enabled = camera_config_json.at("Enabled");
    if (!camera_enabled) {
      BEAM_INFO("skipping camera with name {} as it's set to disabled.",
                camera_name);
      continue;
    }

    std::string instrinsics_filename = camera_config_json.at("Intrinsics");
    std::string instrinsics_path =
        beam::CombinePaths(intrinsics_directory, instrinsics_filename);
    std::vector<std::string> selected_images =
        camera_config_json.at("SelectedImages");
    if (selected_images.empty() || selected_images[0] == "All") {
      selected_images = std::vector<std::string>();
    }

    std::string camera_path =
        beam::CombinePaths(cameras_directory, camera_name);
    std::string images_list_json =
        beam::CombinePaths(camera_path, images_filename + ".json");
    cameras.push_back(Camera(camera_name, instrinsics_path, images_list_json,
                             colorizer_type, selected_images));
  }

  return cameras;
}

const Image& Camera::GetImageByTimestamp(int64_t timestamp_in_Ns) const {
  for (const Image& img : images) {
    if (img.image_container.GetRosTime().toNSec() == timestamp_in_Ns) {
      return img;
    }
  }
  BEAM_CRITICAL("no image found with timestamp {}Ns for camera {}",
                timestamp_in_Ns, name);
  throw std::runtime_error{"no image found"};
}

} // end namespace inspection
