#include <inspection/ImageDatabase.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <beam_containers/ImageBridge.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/log.h>
#include <beam_utils/time.h>

namespace inspection {

ImageDatabase::ImageDatabase(const std::string& cameras_list_path,
                             const ImageContainerType& image_container_type)
    : cameras_list_path_(cameras_list_path),
      image_container_type_(image_container_type) {
  boost::filesystem::path p(cameras_list_path);
  root_directory_ = p.parent_path().string();
}

void ImageDatabase::SetImagesFilename(const std::string& images_filename) {
  images_filename_ = images_filename;
}

std::string ImageDatabase::GetImagesFilename() const {
  return images_filename_;
}

std::string ImageDatabase::GetCamerasListPath() const {
  return cameras_list_path_;
}

void ImageDatabase::LoadMetadata() {
  BEAM_INFO("Reading image database metadata from cameras list: {}",
            cameras_list_path_);
  nlohmann::json J;
  if (!beam::ReadJson(cameras_list_path_, J)) {
    BEAM_CRITICAL("Unable to load json");
    throw std::runtime_error{"unable to load json"};
  }

  images_filename_ = J["ImagesFilename"];
  for (const std::string& camera_name : J["Cameras"]) {
    std::string camera_root = beam::CombinePaths(root_directory_, camera_name);
    std::string image_list_path =
        beam::CombinePaths(camera_root, images_filename_ + ".json");
    ImagesList image_list = ReadImagesList(image_list_path);
    cameras_list_.emplace(camera_name, image_list);
  }
}

ImagesList
    ImageDatabase::ReadImagesList(const std::string& image_list_path) const {
  BEAM_INFO("Reading images list: {}", image_list_path);
  nlohmann::json J;
  if (!beam::ReadJson(image_list_path, J)) {
    BEAM_CRITICAL("Unable to load json");
    throw std::runtime_error{"unable to load json"};
  }

  ImagesList list = J["Images"];
  return list;
}

void ImageDatabase::WriteMetadata() const {
  std::vector<std::string> cameras_name_list;
  for (const auto& [camera_name, images_list] : cameras_list_) {
    std::string camera_dir = beam::CombinePaths(root_directory_, camera_name);
    std::string json_filename =
        beam::CombinePaths(camera_dir, images_filename_ + ".json");
    nlohmann::json J;
    J["Images"] = images_list;
    std::ofstream file(json_filename);
    file << std::setw(4) << J << std::endl;
    cameras_name_list.push_back(camera_name);
  }
  nlohmann::json J;
  J["Cameras"] = cameras_name_list;
  J["ImagesFilename"] = images_filename_;
  std::ofstream file(cameras_list_path_);
  file << std::setw(4) << J << std::endl;
}

void ImageDatabase::AddImage(const std::string& camera_name,
                             const cv::Mat& image, const ros::Time& time,
                             bool is_distorted, const std::string& frame_id,
                             bool is_ir_image,
                             const std::string& dataset_name) {
  auto iter = cameras_list_.find(camera_name);
  if (iter == cameras_list_.end()) {
    std::string camera_dir = beam::CombinePaths(root_directory_, camera_name);
    BEAM_INFO("creating camera folder:  {}.", camera_dir);
    boost::filesystem::create_directories(camera_dir);
    cameras_list_.emplace(camera_name, ImagesList());
    iter = cameras_list_.find(camera_name);
  }

  ImagesList& images_list = iter->second;
  std::string camera_dir = beam::CombinePaths(root_directory_, camera_name);
  if (image_container_type_ == ImageContainerType::IMAGE_BRIDGE) {
    std::string image_name = "ImageBridge" + std::to_string(images_list.size());
    beam_containers::ImageBridge image_container;
    std::string image_container_dir =
        beam::CombinePaths(camera_dir, image_name);
    boost::filesystem::create_directories(image_container_dir);

    if (is_ir_image) {
      image_container.SetIRImage(image);
      image_container.SetIRIsDistorted(is_distorted);
      image_container.SetIRFrameId(frame_id);
    } else {
      image_container.SetBGRImage(image);
      image_container.SetBGRIsDistorted(is_distorted);
      image_container.SetBGRFrameId(frame_id);
    }

    beam::TimePoint image_timepoint = beam::RosTimeToChrono(time);
    image_container.SetTimePoint(image_timepoint);
    image_container.SetImageSeq(images_list.size());
    image_container.SetBagName(dataset_name);
    image_container.Write(image_container_dir);
    images_list.push_back(image_name);
  } else if (image_container_type_ == ImageContainerType::NONE) {
    std::string image_name =
        "ImageBridge" + std::to_string(images_list.size()) + ".jpg";
    std::string output_file = beam::CombinePaths(camera_dir, image_name);
    cv::imwrite(output_file, image);
    images_list.push_back(image_name);
  } else {
    BEAM_CRITICAL(
        "Invalid image container type. Options are: IMAGE_BRIDGE and NONE");
    throw std::invalid_argument{"Invalid image container type"};
  }
}

} // end namespace inspection
