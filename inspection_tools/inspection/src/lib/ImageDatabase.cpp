#include <inspection/ImageDatabase.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <beam_containers/ImageBridge.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/log.h>
#include <beam_utils/time.h>

namespace inspection {

ImageDatabase::ImageDatabase(const std::string& camera_list_path,
                             const ImageContainerType& image_container_type)
    : camera_list_path_(camera_list_path),
      image_container_type_(image_container_type) {
  boost::filesystem::path p(camera_list_path);
  root_directory_ = p.parent_path().string();
}

CameraList::iterator ImageDatabase::CamerasBegin() const {
  return camera_list_.begin();
}

CameraList::iterator ImageDatabase::CamerasEnd() const {
  return camera_list_.end();
}

void ImageDatabase::SetImagesFilename(const std::string& images_filename) {
  images_filename_ = images_filename;
}

std::string ImageDatabase::GetImagesFilename() const {
  return images_filename_;
}

std::string ImageDatabase::GetCameraListPath() const {
  return camera_list_path_;
}

void ImageDatabase::LoadMetadata() {
  BEAM_INFO("Reading image database metadata from cameras list: {}",
            camera_list_path_);
  nlohmann::json J;
  if (!beam::ReadJson(camera_list_path_, J)) {
    BEAM_CRITICAL("Unable to load json");
    throw std::runtime_error{"unable to load json"};
  }

  images_filename_ = J["ImagesFilename"];
  for (const std::string& camera_name : J["Cameras"]) {
    std::string camera_root = beam::CombinePaths(root_directory_, camera_name);
    std::string image_list_path =
        beam::CombinePaths(camera_root, images_filename_ + ".json");
    ImageList image_list = ReadImageList(image_list_path);
    camera_list_.emplace(camera_name, image_list);
  }
}

ImageList
    ImageDatabase::ReadImageList(const std::string& image_list_path) const {
  BEAM_INFO("Reading images list: {}", image_list_path);
  nlohmann::json J;
  if (!beam::ReadJson(image_list_path, J)) {
    BEAM_CRITICAL("Unable to load json");
    throw std::runtime_error{"unable to load json"};
  }

  ImageList list = J["Images"];
  return list;
}

void ImageDatabase::WriteMetadata() const {
  std::vector<std::string> cameras_name_list;
  for (const auto& [camera_name, image_list] : camera_list_) {
    std::string camera_dir = beam::CombinePaths(root_directory_, camera_name);
    std::string json_filename =
        beam::CombinePaths(camera_dir, images_filename_ + ".json");
    nlohmann::json J;
    J["Images"] = image_list;
    std::ofstream file(json_filename);
    file << std::setw(4) << J << std::endl;
    cameras_name_list.push_back(camera_name);
  }
  nlohmann::json J;
  J["Cameras"] = cameras_name_list;
  J["ImagesFilename"] = images_filename_;
  std::ofstream file(camera_list_path_);
  file << std::setw(4) << J << std::endl;
}

void ImageDatabase::AddImage(const std::string& camera_name,
                             const cv::Mat& image, const ros::Time& time,
                             bool is_distorted, const std::string& frame_id,
                             bool is_ir_image,
                             const std::string& dataset_name) {
  auto iter = camera_list_.find(camera_name);
  if (iter == camera_list_.end()) {
    std::string camera_dir = beam::CombinePaths(root_directory_, camera_name);
    BEAM_INFO("creating camera folder:  {}.", camera_dir);
    boost::filesystem::create_directories(camera_dir);
    camera_list_.emplace(camera_name, ImageList());
    iter = camera_list_.find(camera_name);
  }

  ImageList& image_list = iter->second;
  std::string camera_dir = beam::CombinePaths(root_directory_, camera_name);
  if (image_container_type_ == ImageContainerType::IMAGE_BRIDGE) {
    std::string image_name = "ImageBridge" + std::to_string(image_list.size());
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
    image_container.SetImageSeq(image_list.size());
    image_container.SetBagName(dataset_name);
    image_container.Write(image_container_dir);
    image_list.push_back(image_name);
  } else if (image_container_type_ == ImageContainerType::NONE) {
    std::string image_name =
        "ImageBridge" + std::to_string(image_list.size()) + ".jpg";
    std::string output_file = beam::CombinePaths(camera_dir, image_name);
    cv::imwrite(output_file, image);
    image_list.push_back(image_name);
  } else {
    BEAM_CRITICAL(
        "Invalid image container type. Options are: IMAGE_BRIDGE and NONE");
    throw std::invalid_argument{"Invalid image container type"};
  }
}

} // end namespace inspection
