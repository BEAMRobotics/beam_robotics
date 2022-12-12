#include <inspection/ImageDatabase.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include <beam_utils/filesystem.h>
#include <beam_utils/log.h>
#include <beam_utils/time.h>

namespace inspection {

ImageDatabase::ImageDatabase(const std::string& cameras_list_path,
                             const ImageContainerType& image_container_type =
                                 ImageContainerType::IMAGE_BRIDGE)
    : cameras_list_path_(cameras_list_path),
      image_container_type_(image_container_type) {
  boost::filesystem::path p(cameras_list_path);
  root_directory_ = p.parent_path().string();
}

void ImageDatabase::SetImagesFilename(const std::string& images_filename) {
  images_filename_ = images_filename;
}

std::string ImageDatabase::GetImagesFilename() {
  return images_filename_;
}

std::string GetCamerasListPath() {
  return cameras_list_path_;
}

void ImageDatabase::LoadMetadata(const std::string& cameras_list_path) {
  //
}
void ImageDatabase::WriteMetadata(const std::string& cameras_list_path) {
  for (const auto& [camera_name, images_list] : cameras_list_) {
    for (const auto& image : images_list) {
      if (image_object_list_.size() > 0) {
        //////////////////
        // TODO:: Update
        nlohmann::json J;
        J["Images"] = image_object_list_;
        std::ofstream file(beam::CombinePaths(camera_dir, "ImagesList.json"));
        file << std::setw(4) << J << std::endl;
        //////////////////
      }
    }

    //////////////////
    // TODO:: Update
    if (camera_list_.size() > 0) {
      nlohmann::json J;
      J["Cameras"] = camera_list_;
      J["ImagesFilename"] = "ImagesList.json";
      std::ofstream file(
          beam::CombinePaths(save_directory_, "CamerasList.json"));
      file << std::setw(4) << J << std::endl;
    }
    //////////////////
  }
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
    cameras_list_.push_back(camera_name, ImagesList());
    iter = cameras_list_.find(camera_name);
  }

  const ImagesList& images_list = iter->second;
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
    images_list.push_back(image_container_dir)
  } else if (image_container_type_ == ImageContainerType::NONE) {
    std::string image_name =
        "ImageBridge" + std::to_string(images_list.size()) + ".jpg";
    std::string output_file = beam::CombinePaths(camera_dir, image_name);
    cv::imwrite(output_file, image);
    images_list.push_back(image_name)
  } else {
    BEAM_CRITICAL(
        "Invalid image container type. Options are: IMAGE_BRIDGE and NONE");
    throw std::invalid_argument{"Invalid image container type"};
  }
}

} // end namespace inspection
