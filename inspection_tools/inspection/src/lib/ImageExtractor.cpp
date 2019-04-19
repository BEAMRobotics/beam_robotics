#include "inspection/ImageExtractor.h"

namespace inspection {

ImageExtractor::ImageExtractor(const std::string &config_file_location) {
  nlohmann::json J;
  std::ifstream file(config_file_location);
  file >> J;

  bag_file_ = J["bage_file"];
  poses_file_ = J["poses_file"];
  save_directory_ = J["save_directory"];

  for (const auto &topic : J["image_topics"]) {
    image_topics_.push_back(topic.get<std::string>());
  }

  for (const auto &dist : J["distance_between_images"]) {
    distance_between_images_.push_back(dist.get<double>());
  }

  for (const auto &rot : J["rotation_between_images"]) {
    double DEG_TO_RAD = 3.14159265359 / 180;
    rotation_between_images_.push_back(rot.get<double>() * DEG_TO_RAD);
  }

  for (const auto &distorted : J["are_images_distorted"]) {
    are_images_distorted_.push_back(distorted.get<bool>());
  }

  if (image_topics_.size() != distance_between_images_.size()) {
    LOG_ERROR("Number of image topics not equal to number of distances between "
              "poses. Topics = %d, Distances = %d",
              (int)image_topics_.size(), (int)distance_between_images_.size());
    throw std::invalid_argument{"Number of image topics not equal to number of "
                                "distances between images"};
  }

  if (image_topics_.size() != are_images_distorted_.size()) {
    LOG_ERROR("Number of image topics not equal to number of booleans "
              "specifying if images are distorted. Topics = %d, Distorted = %d",
              (int)image_topics_.size(), (int)are_images_distorted_.size());
    throw std::invalid_argument{"Number of image topics not equal to number of "
                                "distances between images"};
  }

  image_container_type_ = J["image_container_type"];
}

void ImageExtractor::ExtractImages() {
  GetTimeStamps();
  OutputImages();
}

void ImageExtractor::GetTimeStamps() {
  poses_ = beam_containers::ReadPoseFile(poses_file_);

  Eigen::Affine3d TA_last, TA_check;
  time_stamps_.clear();

  // iterate over all k cameras
  for (uint8_t k = 0; k < image_topics_.size(); k++) {
    std::vector<beam::TimePoint> time_stamps_k;
    TA_last = poses_[0].second;
    time_stamps_k.push_back(poses_[0].first);
    std::pair<double, double> pose_change_ki;
    int time_stamp_count = 0;

    // iterate over all i poses for camera k
    for (uint16_t i = 1; i < poses_.size(); i++) {
      TA_check = poses_[i].second;
      pose_change_ki = CalculatePoseChange(TA_last, TA_check);
      if (pose_change_ki.first >= distance_between_images_[k] ||
          pose_change_ki.second >= rotation_between_images_[k]) {
        time_stamp_count++;
        time_stamps_k.push_back(poses_[i].first);
        TA_last = TA_check;
      }
    }
    time_stamps_.push_back(time_stamps_k);
  }
}

std::pair<double, double>
ImageExtractor::CalculatePoseChange(const Eigen::Affine3d &p1,
                                    const Eigen::Affine3d &p2) {
  double translation = sqrt((p1.matrix()(0, 3) - p2.matrix()(0, 3)) *
                                (p1.matrix()(0, 3) - p2.matrix()(0, 3)) +
                            (p1.matrix()(1, 3) - p2.matrix()(1, 3)) *
                                (p1.matrix()(1, 3) - p2.matrix()(1, 3)) +
                            (p1.matrix()(2, 3) - p2.matrix()(2, 3)) *
                                (p1.matrix()(2, 3) - p2.matrix()(2, 3)));

  beam::Vec3 eps1, eps2, diffSq;
  eps1 = beam::RToLieAlgebra(p1.rotation());
  eps2 = beam::RToLieAlgebra(p2.rotation());
  diffSq(0, 0) = (eps2(0, 0) - eps1(0, 0)) * (eps2(0, 0) - eps1(0, 0));
  diffSq(1, 0) = (eps2(1, 0) - eps1(1, 0)) * (eps2(1, 0) - eps1(1, 0));
  diffSq(2, 0) = (eps2(2, 0) - eps1(2, 0)) * (eps2(2, 0) - eps1(2, 0));
  double rotation = sqrt(diffSq.maxCoeff());

  return std::make_pair(translation, rotation);
}

void ImageExtractor::OutputImages() {
  boost::filesystem::create_directories(save_directory_);
  std::string camera_dir, image_container_dir;
  beam::TimePoint image_time_point;
  rosbag::Bag bag;
  cv::Mat image_ki;

  /**
   * @todo add_image_container_abstract
   * @body update this once we have created an abstract class for the image
   * containers
   */
  // beam_containers::ImageBridge image_container();
  if (image_container_type_ == "ImageBridge") {
    // initialize abstract class with ImageBridge derived class when it has been
    // implemented in libbeam. See issue #33 in libbeam and issue #16
    // in beam_robotics
  } else {
    LOG_ERROR(
        "Invalid image container type. Options are: ImageBridge. Check that "
        "your image_container_type parameter in ImageExtractorConfig.json and "
        "check that the image container header is included in "
        "ImageExtractor.h");
    throw std::invalid_argument{
        "Invalid image container type. Options are: ImageBridge. Check that "
        "your image_container_type parameter in ImageExtractorConfig.json and "
        "check that the image container header is included in "
        "ImageExtractor.h"};
  }

  try {
    bag.open(bag_file_, rosbag::bagmode::Read);
  } catch (rosbag::BagException &ex) {
    LOG_ERROR("Bag exception : %s", ex.what());
    throw std::invalid_argument{ex.what()};
    return;
  }

  // iterate over all k cameras
  for (uint8_t k = 0; k < image_topics_.size(); k++) {
    camera_dir = save_directory_ + "/camera" + std::to_string(k);
    camera_list_.push_back("camera" + std::to_string(k));
    image_object_list_.clear();
    boost::filesystem::create_directories(camera_dir);
    int img_counter = 0;

    // iterate over all i poses for camera k
    for (uint32_t i = 1; i < time_stamps_[k].size(); i++) {
      img_counter++;
      beam_containers::ImageBridge image_ki_container;
      image_container_dir =
          camera_dir + "/" + image_container_type_ + std::to_string(i);
      boost::filesystem::create_directories(image_container_dir);
      image_time_point = time_stamps_[k][i];
      image_ki = GetImageFromBag(image_time_point, bag, image_topics_[k]);
      image_ki_container.SetBGRImage(image_ki);
      image_ki_container.SetBGRIsDistorted(are_images_distorted_[k]);
      image_ki_container.SetTimePoint(image_time_point);
      image_ki_container.SetImageSeq(img_counter);
      image_ki_container.SetBagName(bag_file_);
      image_ki_container.Write(image_container_dir);
      image_object_list_.push_back(image_container_type_ +
                                   std::to_string(img_counter));
    }
    OutputJSONList(camera_dir + "/ImagesList.json", image_object_list_);
  }
  OutputJSONList(save_directory_ + "/CamerasList.json", camera_list_);
}

cv::Mat ImageExtractor::GetImageFromBag(const beam::TimePoint &time_point,
                                        rosbag::Bag &ros_bag,
                                        std::string &image_topic) {
  ros::Time search_time_start, search_time_end;
  double time_window = 10;
  ros::Duration time_window_half(time_window / 2);
  search_time_start = beam::chronoToRosTime(time_point) - time_window_half;
  search_time_end = beam::chronoToRosTime(time_point) + time_window_half;
  rosbag::View view(ros_bag, rosbag::TopicQuery(image_topic), search_time_start,
                    search_time_end, true);

  if (view.size() == 0) {
    LOG_ERROR("No image messages read. Check your topics in config file.");
    throw std::invalid_argument{
        "No image messages read. Check your topics in config file."};
    cv::Mat image;
    return image;
  }

  // iterate through bag:
  for (auto iter = view.begin(); iter != view.end(); iter++) {
    if (iter->getTopic() == image_topic) {
      auto img_msg = iter->instantiate<sensor_msgs::Image>();
      beam::TimePoint curImgTimepoint = beam::rosTimeToChrono(img_msg->header);
      if (curImgTimepoint >= time_point) {
        cv_bridge::CvImagePtr cv_img_ptr;
        cv_img_ptr =
            cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        return cv_img_ptr->image;
      }
    }
  }
}

void ImageExtractor::OutputJSONList(const std::string &file_name,
                                    const std::vector<std::string> &list) {
  std::string JSONString = "{ \"Items\": [";
  for (uint32_t i = 0; i<list.size()-1; i++){
    JSONString = JSONString + "\"" + list[i] + "\", ";
  }
  JSONString = JSONString + "\"" + list[list.size()-1] + "\"]}";
  auto J = nlohmann::json::parse(JSONString);
  std::ofstream file(file_name);
  file << std::setw(4) << J << std::endl;
}

} // end namespace inspection
