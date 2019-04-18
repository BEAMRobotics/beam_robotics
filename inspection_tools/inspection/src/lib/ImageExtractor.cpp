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

  if (image_topics_.size() != distance_between_images_.size()) {
    LOG_ERROR("Number of image topics not equal to number of distances between "
              "poses. Topics = %d, Distances = %d",
              (int)image_topics_.size(), (int)distance_between_images_.size());
    throw std::invalid_argument{"Number of image topics not equal to number of "
                                "distances between images"};
  }
  GetTimeStamps();
  OutputImages();
}

void ImageExtractor::ExtractImages() {

  beam::TimePoint image_time_point;
  rosbag::Bag bag;

  try {
    bag.open(bag_file_, rosbag::bagmode::Read);
  } catch (rosbag::BagException &ex) {
    LOG_ERROR("Bag exception : %s", ex.what());
    return;
  }
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

    // iterate over all i poses for camera k
    for (uint16_t i = 1; i < poses_.size(); i++) {
      TA_check = poses_[i].second;
      pose_change_ki = CalculatePoseChange(TA_last, TA_check);
      if (pose_change_ki.first >= distance_between_images_[k] ||
          pose_change_ki.second >= rotation_between_images_[k]) {
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
  //
}

} // end namespace inspection
