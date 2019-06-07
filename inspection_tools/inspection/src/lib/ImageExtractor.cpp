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

  for (const auto &ir : J["is_ir_camera"]) {
    is_ir_camera_.push_back(ir.get<bool>());
  }

  for (const auto &enhancing : J["image_enhancing"]) {
    std::string enhance_method = enhancing["enhance_method"].get<std::string>();
    double alpha = enhancing["alpha"].get<double>();
    double beta = enhancing["beta"].get<double>();
    enhance_methods_.push_back(enhance_method);
    alphas_.push_back(alpha);
    betas_.push_back(beta);
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
                                "are_images_distorted"};
  }

  if (image_topics_.size() != is_ir_camera_.size()) {
    LOG_ERROR(
        "Number of image topics not equal to number of booleans "
        "specifying if camera is an infrared camera. Topics = %d, Is IR = %d",
        (int)image_topics_.size(), (int)is_ir_camera_.size());
    throw std::invalid_argument{"Number of image topics not equal to number of "
                                "is_ir_camera booleans"};
  }

  if (image_topics_.size() != enhance_methods_.size()) {
    LOG_ERROR("Number of image topics not equal to number of image enhancing "
              "objects. Topics = %d, Enhancing = %d",
              (int)image_topics_.size(), (int)enhance_methods_.size());
    throw std::invalid_argument{"Number of image topics not equal to number of "
                                "image enhancing objects."};
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
      if (img_counter == 1) {
        image_ki = GetImageFromBag(image_time_point, bag, k, true);
      } else {
        image_ki = GetImageFromBag(image_time_point, bag, k, false);
      }
      if (is_ir_camera_[k]) {
        image_ki_container.SetIRImage(image_ki);
        image_ki_container.SetIRIsDistorted(are_images_distorted_[k]);
        image_ki_container.SetIRFrameId(frame_ids_[k]);
      } else {
        image_ki_container.SetBGRImage(image_ki);
        image_ki_container.SetBGRIsDistorted(are_images_distorted_[k]);
        image_ki_container.SetBGRFrameId(frame_ids_[k]);
      }
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
                                        const int &cam_number,
                                        bool add_frame_id) {
  ros::Time search_time_start, search_time_end;
  double time_window = 10;
  std::string image_topic = image_topics_[cam_number];
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
      sensor_msgs::ImageConstPtr img_msg =
          iter->instantiate<sensor_msgs::Image>();
      beam::TimePoint curImgTimepoint = beam::rosTimeToChrono(img_msg->header);
      if (curImgTimepoint >= time_point) {
        if (add_frame_id) { 
          frame_ids_.push_back(img_msg->header.frame_id);
        }
        if (img_msg->encoding != sensor_msgs::image_encodings::BGR8) {
          cv::Mat image_debayered = ROSDebayer(img_msg);
          return EnhanceImage(image_debayered, cam_number);
        } else {
          cv::Mat image =
              cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)
                  ->image;
          return EnhanceImage(image, cam_number);
        }
      }
    }
  }
}

cv::Mat ImageExtractor::EnhanceImage(cv::Mat &input_image, int cam_number) {
  if (enhance_methods_[cam_number] == "none") {
    return input_image;
  } else if (enhance_methods_[cam_number] == "linear") {
    cv::Mat new_image = cv::Mat::zeros(input_image.size(), input_image.type());
    double alpha = alphas_[cam_number];
    double beta = betas_[cam_number];
    for (int y = 0; y < input_image.rows; y++) {
      for (int x = 0; x < input_image.cols; x++) {
        for (int c = 0; c < input_image.channels(); c++) {
          new_image.at<cv::Vec3b>(y, x)[c] = cv::saturate_cast<uchar>(
              alpha * input_image.at<cv::Vec3b>(y, x)[c] + beta);
        }
      }
    }
    return new_image;
  } else if (enhance_methods_[cam_number] == "histogram") {
    cv::Mat ycrcb;
    cv::cvtColor(input_image, ycrcb, CV_BGR2YCrCb);
    std::vector<cv::Mat> channels;
    cv::split(ycrcb, channels);
    cv::equalizeHist(channels[0], channels[0]);
    cv::Mat new_image;
    cv::merge(channels, ycrcb);
    cv::cvtColor(ycrcb, new_image, CV_YCrCb2BGR);
    return new_image;
  } else if (enhance_methods_[cam_number] == "clahe") {
    cv::Mat lab_image;
    cv::cvtColor(input_image, lab_image, CV_BGR2Lab);
    std::vector<cv::Mat> lab_planes(6);
    cv::split(lab_image, lab_planes);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    // Explanation of ClipLimit
    // here:https://en.wikipedia.org/wiki/Adaptive_histogram_equalization#Contrast_Limited_AHE
    clahe->setClipLimit(3);
    cv::Mat dst;
    clahe->apply(lab_planes[0], dst);
    dst.copyTo(lab_planes[0]);
    cv::merge(lab_planes, lab_image);
    cv::Mat new_image;
    cv::cvtColor(lab_image, new_image, CV_Lab2BGR);
    return new_image;
  } else {
    LOG_ERROR("invalid enhance_method parameter in ImageExtractorConfig.json"
              "Options include: none, linear, and histogram");
    throw std::invalid_argument{
        "invalid enhance_method parameter in ImageExtractorConfig.json"};
    return input_image;
  }
}

cv::Mat ImageExtractor::ROSDebayer(sensor_msgs::ImageConstPtr &image_raw) {
  cv::Mat image_color;
  int code = 0, raw_type = CV_8UC1;
  std::string raw_encoding = image_raw->encoding;
  if (raw_encoding == sensor_msgs::image_encodings::BAYER_RGGB8) {
    code = cv::COLOR_BayerBG2BGR;
  } else {
    LOG_ERROR("ROS msg encoding not supported.");
    throw std::runtime_error{"ROS msg encoding not supported."};
    return image_color;
  }
  cv::Mat raw(image_raw->height, image_raw->width, raw_type,
              const_cast<uint8_t *>(&image_raw->data[0]), image_raw->step);
  cv::cvtColor(raw, image_color, code);
  return image_color;
}

void ImageExtractor::OutputJSONList(const std::string &file_name,
                                    const std::vector<std::string> &list) {
  std::string JSONString = "{ \"Items\": [";
  for (uint32_t i = 0; i < list.size() - 1; i++) {
    JSONString = JSONString + "\"" + list[i] + "\", ";
  }
  JSONString = JSONString + "\"" + list[list.size() - 1] + "\"]}";
  auto J = nlohmann::json::parse(JSONString);
  std::ofstream file(file_name);
  file << std::setw(4) << J << std::endl;
}

} // end namespace inspection
