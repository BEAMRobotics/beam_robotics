#include <inspection/ImageExtractor.h>

#include <boost/filesystem.hpp>
#include <rosbag/view.h>
#include <sensor_msgs/image_encodings.h>

#include <beam_containers/ImageBridge.h>
#include <beam_mapping/Poses.h>
#include <beam_cv/OpenCVConversions.h>
#include <beam_utils/angles.h>
#include <beam_utils/log.h>
#include <beam_utils/math.h>
#include <beam_utils/time.h>

namespace inspection {

typedef Eigen::aligned_allocator<Eigen::Affine3d> AffineAlign;

ImageExtractor::ImageExtractor(const std::string& bag_file,
                               const std::string& poses_file,
                               const std::string& save_directory,
                               const std::string& config_file_location) {
  bag_file_ = bag_file;
  poses_file_ = poses_file;
  save_directory_ = save_directory;

  BEAM_INFO("Loading config from: {}", config_file_location);
  nlohmann::json J;
  std::ifstream file(config_file_location);
  file >> J;

  image_container_type_ = J["image_container_type"];
  std::string intrinsics_directory = J["intrinsics_directory"];

  // initialize undistort vector
  for (const auto& camera_params : J["camera_params"]) {
    undistort_images_.push_back(nullptr);
  }

  for (const auto& camera_params : J["camera_params"]) {
    nlohmann::json topic = camera_params["image_topic"];
    image_topics_.push_back(topic.get<std::string>());

    nlohmann::json intrinsics = camera_params["intrinsics_name"];
    intrinsics_.push_back(intrinsics_directory + intrinsics.get<std::string>());

    nlohmann::json distance = camera_params["distance_between_images_m"];
    distance_between_images_.push_back(distance.get<double>());

    nlohmann::json rotation = camera_params["rotation_between_images_deg"];
    rotation_between_images_.push_back(beam::Deg2Rad(rotation.get<double>()));

    nlohmann::json distorted = camera_params["are_images_distorted"];
    are_input_images_distorted_.push_back(distorted.get<bool>());

    nlohmann::json compressed = camera_params["are_images_compressed"];
    are_images_compressed_.push_back(compressed.get<bool>());

    nlohmann::json ir = camera_params["is_ir_camera"];
    is_ir_camera_.push_back(ir.get<bool>());

    std::vector<ImageTransform> image_transforms =
        GetImageTransforms(camera_params["image_transforms"]);
    image_transforms_.push_back(image_transforms);
  }
  are_output_images_distorted_ = are_input_images_distorted_;
}

std::vector<ImageTransform>
    ImageExtractor::GetImageTransforms(const nlohmann::json& J) {
  std::vector<ImageTransform> image_transforms;
  for (const auto& transform : J) {
    std::string type = transform["type"].get<std::string>();
    std::vector<double> params;
    if (type == "LINEAR") {
      params.push_back(transform["alpha"].get<double>());
      params.push_back(transform["beta"].get<double>());
    } else if (type == "UNDISTORT") {
      params.push_back(transform["crop_height"].get<double>());
      params.push_back(transform["crop_width"].get<double>());
    } else if (type == "HISTOGRAM") {
      // null
    } else if (type == "CLAHE") {
      // null
    } else {
      BEAM_ERROR("Invalid image_transform type in config file. Options: "
                 "LINEAR, HISTOGRAM, UNDISTORT");
      throw std::invalid_argument{"Invalid image_transform type"};
    }
    image_transforms.push_back(ImageTransform{.type = type, .params = params});
  }
  return image_transforms;
}

void ImageExtractor::ExtractImages() {
  // open bag
  try {
    bag_.open(bag_file_, rosbag::bagmode::Read);
  } catch (rosbag::BagException& ex) {
    BEAM_ERROR("Bag exception : %s", ex.what());
    throw std::invalid_argument{ex.what()};
    return;
  }

  GetTimeStamps();
  OutputImages();
}

void ImageExtractor::GetTimeStamps() {
  // load all poses
  beam_mapping::Poses p;
  p.LoadFromJSON(poses_file_);
  std::vector<Eigen::Affine3d, AffineAlign> poses = p.GetPoses();
  std::vector<ros::Time> pose_time_stamps = p.GetTimeStamps();
  image_time_stamps_ =
      std::vector<std::vector<ros::Time>>(image_topics_.size());

  // iterate over all cameras
  for (uint8_t cam_iter = 0; cam_iter < image_topics_.size(); cam_iter++) {
    // get start and end of image topic
    rosbag::View view(bag_, rosbag::TopicQuery(image_topics_[cam_iter]),
                      ros::TIME_MIN, ros::TIME_MAX, true);
    if (view.size() == 0) {
      BEAM_ERROR("No image messages read for image topic {}. Check your topics "
                 "in config file.",
                 image_topics_[cam_iter]);
      continue;
    }

    // get start time
    rosbag::View::iterator iter_start = view.begin();
    sensor_msgs::ImageConstPtr img_msg_begin =
        iter_start->instantiate<sensor_msgs::Image>();
    ros::Time topic_start_time = img_msg_begin->header.stamp;

    // iterate through bag and get last message time
    rosbag::View::iterator last_iter;
    for (auto iter = view.begin(); iter != view.end(); iter++) {
      last_iter = iter;
    }
    sensor_msgs::ImageConstPtr img_msg_end =
        last_iter->instantiate<sensor_msgs::Image>();
    ros::Time topic_end_time = img_msg_end->header.stamp;

    // iterate over all poses for this camera
    std::vector<ros::Time> time_stamps;
    Eigen::Affine3d TA_moving_fixed_last = poses[0];
    for (uint16_t pose_iter = 1; pose_iter < poses.size(); pose_iter++) {
      // first check that pose time is not outside current bag time
      if (pose_time_stamps[pose_iter] < topic_start_time ||
          pose_time_stamps[pose_iter] > topic_end_time) {
        continue;
      }
      // next, check sufficient motion has passed
      Eigen::Affine3d TA_moving_fixed_curr = poses[pose_iter];
      Eigen::Affine3d TA_curr_last =
          TA_moving_fixed_curr.inverse() * TA_moving_fixed_last;
      if (PassedMinMotion(TA_curr_last, cam_iter)) {
        time_stamps.push_back(pose_time_stamps[pose_iter]);
        TA_moving_fixed_last = TA_moving_fixed_curr;
      }
    }
    image_time_stamps_[cam_iter] = time_stamps;
    BEAM_INFO("Saving {} images from camera topic: {}", time_stamps.size(),
              image_topics_[cam_iter]);
  }
}

bool ImageExtractor::PassedMinMotion(const Eigen::Affine3d& TA_curr_last,
                                     int cam_number) {
  Eigen::Vector3d error_t = TA_curr_last.translation();
  error_t[0] = std::abs(error_t[0]);
  error_t[1] = std::abs(error_t[1]);
  error_t[2] = std::abs(error_t[2]);
  if (error_t[0] > distance_between_images_[cam_number] ||
      error_t[1] > distance_between_images_[cam_number] ||
      error_t[2] > distance_between_images_[cam_number]) {
    return true;
  }

  double error_r = Eigen::AngleAxis<double>(TA_curr_last.rotation()).angle();
  if (error_r > rotation_between_images_[cam_number]) { return true; }

  return false;
}

void ImageExtractor::OutputImages() {
  /**
   * @todo add_image_container_abstract
   * @body update this once we have created an abstract class for the image
   * containers
   */
  boost::filesystem::create_directories(save_directory_);

  // iterate over all cameras
  for (uint8_t cam_count = 0; cam_count < image_topics_.size(); cam_count++) {
    BEAM_INFO("Saving images for Camera {}.", cam_count);
    std::string camera_dir =
        save_directory_ + "/camera" + std::to_string(cam_count);
    boost::filesystem::create_directories(camera_dir);
    camera_list_.push_back("camera" + std::to_string(cam_count));
    image_object_list_.clear();

    // iterate over all poses for this camera
    for (uint32_t image_count = 0;
         image_count < image_time_stamps_[cam_count].size(); image_count++) {
      ros::Time image_time = image_time_stamps_[cam_count][image_count];
      cv::Mat image =
          GetImageFromBag(image_time, cam_count, (image_count == 0));

      if (image_container_type_ == "ImageBridge") {
        beam_containers::ImageBridge image_container;
        std::string image_container_dir = camera_dir + "/" +
                                          image_container_type_ +
                                          std::to_string(image_count);
        boost::filesystem::create_directories(image_container_dir);
        if (is_ir_camera_[cam_count]) {
          image_container.SetIRImage(image);
          image_container.SetIRIsDistorted(
              are_output_images_distorted_[cam_count]);
          image_container.SetIRFrameId(frame_ids_[cam_count]);
        } else {
          image_container.SetBGRImage(image);
          image_container.SetBGRIsDistorted(
              are_output_images_distorted_[cam_count]);
          image_container.SetBGRFrameId(frame_ids_[cam_count]);
        }

        beam::TimePoint image_timepoint = beam::RosTimeToChrono(image_time);
        image_container.SetTimePoint(image_timepoint);
        image_container.SetImageSeq(image_count);
        image_container.SetBagName(bag_file_);
        image_container.Write(image_container_dir);
        image_object_list_.push_back(image_container_type_ +
                                     std::to_string(image_count));
      } else if (image_container_type_ == "None") {
        std::string output_file =
            camera_dir + "/Image" + std::to_string(image_count) + ".jpg";
        cv::imwrite(output_file, image);
        image_object_list_.push_back(output_file);
      } else {
        BEAM_ERROR("Invalid image container type. Options are: ImageBridge. "
                   "Check that your image_container_type parameter in "
                   "ImageExtractorConfig.json and check that the image "
                   "container header is included in ImageExtractor.h");
        throw std::invalid_argument{
            "Invalid image container type. Options are: ImageBridge. Check "
            "that your image_container_type parameter in "
            "ImageExtractorConfig.json and check that the image container "
            "header is included in ImageExtractor.h"};
      }
    }

    if (image_object_list_.size() > 0) {
      OutputJSONList(camera_dir + "/ImagesList.json", image_object_list_);
    }
  }

  if (camera_list_.size() > 0) {
    OutputJSONList(save_directory_ + "/CamerasList.json", camera_list_);
  }
}

cv::Mat ImageExtractor::GetImageFromBag(ros::Time& image_time, int cam_number,
                                        bool first_image) {
  if (first_image) { last_image_time_ = ros::TIME_MIN; }

  rosbag::View view(bag_, rosbag::TopicQuery(image_topics_[cam_number]),
                    last_image_time_, ros::TIME_MAX, true);
  if (view.size() == 0) {
    BEAM_ERROR("No image messages read. Check your topics in config file.");
    throw std::invalid_argument{
        "No image messages read. Check your topics in config file."};
  }

  // iterate through bag:
  for (rosbag::View::iterator iter = view.begin(); iter != view.end(); iter++) {
    sensor_msgs::ImageConstPtr img_msg =
        iter->instantiate<sensor_msgs::Image>();

    // check that topic type is correct:
    if (img_msg == NULL) {
      BEAM_ERROR("Unable to instantiate image message. You are probably "
                 "feeding a topic with the incorrect message type. Message "
                 "type must be sensor_msgs::Image. Topic: {}",
                 image_topics_[cam_number]);
      throw std::runtime_error{"Unable to instantiate ROS message."};
    }

    if (img_msg->header.stamp < image_time) { continue; }
    image_time = img_msg->header.stamp;
    last_image_time_ = img_msg->header.stamp;
    if (first_image) { frame_ids_.push_back(img_msg->header.frame_id); }
    if (img_msg->encoding == sensor_msgs::image_encodings::BGR8) {
      cv::Mat image = beam_cv::OpenCVConversions::RosImgToMat(*img_msg);
      return ApplyImageTransforms(image, cam_number);
    } else if (img_msg->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::Mat image_RGB = beam_cv::OpenCVConversions::RosImgToMat(*img_msg);
      cv::Mat image_BGR;
      cv::cvtColor(image_RGB, image_BGR, CV_RGB2BGR);
      return ApplyImageTransforms(image_BGR, cam_number);
    } else {
      cv::Mat image = beam_cv::OpenCVConversions::RosImgToMat(*img_msg);
      return ApplyImageTransforms(image, cam_number);
    }
  }
}

cv::Mat ImageExtractor::ApplyImageTransforms(const cv::Mat& image,
                                             int cam_number) {
  std::vector<ImageTransform> transforms = image_transforms_[cam_number];

  // avoid useless copying if no transforms are specified
  if (transforms.size() == 0) { return image; }

  cv::Mat output_image = image.clone();
  for (ImageTransform transform : transforms) {
    if (transform.type == "LINEAR") {
      output_image = ApplyLinearTransform(output_image, transform.params);
    } else if (transform.type == "HISTOGRAM") {
      output_image = ApplyHistogramTransform(output_image, transform.params);
    } else if (transform.type == "CLAHE") {
      output_image = ApplyClaheTransform(output_image, transform.params);
    } else if (transform.type == "UNDISTORT") {
      output_image =
          ApplyUndistortTransform(output_image, transform.params, cam_number);
    } else {
      BEAM_WARN("Image transform type ({}) not yet implemented. Skipping",
                transform.type);
    }
  }
  return output_image;
}

cv::Mat
    ImageExtractor::ApplyLinearTransform(const cv::Mat& image,
                                         const std::vector<double>& params) {
  cv::Mat new_image = cv::Mat::zeros(image.size(), image.type());
  double alpha = params[0];
  double beta = params[1];
  for (int y = 0; y < image.rows; y++) {
    for (int x = 0; x < image.cols; x++) {
      for (int c = 0; c < image.channels(); c++) {
        new_image.at<cv::Vec3b>(y, x)[c] = cv::saturate_cast<uchar>(
            alpha * image.at<cv::Vec3b>(y, x)[c] + beta);
      }
    }
  }
  return new_image;
}

cv::Mat
    ImageExtractor::ApplyHistogramTransform(const cv::Mat& image,
                                            const std::vector<double>& params) {
  cv::Mat ycrcb;
  cv::cvtColor(image, ycrcb, CV_BGR2YCrCb);
  std::vector<cv::Mat> channels;
  cv::split(ycrcb, channels);
  cv::equalizeHist(channels[0], channels[0]);
  cv::Mat new_image;
  cv::merge(channels, ycrcb);
  cv::cvtColor(ycrcb, new_image, CV_YCrCb2BGR);
  return new_image;
}

cv::Mat ImageExtractor::ApplyUndistortTransform(
    const cv::Mat& image, const std::vector<double>& params, int cam_number) {
  // create undistortion object if not created already
  if (undistort_images_[cam_number] == nullptr) {
    are_output_images_distorted_[cam_number] = false;
    std::shared_ptr<beam_calibration::CameraModel> camera_model =
        beam_calibration::CameraModel::Create(intrinsics_[cam_number]);
    Eigen::Vector2i src_image_dims;
    Eigen::Vector2i dst_image_dims;
    if (are_images_compressed_[cam_number]) {
      src_image_dims[0] = camera_model->GetHeight();
      src_image_dims[1] = camera_model->GetWidth();
      dst_image_dims[0] =
          static_cast<int>(camera_model->GetHeight() * params[0]);
      dst_image_dims[1] =
          static_cast<int>(camera_model->GetWidth() * params[1]);
    } else {
      src_image_dims[0] = image.rows;
      src_image_dims[1] = image.cols;
      dst_image_dims[0] = static_cast<int>(image.rows * params[0]);
      dst_image_dims[1] = static_cast<int>(image.cols * params[1]);
    }

    beam_calibration::UndistortImages undistort(camera_model, src_image_dims,
                                                dst_image_dims);
    undistort_images_[cam_number] =
        std::make_shared<beam_calibration::UndistortImages>(undistort);
  }

  if (are_images_compressed_[cam_number]) {
    cv::Mat image_upsampled =
        undistort_images_[cam_number]->UpsampleImage(image);
    cv::Mat image_undistorted =
        undistort_images_[cam_number]->ConvertImage<cv::Vec3b>(image_upsampled);
    return undistort_images_[cam_number]->DownsampleImage(
        image_undistorted, Eigen::Vector2i(image.rows, image.cols));
  }

  return undistort_images_[cam_number]->ConvertImage<cv::Vec3b>(image);
}

cv::Mat ImageExtractor::ApplyClaheTransform(const cv::Mat& image,
                                            const std::vector<double>& params) {
  cv::Mat lab_image;
  cv::cvtColor(image, lab_image, CV_BGR2Lab);
  std::vector<cv::Mat> lab_planes(6);
  cv::split(lab_image, lab_planes);
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(3);
  cv::Mat dst;
  clahe->apply(lab_planes[0], dst);
  dst.copyTo(lab_planes[0]);
  cv::merge(lab_planes, lab_image);
  cv::Mat new_image;
  cv::cvtColor(lab_image, new_image, CV_Lab2BGR);
  return new_image;
}

cv::Mat ImageExtractor::ROSConvertColor(
    const sensor_msgs::ImageConstPtr& image_raw) {
  cv::Mat image_color;
  int code = 0, raw_type = CV_8UC1;
  std::string raw_encoding = image_raw->encoding;
  if (raw_encoding == sensor_msgs::image_encodings::BAYER_RGGB8) {
    code = cv::COLOR_BayerBG2BGR;
  } else if (raw_encoding == sensor_msgs::image_encodings::BAYER_BGGR8) {
    code = cv::COLOR_BayerRG2BGR;
  } else if (raw_encoding == sensor_msgs::image_encodings::BAYER_GBRG8) {
    code = cv::COLOR_BayerGR2BGR;
  } else if (raw_encoding == sensor_msgs::image_encodings::BAYER_GRBG8) {
    code = cv::COLOR_BayerGR2BGR;
  } else if (raw_encoding == sensor_msgs::image_encodings::MONO8) {
    code = cv::COLOR_BayerGB2BGR;
  } else {
    BEAM_ERROR("ROS msg encoding ({}) not supported.", raw_encoding);
    throw std::runtime_error{"ROS msg encoding not supported."};
    return image_color;
  }
  cv::Mat raw(image_raw->height, image_raw->width, raw_type,
              const_cast<uint8_t*>(&image_raw->data[0]), image_raw->step);
  cv::cvtColor(raw, image_color, code);
  return image_color;
}

void ImageExtractor::OutputJSONList(const std::string& file_name,
                                    const std::vector<std::string>& list) {
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
