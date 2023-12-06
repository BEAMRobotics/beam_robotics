#include <inspection/ImageExtractor.h>

#include <boost/filesystem.hpp>
#include <rosbag/view.h>
#include <sensor_msgs/image_encodings.h>

#include <beam_containers/ImageBridge.h>
#include <beam_cv/OpenCVConversions.h>
#include <beam_mapping/Poses.h>
#include <beam_utils/angles.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/log.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>
#include <beam_utils/time.h>

#include <inspection/ImageDatabase.h>
#include <inspection/Utils.h>

namespace inspection {

ImageExtractor::ImageExtractor(const std::string& bag_file,
                               const std::string& poses_file,
                               const std::string& save_directory,
                               const std::string& config_file_location) {
  bag_file_ = bag_file;
  poses_file_ = poses_file;
  save_directory_ = save_directory;

  BEAM_INFO("Loading config from: {}", config_file_location);
  nlohmann::json J;
  if (!beam::ReadJson(config_file_location, J)) {
    throw std::runtime_error{"cannot read json"};
  }

  beam::ValidateJsonKeysOrThrow(
      {"image_container_type", "intrinsics_directory", "camera_params"}, J);

  image_container_type_ = J["image_container_type"];
  std::string intrinsics_directory = J["intrinsics_directory"];

  for (const auto& camera_params : J["camera_params"]) {
    CameraData camera_data;
    beam::ValidateJsonKeysOrThrow(
        {"image_topic", "intrinsics_name", "distance_between_images_m",
         "rotation_between_images_deg", "are_images_distorted", "is_ir_camera",
         "image_transforms"},
        camera_params);
    camera_data.topic = camera_params["image_topic"];
    camera_data.distance_between_images_m =
        camera_params["distance_between_images_m"];
    camera_data.rotation_between_images_rad =
        beam::Deg2Rad(camera_params["rotation_between_images_deg"]);
    camera_data.input_distorted = camera_params["are_images_distorted"];
    camera_data.output_distorted = camera_params["are_images_distorted"];
    camera_data.ir_camera = camera_params["is_ir_camera"];
    camera_data.transforms =
        GetImageTransforms(camera_params["image_transforms"]);
    std::string intrinsics_path = beam::CombinePaths(
        intrinsics_directory, camera_params["intrinsics_name"]);
    camera_data.model = beam_calibration::CameraModel::Create(intrinsics_path);
    camera_data_.push_back(camera_data);
  }
}

std::vector<ImageTransform>
    ImageExtractor::GetImageTransforms(const nlohmann::json& J) {
  std::vector<ImageTransform> image_transforms;
  for (const auto& transform : J) {
    std::string type = transform["type"];
    std::vector<double> params;
    if (type == "LINEAR") {
      params.push_back(transform["alpha"]);
      params.push_back(transform["beta"]);
    } else if (type == "UNDISTORT") {
      params.push_back(transform["crop_height"]);
      params.push_back(transform["crop_width"]);
    } else if (type == "HISTOGRAM") {
      BEAM_ERROR("HISTOGRAM FILTER NOT YET IMPLEMENTED! Skipping");
    } else if (type == "CLAHE") {
      BEAM_ERROR("CLAHE FILTER NOT YET IMPLEMENTED! Skipping");
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
  try {
    bag_.open(bag_file_, rosbag::bagmode::Read);
  } catch (rosbag::BagException& ex) {
    BEAM_ERROR("Bag exception : %s", ex.what());
    throw std::invalid_argument{ex.what()};
  }

  GetImageProperties();
  GetTimeStamps();
  OutputImages();
}

void ImageExtractor::GetImageProperties() {
  for (auto& camera_data : camera_data_) {
    rosbag::View view(bag_, rosbag::TopicQuery(camera_data.topic),
                      ros::TIME_MIN, ros::TIME_MAX, true);
    if (view.size() == 0) {
      BEAM_ERROR("No image messages read for image topic {}. Check your topics "
                 "in config file.",
                 camera_data.topic);
      throw std::runtime_error{"invalid topic"};
    }

    rosbag::View::iterator iter = view.begin();
    auto img_msg = iter->instantiate<sensor_msgs::Image>();
    if (img_msg) {
      camera_data.compressed = false;
      camera_data.frame_id = img_msg->header.frame_id;
      if (img_msg->height != camera_data.model->GetHeight() ||
          img_msg->width != camera_data.model->GetWidth()) {
        camera_data.downsampled = true;
      }
    } else {
      auto comp_img_msg = iter->instantiate<sensor_msgs::CompressedImage>();
      if (!comp_img_msg) {
        BEAM_ERROR("Image data is not of type sensor_msgs::Image or "
                   "sensor_msgs::CompressedImage");
        throw std::runtime_error{"invalid image type"};
      }
      sensor_msgs::Image img_msg_decomp;
      if (!rosbag_tools::utils::DecompressJpegImage(*comp_img_msg,
                                                    img_msg_decomp)) {
        BEAM_ERROR("Unable to decompress image.");
        throw std::runtime_error{"unable to decompress image"};
      }
      camera_data.compressed = true;
      camera_data.frame_id = img_msg_decomp.header.frame_id;
      if (img_msg_decomp.height != camera_data.model->GetHeight() ||
          img_msg_decomp.width != camera_data.model->GetWidth()) {
        camera_data.downsampled = true;
      }
    }
  }
}

void ImageExtractor::GetTimeStamps() {
  // load all poses
  beam_mapping::Poses p;
  p.LoadFromJSON(poses_file_);
  const std::vector<Eigen::Matrix4d, beam::AlignMat4d>& poses = p.GetPoses();
  const std::vector<ros::Time>& pose_time_stamps = p.GetTimeStamps();

  // iterate over all cameras
  for (auto& camera_data : camera_data_) {
    // get start and end of image topic
    rosbag::View view(bag_, rosbag::TopicQuery(camera_data.topic),
                      ros::TIME_MIN, ros::TIME_MAX, true);

    // get start time
    rosbag::View::iterator iter_start = view.begin();
    ros::Time topic_start_time;
    if (camera_data.compressed) {
      auto img_msg = iter_start->instantiate<sensor_msgs::CompressedImage>();
      topic_start_time = img_msg->header.stamp;
    } else {
      auto img_msg = iter_start->instantiate<sensor_msgs::Image>();
      topic_start_time = img_msg->header.stamp;
    }

    // iterate through bag and get last message time
    rosbag::View::iterator last_iter;
    for (auto iter = view.begin(); iter != view.end(); iter++) {
      last_iter = iter;
    }

    ros::Time topic_end_time;
    if (camera_data.compressed) {
      auto img_msg = last_iter->instantiate<sensor_msgs::CompressedImage>();
      topic_end_time = img_msg->header.stamp;
    } else {
      auto img_msg = last_iter->instantiate<sensor_msgs::Image>();
      topic_end_time = img_msg->header.stamp;
    }

    // iterate over all poses for this camera
    std::vector<ros::Time> time_stamps;
    Eigen::Matrix4d T_moving_fixed_last = poses[0];
    for (uint16_t pose_iter = 1; pose_iter < poses.size(); pose_iter++) {
      // first check that pose time is not outside current bag time
      if (pose_time_stamps[pose_iter] < topic_start_time ||
          pose_time_stamps[pose_iter] > topic_end_time) {
        continue;
      }
      // next, check sufficient motion has passed
      Eigen::Matrix4d T_moving_fixed_curr = poses[pose_iter];
      Eigen::Matrix4d T_curr_last =
          T_moving_fixed_curr.inverse() * T_moving_fixed_last;
      if (beam::PassedMotionThreshold(T_moving_fixed_curr, T_moving_fixed_last,
                                      camera_data.rotation_between_images_rad,
                                      camera_data.distance_between_images_m,
                                      true, false, false)) {
        time_stamps.push_back(pose_time_stamps[pose_iter]);
        T_moving_fixed_last = T_moving_fixed_curr;
      }
    }
    camera_data.timestamps = time_stamps;
    BEAM_INFO("Saving {} images from camera topic: {}", time_stamps.size(),
              camera_data.topic);
  }
}

void ImageExtractor::OutputImages() {
  if (!boost::filesystem::exists(save_directory_)) {
    BEAM_INFO("Creating output directory: {}", save_directory_);
    boost::filesystem::create_directories(save_directory_);
  }
  std::string camera_list_path =
      beam::CombinePaths(save_directory_, "CameraList.json");
  ImageDatabase image_db(camera_list_path);

  // iterate over all cameras
  for (int cam_count = 0; cam_count < camera_data_.size(); cam_count++) {
    auto& camera_data = camera_data_.at(cam_count);
    BEAM_INFO("Saving images for Camera {}.", camera_data.frame_id);

    // iterate over all poses for this camera
    for (const ros::Time& timestamp : camera_data.timestamps) {
      ros::Time real_image_time = timestamp;
      cv::Mat image = GetImageFromBag(real_image_time, cam_count);
      image_db.AddImage(camera_data.frame_id, image, real_image_time,
                        camera_data.output_distorted, camera_data.frame_id,
                        camera_data.ir_camera, bag_file_);
    }
  }
  image_db.WriteMetadata();
}

cv::Mat ImageExtractor::GetImageFromBag(ros::Time& image_time, int cam_number) {
  auto& camera_data = camera_data_.at(cam_number);
  rosbag::View view(bag_, rosbag::TopicQuery(camera_data.topic),
                    last_image_time_, ros::TIME_MAX, true);

  // iterate through bag:
  for (rosbag::View::iterator iter = view.begin(); iter != view.end(); iter++) {
    cv::Mat image_raw;

    sensor_msgs::ImagePtr img_msg = boost::make_shared<sensor_msgs::Image>();
    if (camera_data.compressed) {
      auto img_msg_comp = iter->instantiate<sensor_msgs::CompressedImage>();
      if (img_msg_comp->header.stamp < image_time) { continue; }
      if (!rosbag_tools::utils::DecompressJpegImage(*img_msg_comp, *img_msg)) {
        throw std::runtime_error{"unable to decompress image"};
      };
    } else {
      img_msg = iter->instantiate<sensor_msgs::Image>();
      if (img_msg->header.stamp < image_time) { continue; }
    }

    image_time = img_msg->header.stamp;
    last_image_time_ = img_msg->header.stamp;

    if (img_msg->encoding == sensor_msgs::image_encodings::BGR8) {
      image_raw = beam_cv::OpenCVConversions::RosImgToMat(*img_msg);
    } else if (img_msg->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::Mat image_RGB = beam_cv::OpenCVConversions::RosImgToMat(*img_msg);
      cv::cvtColor(image_RGB, image_raw, cv::COLOR_RGB2BGR);
    } else {
      image_raw = beam_cv::OpenCVConversions::RosImgToMat(*img_msg);
    }

    if (camera_data.transforms.empty()) {
      return image_raw.clone();
    } else {
      return ApplyImageTransforms(image_raw, cam_number);
    }
  }

  BEAM_ERROR("unable to get image from bag for camera {} with timestamp: {}",
             cam_number, std::to_string(image_time.toSec()));
  return cv::Mat();
}

cv::Mat ImageExtractor::ApplyImageTransforms(const cv::Mat& image,
                                             int cam_number) {
  auto& camera_data = camera_data_.at(cam_number);

  // avoid useless copying if no transforms are specified
  if (camera_data.transforms.size() == 0) { return image.clone(); }

  cv::Mat output_image = image.clone();
  for (const ImageTransform& transform : camera_data.transforms) {
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
  return output_image.clone();
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
  cv::cvtColor(image, ycrcb, cv::COLOR_BGR2YCrCb);
  std::vector<cv::Mat> channels;
  cv::split(ycrcb, channels);
  cv::equalizeHist(channels[0], channels[0]);
  cv::Mat new_image;
  cv::merge(channels, ycrcb);
  cv::cvtColor(ycrcb, new_image, cv::COLOR_YCrCb2BGR);
  return new_image;
}

cv::Mat ImageExtractor::ApplyUndistortTransform(
    const cv::Mat& image, const std::vector<double>& params, int cam_number) {
  // create undistortion object if not created already
  auto& camera_data = camera_data_.at(cam_number);
  if (camera_data.rectifier == nullptr) {
    camera_data.output_distorted = false;
    Eigen::Vector2i src_image_dims;
    Eigen::Vector2i dst_image_dims;
    if (camera_data.downsampled) {
      src_image_dims[0] = camera_data.model->GetHeight();
      src_image_dims[1] = camera_data.model->GetWidth();
      dst_image_dims[0] =
          static_cast<int>(camera_data.model->GetHeight() * params[0]);
      dst_image_dims[1] =
          static_cast<int>(camera_data.model->GetWidth() * params[1]);
    } else {
      src_image_dims[0] = image.rows;
      src_image_dims[1] = image.cols;
      dst_image_dims[0] = static_cast<int>(image.rows * params[0]);
      dst_image_dims[1] = static_cast<int>(image.cols * params[1]);
    }

    beam_calibration::UndistortImages undistort(camera_data.model,
                                                src_image_dims, dst_image_dims);
    camera_data.rectifier =
        std::make_shared<beam_calibration::UndistortImages>(undistort);
  }

  if (camera_data.downsampled) {
    cv::Mat image_upsampled = camera_data.rectifier->UpsampleImage(image);
    cv::Mat image_undistorted =
        camera_data.rectifier->ConvertImage<cv::Vec3b>(image_upsampled);
    return camera_data.rectifier->DownsampleImage(
        image_undistorted, Eigen::Vector2i(image.rows, image.cols));
  }

  return camera_data.rectifier->ConvertImage<cv::Vec3b>(image);
}

cv::Mat ImageExtractor::ApplyClaheTransform(const cv::Mat& image,
                                            const std::vector<double>& params) {
  cv::Mat lab_image;
  cv::cvtColor(image, lab_image, cv::COLOR_BGR2Lab);
  std::vector<cv::Mat> lab_planes(6);
  cv::split(lab_image, lab_planes);
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(3);
  cv::Mat dst;
  clahe->apply(lab_planes[0], dst);
  dst.copyTo(lab_planes[0]);
  cv::merge(lab_planes, lab_image);
  cv::Mat new_image;
  cv::cvtColor(lab_image, new_image, cv::COLOR_Lab2BGR);
  return new_image;
}

} // end namespace inspection
