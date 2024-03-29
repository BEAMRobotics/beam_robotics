#include <gflags/gflags.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

#include <beam_calibration/CameraModel.h>
#include <beam_calibration/ConvertCameraModel.h>
#include <beam_cv/OpenCVConversions.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/gflags.h>

#include <rosbag_tools/RosBagIO.h>
#include <rosbag_tools/VelodyneTools.h>

DEFINE_string(input, "", "Full path to input bag file (Required)");
DEFINE_validator(input, &beam::gflags::ValidateBagFileMustExist);
DEFINE_string(output, "", "Full path to output bag file (Required)");
DEFINE_validator(output, &beam::gflags::ValidateDirMustExist);
DEFINE_bool(unpack_scans, false,
            "Set to true if you want to unpack velodyne scans. New topic will "
            "be the scan topic post-fixed with _unpacked");
DEFINE_bool(debayer_images, true,
            "Set to true if you want to debayer images. Topics will be "
            "post-fixed with _debayered");
DEFINE_bool(rectify_images, false,
            "Set to true if you want to rectify images.");
DEFINE_bool(compress_images, false,
            "Set to true if you want to compress the images.");
DEFINE_double(
    resize_multiplier, 1,
    "Setting to a value in range (0,1) will resize images using this "
    "multiplier (i.e., w x resize_multiplier by h x resize_multiplier). Topics "
    "will be post-fixed with _resized. Note this will override the "
    "_debayered post-fix from the flag above. Debayering is "
    "necessary if this is set to anything other than 1");
DEFINE_string(lidar_model, "VLP16",
              "Only needed if unpack_scans is set to true. Options: VLP16, "
              "32C, 32E, VLS128. (Optional)");
DEFINE_string(camera_model_path, "",
              "Path to camera model (Required if rectified = true)");

bool compress_images_;

namespace enc = sensor_msgs::image_encodings;

sensor_msgs::Image ProcessImage(
    const sensor_msgs::Image& msg, const double& resize_multiplier,
    std::shared_ptr<beam_calibration::ConvertCameraModel>& converter) {
  using namespace beam_cv;
  using namespace OpenCVConversions;

  // else, convert to cv::Mat which will automatically do the encoding
  cv::Mat mat = RosImgToMat(msg);

  // rectify image
  if (FLAGS_rectify_images) { mat = converter->ConvertImage<cv::Vec3b>(mat); }

  cv::Mat mat_resized = mat.clone();

  if (resize_multiplier != 1) {
    cv::resize(
        mat, mat_resized,
        cv::Size(resize_multiplier * mat.cols, resize_multiplier * mat.rows),
        cv::INTER_LINEAR);
  }

  std::string encoding = msg.encoding;
  if (msg.encoding.find("bayer") != std::string::npos) {
    auto iter = bayer_decoding_map.find(encoding);
    if (iter == bayer_decoding_map.end()) {
      BEAM_CRITICAL("Bayered image encoding ({}) not yet supported",
                    msg.encoding);
      throw std::runtime_error{"image encoding not supported"};
    }
    encoding = iter->second;
  }
  return MatToRosImg(mat_resized, msg.header, encoding);
}

bool CompressImage(const sensor_msgs::Image& msg,
                   sensor_msgs::CompressedImage& compressed) {
  compressed.header = msg.header;
  compressed.format = msg.encoding;

  // Compression settings
  std::vector<int> params;
  params.resize(3, 0);

  // Bit depth of image encoding
  int bitDepth = sensor_msgs::image_encodings::bitDepth(msg.encoding);
  int numChannels = sensor_msgs::image_encodings::numChannels(msg.encoding);

  params[0] = 85;
  params[1] = 85;

  // Update ros message format header
  compressed.format += "; jpeg compressed";

  // check input format
  if (bitDepth != 8) {
    BEAM_ERROR(
        "JPEG compression only works on 8 bit images, not compressing images");
    return false;
  }
  if (numChannels != 1 && numChannels != 3) {
    BEAM_ERROR(
        "Cannot compress images with 1 or 3 channels, not compressing images");
    return false;
  }

  // Target image format
  std::stringstream targetFormat;
  if (sensor_msgs::image_encodings::isColor(msg.encoding)) {
    // convert color images to RGB domain
    targetFormat << "rgb" << bitDepth;
  }

  // OpenCV-ros bridge
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, targetFormat.str());

    // Compress image
    if (!cv::imencode(".jpg", cv_ptr->image, compressed.data, params)) {
      BEAM_ERROR("cv::imencode (jpeg) failed on input image");
      return false;
    }
  } catch (cv_bridge::Exception& e) {
    BEAM_ERROR("{}", e.what());
    return false;
  } catch (cv::Exception& e) {
    BEAM_ERROR("{}", e.what());
    return false;
  }
  return true;
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  using namespace rosbag_tools;

  // check inputs
  if (FLAGS_resize_multiplier <= 0) {
    BEAM_CRITICAL("Invalid resize_multiplier params, must be in (0,1]");
    throw std::invalid_argument{"invalid resize_multiplier"};
  }
  ros::Time::init();
  VelodyneTools velodyne_tools(FLAGS_lidar_model);

  compress_images_ = FLAGS_compress_images;

  boost::filesystem::path p(FLAGS_input);

  RosBagReader reader(FLAGS_input);
  RosBagWriter writer(FLAGS_output);

  std::shared_ptr<beam_calibration::CameraModel> cam_model;
  std::shared_ptr<beam_calibration::ConvertCameraModel> converter;
  if (FLAGS_rectify_images) {
    cam_model = beam_calibration::CameraModel::Create(FLAGS_camera_model_path);
    auto height = cam_model->GetHeight();
    auto width = cam_model->GetWidth();
    Eigen::Vector2i size(height, width);
    converter = std::make_shared<beam_calibration::ConvertCameraModel>(
        cam_model, size, size);
  }

  while (true) {
    rosbag::View::iterator iter;
    if (!reader.GetNextMsg(iter)) { break; }

    if (FLAGS_unpack_scans) {
      auto maybe_lidar_msg = iter->instantiate<velodyne_msgs::VelodyneScan>();
      if (maybe_lidar_msg) {
        sensor_msgs::PointCloud2 cloud =
            velodyne_tools.UnpackScan(maybe_lidar_msg);
        writer.AddMsg(iter->getTopic() + "_unpacked", iter->getTime(), cloud);
        continue;
      }
    }

    if (FLAGS_debayer_images || FLAGS_resize_multiplier != 1 ||
        FLAGS_compress_images) {
      std::string postfix =
          FLAGS_resize_multiplier != 1 ? "_resized" : "_debayered";
      auto maybe_image_msg = iter->instantiate<sensor_msgs::Image>();
      if (maybe_image_msg != nullptr) {
        sensor_msgs::Image new_msg =
            ProcessImage(*maybe_image_msg, FLAGS_resize_multiplier, converter);
        if (compress_images_) {
          sensor_msgs::CompressedImage img_comp;
          if (!CompressImage(new_msg, img_comp)) { compress_images_ = false; }
          writer.AddMsg(iter->getTopic() + postfix, iter->getTime(), img_comp);
        } else {
          writer.AddMsg(iter->getTopic() + postfix, iter->getTime(), new_msg);
        }

        continue;
      }
    }

    // if we reach here, that means we aren't editing this message so we just
    // copy it
    writer.AddMsg(*iter);
  }

  writer.CloseBag();
  BEAM_INFO("ProcessBag ran successfully!");
  ros::Time::shutdown();
  return 0;
}
