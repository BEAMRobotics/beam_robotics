#include <rosbag_tools/Utils.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

#include <beam_utils/log.h>

namespace rosbag_tools::utils {

namespace enc = sensor_msgs::image_encodings;

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

bool DecompressJpegImage(const sensor_msgs::CompressedImage& image,
                         sensor_msgs::Image& decompressed) {
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy image header
  cv_ptr->header = image.header;

  // Assign image encoding
  std::string image_encoding = image.format.substr(0, image.format.find(';'));
  cv_ptr->encoding = image_encoding;

  // Decode color/mono image
  try {
    cv_ptr->image = cv::imdecode(cv::Mat(image.data), cv::IMREAD_UNCHANGED);

    if (enc::isColor(image_encoding)) {
      // Revert color transformation
      if ((image_encoding == enc::BGR8) || (image_encoding == enc::BGR16))
        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);

      if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGRA);

      if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2RGBA);
    }
  } catch (cv::Exception& e) { BEAM_ERROR("{}", e.what()); }

  size_t rows = cv_ptr->image.rows;
  size_t cols = cv_ptr->image.cols;

  if ((rows > 0) && (cols > 0)) {
    decompressed = *cv_ptr->toImageMsg();
    return true;
  } else {
    BEAM_ERROR("unable to decompress image, result is empty");
    return false;
  }
}

} // namespace rosbag_tools::utils
