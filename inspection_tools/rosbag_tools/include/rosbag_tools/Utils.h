#pragma once

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

namespace rosbag_tools::utils {

// Encodes to JPEG. Adapted from:
// github.com/ethz-asl/ros-message-transport/blob/master/compressed_imagem_transport/src/compressed_publisher.cpp
// Returns true if successful
bool CompressImage(const sensor_msgs::Image& msg,
                   sensor_msgs::CompressedImage& compressed);

// Decompressed image from JPEG format, see CompressImage above. Adapted from:
// github.com/jkammerl/compressed_image_transport/blob/master/src/compressed_subscriber.cpp
// Returns true if successful
bool DecompressJpegImage(const sensor_msgs::CompressedImage& image,
                         sensor_msgs::Image& decompressed);

} // namespace rosbag_tools::utils
