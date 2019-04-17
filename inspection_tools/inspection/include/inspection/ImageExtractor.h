#pragma once

#include <string>

namespace inspection {

/**
 * @brief class for extracting images from a ROS bag
 */
class ImageExtractor {
public:
  ImageExtractor(const std::string config_file_location);

  ~ImageExtractor() = default;

private:
};

} // namespace inspection
