#pragma once

#include<string>

namespace inspection
{

  /**
   * @brief class for labeling images.
   */
  class ImageLabeler
  {
  public:

    ImageLabeler(const std::string config_file_location);

    ~ImageLabeler() = default;

  private:
  };

} // end inspection namespace
