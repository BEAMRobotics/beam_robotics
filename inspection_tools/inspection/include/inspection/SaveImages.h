#pragma once

#include<string>

namespace inspection
{

  /**
   * @brief class for saving images from ImageExtractor for hand labeling
   */
  class SaveImages
  {
  public:

    SaveImages(const std::string config_file_location);

    ~SaveImages() = default;

  private:
  };

} // end inspection namespace
