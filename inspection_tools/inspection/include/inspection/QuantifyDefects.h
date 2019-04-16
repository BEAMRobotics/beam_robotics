#pragma once

#include<string>

namespace inspection
{

  /**
   * @brief class for quantifying defects from a labeled map, using libbeam
   * beam_defects
   */
  class QuantifyDefects
  {
  public:

    QuantifyDefects(const std::string config_file_location);

    ~QuantifyDefects() = default;

  private:
  };

} // end inspection namespace
