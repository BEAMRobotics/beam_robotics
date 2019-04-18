#pragma once

//#include <beam_defects/extract_functions.h>
#include <beam_defects/Crack.h>
#include <beam_defects/Spall.h>
#include <beam_defects/Delam.h>
#include <beam_containers/PointBridge.h>

#include <pcl/point_types.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace inspection {

/**
 * @brief class for quantifying defects from a labeled map, using libbeam
 * beam_defects
 */
class QuantifyDefects {
public:
  /**
   * @brief Default constructor
   */
  QuantifyDefects(const std::string config_file_location);

  /**
   * @brief Default destructor
   */
  ~QuantifyDefects() = default;

  void OutputCrackInfo();
  void OutputSpallInfo();
  void OutputDelamInfo();

  void SaveCrackOnlyCloud();
  void SaveSpallOnlyCloud();
  void SaveDelamOnlyCloud();

private:
  float crack_threshold_, spall_threshold_, delam_threshold_, cloud_filename_;
  // pcl::PointCloud<beam_containers::PointBridge>::Ptr =
  //   boost::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();

};

} // namespace inspection
