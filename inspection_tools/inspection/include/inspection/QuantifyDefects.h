#pragma once

#include <beam_defects/extract_functions.h>
#include <beam_defects/Crack.h>
#include <beam_defects/Spall.h>
#include <beam_defects/Delam.h>
#include <beam_defects/Corrosion.h>
#include <beam_containers/PointBridge.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

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

  /**
   * @brief Ouput the indexed cracks and corresponding lengths
   */
  void OutputCrackInfo();

  /**
   * @brief Ouput the indexed spalls and corresponding areas
   */
  void OutputSpallInfo();

  /**
   * @brief Ouput the indexed delaminations and corresponding areas
   */
  void OutputDelamInfo();

  /**
   * @brief Ouput the indexed corrosion and corresponding areas
   */
  void OutputCorrosionInfo();

  /**
   * @brief Save individual crack point clouds to directory
   */
  void SaveCrackOnlyCloud();

  /**
   * @brief Save individual spall point clouds to directory
   */
  void SaveSpallOnlyCloud();

  /**
   * @brief Save individual delamination point clouds to directory
   */
  void SaveDelamOnlyCloud();

  /**
   * @brief Save individual corrosion point clouds to directory
   */
  void SaveCorrosionOnlyCloud();

private:
  float crack_threshold_, spall_threshold_, delam_threshold_;
  float corrosion_threshold_;
  std::string cloud_filename_, cloud_savedir_;
  pcl::PointCloud<beam_containers::PointBridge>::Ptr point_cloud_ =
     boost::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();

};

} // namespace inspection
