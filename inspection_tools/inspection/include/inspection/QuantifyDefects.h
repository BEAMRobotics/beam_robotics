#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <beam_containers/PointBridge.h>

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
  QuantifyDefects(const std::string& cloud_filename,
                  const std::string& output_directory,
                  const std::string& config_file_location);

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
  float crack_threshold_;
  float spall_threshold_;
  float delam_threshold_;
  float corrosion_threshold_;
  std::string cloud_filename_;
  std::string cloud_savedir_;
  pcl::PointCloud<beam_containers::PointBridge>::Ptr point_cloud_;
};

} // namespace inspection
