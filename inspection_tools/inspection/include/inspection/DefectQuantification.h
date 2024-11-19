#pragma once

#include <nlohmann/json.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <beam_containers/PointBridge.h>
#include <beam_defects/Defect.h>

namespace inspection {

/**
 * @brief class for quantifying defects from a labeled map, using libbeam
 * beam_defects
 */
class DefectQuantification {
public:
  DefectQuantification(const std::string& cloud_filepath,
                       const std::string& output_directory,
                       const std::string& config_file_location);

  ~DefectQuantification() = default;

  void ProcessDefects();

  void SaveResults();

private:
  nlohmann::json GetDefectInfo(const beam_defects::Defect::Ptr& defect, int id);

  void SaveDefectCloud(const beam_defects::Defect::Ptr& defect,
                       const std::string save_path);

  // Config / inputs
  float crack_threshold_;
  float spall_threshold_;
  float delam_threshold_;
  float corrosion_threshold_;
  float euc_clustering_tol_m_;
  int euc_clustering_min_points_;
  float concave_hull_alpha_;
  std::string cloud_filepath_;
  std::string output_directory_;
  std::string config_file_location_;
  pcl::PointCloud<beam_containers::PointBridge>::Ptr point_cloud_;

  // Results
  std::vector<beam_defects::Defect::Ptr> cracks_;
  std::vector<beam_defects::Defect::Ptr> spalls_;
  std::vector<beam_defects::Defect::Ptr> delams_;
  std::vector<beam_defects::Defect::Ptr> corrosions_;
};

} // namespace inspection
