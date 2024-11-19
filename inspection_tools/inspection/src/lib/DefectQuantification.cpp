#include <inspection/DefectQuantification.h>

#include <pcl/io/pcd_io.h>

#include <beam_defects/extract_functions.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/log.h>
#include <beam_utils/pointclouds.h>

namespace inspection {

DefectQuantification::DefectQuantification(
    const std::string& cloud_filepath, const std::string& output_directory,
    const std::string& config_file_location) {
  cloud_filepath_ = cloud_filepath;
  output_directory_ = output_directory;
  config_file_location_ = config_file_location;

  BEAM_INFO("Loading config from: {}", config_file_location);
  nlohmann::json J;
  if (!beam::ReadJson(config_file_location, J)) {
    throw std::runtime_error{"invalid file path"};
  }
  beam::ValidateJsonKeysOrThrow(
      {"crack_threshold", "spall_threshold", "delam_threshold",
       "corrosion_threshold", "euc_clustering_tol_m",
       "euc_clustering_min_points", "concave_hull_alpha"},
      J);

  crack_threshold_ = J["crack_threshold"];
  spall_threshold_ = J["spall_threshold"];
  delam_threshold_ = J["delam_threshold"];
  corrosion_threshold_ = J["corrosion_threshold"];
  euc_clustering_tol_m_ = J["euc_clustering_tol_m"];
  euc_clustering_min_points_ = J["euc_clustering_min_points"];
  concave_hull_alpha_ = J["concave_hull_alpha"];

  BEAM_INFO("Reading input cloud: {}", cloud_filepath_);
  point_cloud_ =
      std::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();
  pcl::PCDReader reader;
  reader.read(cloud_filepath_, *point_cloud_);
  BEAM_INFO("Done setting up DefectQuantification");
}

void DefectQuantification::ProcessDefects() {
  BEAM_INFO("Extracting and quantifying cracks");
  cracks_ = beam_defects::GetCracks(point_cloud_, crack_threshold_,
                                    concave_hull_alpha_);
  BEAM_INFO("Extracted and quantified {} cracks", cracks_.size());

  BEAM_INFO("Extracting and quantifying spalls");
  spalls_ = beam_defects::GetSpalls(point_cloud_, crack_threshold_,
                                    concave_hull_alpha_);
  BEAM_INFO("Extracted and quantified {} spalls", spalls_.size());

  BEAM_INFO("Extracting and quantifying delaminations");
  delams_ = beam_defects::GetDelams(point_cloud_, crack_threshold_,
                                    concave_hull_alpha_);
  BEAM_INFO("Extracted and quantified {} delaminations", delams_.size());

  BEAM_INFO("Extracting and quantifying corrosions");
  corrosions_ = beam_defects::GetCorrosion(point_cloud_, crack_threshold_,
                                           concave_hull_alpha_);
  BEAM_INFO("Extracted and quantified {} corrosions", corrosions_.size());
}

void DefectQuantification::SaveResults() {
  nlohmann::json J;

  // Save crack results
  std::vector<nlohmann::json> J_cracks;
  for (int i = 0; i < cracks_.size(); i++) {
    nlohmann::json J_crack = GetDefectInfo(cracks_[i], i);
    J_cracks.push_back(J_crack);
    std::string save_path = beam::CombinePaths(
        output_directory_, "crack_" + std::to_string(i) + ".pcd");
    SaveDefectCloud(cracks_[i], save_path);
  }
  J["cracks"] = J_cracks;

  // save spall results
  std::vector<nlohmann::json> J_spalls;
  for (int i = 0; i < spalls_.size(); i++) {
    nlohmann::json J_spall = GetDefectInfo(spalls_[i], i);
    J_spalls.push_back(J_spall);
    std::string save_path = beam::CombinePaths(
        output_directory_, "spall_" + std::to_string(i) + ".pcd");
    SaveDefectCloud(spalls_[i], save_path);
  }
  J["spalls"] = J_spalls;

  // save delams results
  std::vector<nlohmann::json> J_delams;
  for (int i = 0; i < delams_.size(); i++) {
    nlohmann::json J_delam = GetDefectInfo(delams_[i], i);
    J_delams.push_back(J_delam);
    std::string save_path = beam::CombinePaths(
        output_directory_, "delam_" + std::to_string(i) + ".pcd");
    SaveDefectCloud(delams_[i], save_path);
  }
  J["delams"] = J_delams;

  // save corrosion results
  std::vector<nlohmann::json> J_corrosions;
  for (int i = 0; i < corrosions_.size(); i++) {
    nlohmann::json J_corrosion = GetDefectInfo(corrosions_[i], i);
    J_corrosions.push_back(J_corrosion);
    std::string save_path = beam::CombinePaths(
        output_directory_, "corrosion_" + std::to_string(i) + ".pcd");
    SaveDefectCloud(corrosions_[i], save_path);
  }
  J["corrosions"] = J_corrosions;

  std::string defects_summary_path =
      beam::CombinePaths(output_directory_, "defects_summary.json");
  BEAM_INFO("Saving defects info to {}", defects_summary_path);
  std::ofstream o(defects_summary_path);
  o << std::setw(4) << J << std::endl;
}

nlohmann::json
    DefectQuantification::GetDefectInfo(const beam_defects::Defect::Ptr& defect,
                                        int id) {
  nlohmann::json J;
  J["id"] = id;
  J["size"] = defect->GetSize();
  J["max_dim_2d"] = defect->GetMaxDim2D();
  J["bbox_2d_dims"] = defect->GetBBoxDims2D();
  J["num_defect_points"] = defect->GetPointCloud()->size();
  J["num_2d_hull_points"] = defect->GetHull2D()->size();
  return J;
}

void DefectQuantification::SaveDefectCloud(
    const beam_defects::Defect::Ptr& defect, const std::string save_path) {
  PointCloud defect_points = *defect->GetPointCloud();
  PointCloudCol defect_colored =
      beam::ColorPointCloud(defect_points, 255, 255, 0); // yellow
  // TODO: fix this. Currently the hull is in some other frame so the viz
  // doesn't make sense. The hull calculation function has the ability to get
  // the indices for the points that were used so we can use that to extract the
  // original points belonging to the hull
  // PointCloud hull_points = *defect->GetHull2D();
  // PointCloudCol hull_points_col =
  //     beam::ColorPointCloud(hull_points, 255, 0, 0); // red
  // defect_colored += hull_points_col;

  BEAM_INFO("Saving defect cloud to {}", save_path);
  beam::SavePointCloud(save_path, defect_colored);
}

} // end namespace inspection
