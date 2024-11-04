#include <inspection/QuantifyDefects.h>

#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>

#include <beam_defects/Corrosion.h>
#include <beam_defects/Crack.h>
#include <beam_defects/Delam.h>
#include <beam_defects/Spall.h>
#include <beam_defects/extract_functions.h>
#include <beam_utils/log.h>

namespace inspection {

QuantifyDefects::QuantifyDefects(const std::string& cloud_filename,
                                 const std::string& output_directory,
                                 const std::string& config_file_location) {
  point_cloud_ =
      std::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();
  cloud_filename_ = cloud_filename;
  cloud_savedir_ = output_directory;

  // load member variables from json
  BEAM_INFO("Loading config from: {}", config_file_location);
  nlohmann::json J;
  std::ifstream file(config_file_location);
  file >> J;

  crack_threshold_ = J["crack_threshold"];
  spall_threshold_ = J["spall_threshold"];
  delam_threshold_ = J["delam_threshold"];
  corrosion_threshold_ = J["corrosion_threshold"];

  BEAM_INFO("Reading input cloud: {}", cloud_filename_);
  pcl::PCDReader reader;
  reader.read(cloud_filename_, *point_cloud_);
  BEAM_INFO("Done setting up QuantifyDefects");
}

void QuantifyDefects::OutputCrackInfo() {
  BEAM_INFO("Getting crack info");
  std::vector<beam_defects::Defect::Ptr> crack_vector_ =
      beam_defects::GetCracks(point_cloud_, crack_threshold_);
  if (crack_vector_.empty()) {
    BEAM_INFO("No crack labels found");
    return;
  }
  int j = 1;
  for (auto& defect : crack_vector_) {
    std::cout << "Crack " << j << " length: " << defect->GetSize() << "m"
              << std::endl;
    j++;
  }
};

void QuantifyDefects::OutputSpallInfo() {
  BEAM_INFO("Getting spall info");
  std::vector<beam_defects::Defect::Ptr> spall_vector =
      beam_defects::GetSpalls(point_cloud_, spall_threshold_);
  if (spall_vector.empty()) {
    BEAM_INFO("No spall labels found");
    return;
  }
  int j = 1;
  for (auto& defect : spall_vector) {
    std::cout << "Spall " << j << " area: " << defect->GetSize() << "m^2"
              << std::endl;
    j++;
  }
};

void QuantifyDefects::OutputDelamInfo() {
  BEAM_INFO("Getting delamination info");
  std::vector<beam_defects::Defect::Ptr> delam_vector =
      beam_defects::GetDelams(point_cloud_, delam_threshold_);
  if (delam_vector.empty()) {
    BEAM_INFO("No delam labels found");
    return;
  }
  int j = 1;
  for (auto& defect : delam_vector) {
    std::cout << "Delamination " << j << " area: " << defect->GetSize() << "m^2"
              << std::endl;
    j++;
  }
};

void QuantifyDefects::OutputCorrosionInfo() {
  BEAM_INFO("Getting corrosion info");
  std::vector<beam_defects::Defect::Ptr> corrosion_vector =
      beam_defects::GetCorrosion(point_cloud_, corrosion_threshold_);
  if (corrosion_vector.empty()) { BEAM_INFO("No corrosion labels found"); }
  int j = 1;
  for (auto& defect : corrosion_vector) {
    std::cout << "Corrosion " << j << " area: " << defect->GetSize() << "m^2"
              << std::endl;
    j++;
  }
}

void QuantifyDefects::SaveCrackOnlyCloud() {
  auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *cloud_filtered =
      beam_defects::IsolateCrackPoints(point_cloud_, crack_threshold_);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector =
      beam_defects::GetExtractedClouds(cloud_filtered);

  int j = 1;
  for (auto& cloud : cloud_vector) {
    std::stringstream ss;
    ss << cloud_savedir_ << "crack_" << j << ".pcd";
    pcl::io::savePCDFile(ss.str(), *cloud);
    j++;
  }
};

void QuantifyDefects::SaveSpallOnlyCloud() {
  auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *cloud_filtered =
      beam_defects::IsolateSpallPoints(point_cloud_, spall_threshold_);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector =
      beam_defects::GetExtractedClouds(cloud_filtered);

  int j = 1;
  for (auto& cloud : cloud_vector) {
    std::stringstream ss;
    ss << cloud_savedir_ << "spall_" << j << ".pcd";
    pcl::io::savePCDFile(ss.str(), *cloud);
    j++;
  }
};

void QuantifyDefects::SaveDelamOnlyCloud() {
  auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *cloud_filtered =
      beam_defects::IsolateDelamPoints(point_cloud_, delam_threshold_);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector =
      beam_defects::GetExtractedClouds(cloud_filtered);

  int j = 1;
  for (auto& cloud : cloud_vector) {
    std::stringstream ss;
    ss << cloud_savedir_ << "delam_" << j << ".pcd";
    pcl::io::savePCDFile(ss.str(), *cloud);
    j++;
  }
};

void QuantifyDefects::SaveCorrosionOnlyCloud() {
  auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *cloud_filtered =
      beam_defects::IsolateCorrosionPoints(point_cloud_, corrosion_threshold_);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector =
      beam_defects::GetExtractedClouds(cloud_filtered);

  int j = 1;
  for (auto& cloud : cloud_vector) {
    std::stringstream ss;
    ss << cloud_savedir_ << "corrosion_" << j << ".pcd";
    pcl::io::savePCDFile(ss.str(), *cloud);
    j++;
  }
};

} // end namespace inspection
