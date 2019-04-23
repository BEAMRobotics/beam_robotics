#include "inspection/QuantifyDefects.h"

namespace inspection {
QuantifyDefects::QuantifyDefects(const std::string config_file_location) {
  // load member variables
  nlohmann::json J;
  std::ifstream file(config_file_location);
  file >> J;

  cloud_filename_ = J["cloud_filename"];
  cloud_savedir_ = J["cloud_savedir"];
  crack_threshold_ = J["crack_threshold"];
  spall_threshold_ = J["spall_threshold"];
  delam_threshold_ = J["delam_threshold"];
  corrosion_threshold_ = J["corrosion_threshold"];

  pcl::PCDReader reader;
  // auto cloud = boost::make_shared<pcl::PointCloud<beam_containers::PointBridge>>();
  reader.read(cloud_filename_, *point_cloud_);
}

void QuantifyDefects::OutputCrackInfo() {
  std::vector<beam_defects::Crack> crack_vector_ =
    beam_defects::GetCracks(point_cloud_, crack_threshold_);

  int j = 1;
  for (auto& defect : crack_vector_){
    std::cout << "Crack " << j << " length: " << defect.GetSize() << "m"
              << std::endl;
    j++;
  }
};

void QuantifyDefects::OutputSpallInfo() {
  std::vector<beam_defects::Spall> spall_vector =
    beam_defects::GetSpalls(point_cloud_, spall_threshold_);

  int j = 1;
  for (auto& defect : spall_vector){
    std::cout << "Spall " << j << " area: " << defect.GetSize() << "m^2"
              << std::endl;
    j++;
  }
};

void QuantifyDefects::OutputDelamInfo() {
  std::vector<beam_defects::Delam> delam_vector =
    beam_defects::GetDelams(point_cloud_, delam_threshold_);

  int j = 1;
  for (auto& defect : delam_vector){
    std::cout << "Delamination " << j << " area: " << defect.GetSize() << "m^2"
              << std::endl;
    j++;
  }
};

void QuantifyDefects::OutputCorrosionInfo() {
  std::vector<beam_defects::Corrosion> corrosion_vector =
    beam_defects::GetCorrosion(point_cloud_, corrosion_threshold_);

  int j = 1;
  for (auto& defect : corrosion_vector){
    std::cout << "Patch Repair " << j << " area: " << defect.GetSize() << "m^2"
              << std::endl;
    j++;
  }
}

void QuantifyDefects::SaveCrackOnlyCloud() {
  auto cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *cloud_filtered = beam_defects::IsolateCrackPoints(point_cloud_, crack_threshold_);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector =
    beam_defects::GetExtractedClouds(cloud_filtered);

  int j = 1;
  for (auto& cloud : cloud_vector){
    std::stringstream ss;
    ss << cloud_savedir_ << "crack_" << j << ".pcd";
    pcl::io::savePCDFile (ss.str(), *cloud);
    j++;
  }
};

void QuantifyDefects::SaveSpallOnlyCloud() {
  auto cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *cloud_filtered = beam_defects::IsolateSpallPoints(point_cloud_, spall_threshold_);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector =
    beam_defects::GetExtractedClouds(cloud_filtered);

  int j = 1;
  for (auto& cloud : cloud_vector){
    std::stringstream ss;
    ss << cloud_savedir_ << "spall_" << j << ".pcd";
    pcl::io::savePCDFile (ss.str(), *cloud);
    j++;
  }
};

void QuantifyDefects::SaveDelamOnlyCloud() {
  auto cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *cloud_filtered = beam_defects::IsolateDelamPoints(point_cloud_, delam_threshold_);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector =
    beam_defects::GetExtractedClouds(cloud_filtered);

  pcl::io::savePCDFile("/home/beam-install/clouds/test.pcd", *cloud_filtered);
  int j = 1;
  for (auto& cloud : cloud_vector){
    std::stringstream ss;
    ss << cloud_savedir_ << "delam_" << j << ".pcd";
    pcl::io::savePCDFile (ss.str(), *cloud);
    j++;
  }
};

void QuantifyDefects::SaveCorrosionOnlyCloud() {
  auto cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *cloud_filtered = beam_defects::IsolateCorrosionPoints(point_cloud_, corrosion_threshold_);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector =
    beam_defects::GetExtractedClouds(cloud_filtered);

  int j = 1;
  for (auto& cloud : cloud_vector){
    std::stringstream ss;
    ss << cloud_savedir_ << "patch_repair_" << j << ".pcd";
    pcl::io::savePCDFile (ss.str(), *cloud);
    j++;
  }
}
} // end namespace inspection
