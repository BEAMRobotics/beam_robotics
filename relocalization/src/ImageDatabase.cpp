#include <ImageDatabase.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <pwd.h>
#include <sys/types.h>
#include <unistd.h>

#include <beam_cv/descriptors/BRISKDescriptor.h>
#include <beam_cv/descriptors/ORBDescriptor.h>
#include <beam_cv/descriptors/SIFTDescriptor.h>

#include <beam_cv/detectors/FASTDetector.h>
#include <beam_cv/detectors/ORBDetector.h>
#include <beam_cv/detectors/SIFTDetector.h>

#include "beam_cv/Utils.h"
#include <beam_calibration/TfTree.h>
#include <beam_mapping/Poses.h>


namespace relocalization {

ImageDatabase::ImageDatabase() {
  auto time = std::chrono::system_clock::now();
  std::time_t date = std::chrono::system_clock::to_time_t(time);
  std::string cur_date = std::string(std::ctime(&date));
  std::replace(cur_date.begin(), cur_date.end(), ' ', '_');
  std::replace(cur_date.begin(), cur_date.end(), '\n', '_');

  std::string current_location = __FILE__;
  current_location.erase(current_location.end() - 50, current_location.end());
  //create default image location in user's home directory
  struct passwd* pw = getpwuid(getuid());
  std::string HOME = pw->pw_dir;
  image_folder_ = HOME + "/" + cur_date + "/";
}

ImageDatabase::ImageDatabase(std::string database_config) : ImageDatabase() {
  json J_db_config;
  std::ifstream db_config(database_config);
  db_config >> J_db_config;
  // read paths to both databases
  std::string DBoW3_path = J_db_config["DBoW3_database_path"];
  std::string image_db_path = J_db_config["image_db_path"];
  // load DBoW3 and image databases
  this->LoadDatabase(DBoW3_path, image_db_path);

  // determine descriptor to use
  std::string descriptor_t = J_db_config["descriptor"];
  if (descriptor_types_[descriptor_t] == DescriptorType::ORB) {
    this->descriptor_ = std::make_shared<beam_cv::ORBDescriptor>();
  } else if (descriptor_types_[descriptor_t] == DescriptorType::SIFT) {
    this->descriptor_ = std::make_shared<beam_cv::SIFTDescriptor>();
  } else if (descriptor_types_[descriptor_t] == DescriptorType::BRISK) {
    this->descriptor_ = std::make_shared<beam_cv::BRISKDescriptor>();
  }
  // determine the detector to use
  std::string detector_t = J_db_config["detector"];
  if (detector_types_[detector_t] == DetectorType::ORB) {
    this->detector_ = std::make_shared<beam_cv::ORBDetector>();
  } else if (detector_types_[detector_t] == DetectorType::SIFT) {
    this->detector_ = std::make_shared<beam_cv::SIFTDetector>();
  } else if (detector_types_[detector_t] == DetectorType::FAST) {
    this->detector_ = std::make_shared<beam_cv::FASTDetector>();
  }
}

ImageDatabase::ImageDatabase(std::string vocab_location,
                             std::shared_ptr<beam_cv::Descriptor> descriptor,
                             std::shared_ptr<beam_cv::Detector> detector)
    : ImageDatabase() {
  DBoW3::Vocabulary vocab(vocab_location);
  this->bow_db_ = std::make_shared<DBoW3::Database>(vocab);
  this->descriptor_ = descriptor;
  this->detector_ = detector;
}

void ImageDatabase::LoadDatabase(std::string DBoW3_file_path,
                                 std::string imagedb_file_path) {
  // load DBoW3 database into bow db object
  this->bow_db_ = std::make_shared<DBoW3::Database>(DBoW3_file_path);
  // load image db into image db json object
  std::ifstream image_db_file(imagedb_file_path);
  image_db_ = json::object();
  image_db_file >> image_db_;
}

void ImageDatabase::SaveDatabase(std::string DBoW3_file_path,
                                 std::string imagedb_file_path) {
  // save DBoW3 database
  this->bow_db_->save(DBoW3_file_path);
  // write image db to file
  std::ofstream image_db_file_out(imagedb_file_path);
  image_db_file_out << image_db_.dump(4);
}

void ImageDatabase::SetDescriptor(
    std::shared_ptr<beam_cv::Descriptor> descriptor) {
  this->descriptor_ = descriptor;
}

void ImageDatabase::SetDetector(std::shared_ptr<beam_cv::Detector> detector) {
  this->detector_ = detector;
}

void ImageDatabase::RetrainVocabulary() {
  std::vector<cv::Mat> features;
  for (unsigned int i = 0; i < num_images_; i++) {
    cv::Mat image = this->GetImage(i);
    std::vector<cv::KeyPoint> kps = detector_->DetectFeatures(image);
    cv::Mat descriptor_mat = descriptor_->ExtractDescriptors(image, kps);
    features.push_back(descriptor_mat);
  }
  // branching factor and depth levels
  const int k = 9;
  const int L = 3;
  DBoW3::Vocabulary voc(k, L, DBoW3::TF_IDF, DBoW3::L1_NORM);
  voc.create(features);
  this->SetVocabulary(voc);
  // copy old json image_db
  nlohmann::json image_db_copy = image_db_;
  // clear image_db
  image_db_.clear();
  // clear dbow_db
  bow_db_->clear();
  // loop through every image in old image_db and add to cleared database
  int num_images_copy = num_images_;
  num_images_ = 0;
  for (unsigned int i = 0; i < num_images_copy; i++) {
    // get image and pose
    std::string idx_str = std::to_string(i);
    std::string image_path = image_db_copy[idx_str]["path"];
    cv::Mat img = cv::imread(image_path);
    std::vector<int> pose_vec = image_db_[idx_str]["pose"];
    Eigen::Matrix4d pose;
    pose << pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4],
        pose_vec[5], pose_vec[6], pose_vec[7], pose_vec[8], pose_vec[9],
        pose_vec[10], pose_vec[11], pose_vec[12], pose_vec[13], pose_vec[14],
        pose_vec[15];
    // add image, pose and path
    this->AddImage(img, pose, image_path);
  }
}

void ImageDatabase::SetVocabulary(DBoW3::Vocabulary voc) {
  this->bow_db_->setVocabulary(voc);
}

std::vector<unsigned int> ImageDatabase::QueryDatabase(cv::Mat query_image,
                                                       int N) {
  std::vector<cv::KeyPoint> kps = detector_->DetectFeatures(query_image);
  cv::Mat features = descriptor_->ExtractDescriptors(query_image, kps);
  DBoW3::QueryResults results;
  this->bow_db_->query(features, results, N);

  std::vector<unsigned int> indices;
  for (unsigned int i = 0; i < results.size(); i++) {
    DBoW3::Result res = results[i];
    indices.push_back(res.Id);
  }
  return indices;
}

void ImageDatabase::AddImage(cv::Mat image, Eigen::Matrix4d pose,
                             std::string path_to_image) {
  std::vector<cv::KeyPoint> kps = detector_->DetectFeatures(image);
  cv::Mat features = descriptor_->ExtractDescriptors(image, kps);
  unsigned int idx = this->bow_db_->add(features);

  std::string idx_string = std::to_string(idx);
  image_db_[idx_string] = {
      {"pose",
       {pose(0, 0), pose(0, 1), pose(0, 2), pose(0, 3), pose(1, 0), pose(1, 1),
        pose(1, 2), pose(1, 3), pose(2, 0), pose(2, 1), pose(2, 2), pose(2, 3),
        pose(3, 0), pose(3, 1), pose(3, 2), pose(3, 3)}},
      {"path", path_to_image}};
  num_images_++;
}

void ImageDatabase::AddImage(cv::Mat image, Eigen::Matrix4d pose) {
  std::vector<cv::KeyPoint> kps = detector_->DetectFeatures(image);
  cv::Mat features = descriptor_->ExtractDescriptors(image, kps);
  unsigned int idx = this->bow_db_->add(features);

  std::string idx_string = std::to_string(idx);
  std::string path_to_image = image_folder_ + "image" + idx_string;
  image_db_[idx_string] = {
      {"pose",
       {pose(0, 0), pose(0, 1), pose(0, 2), pose(0, 3), pose(1, 0), pose(1, 1),
        pose(1, 2), pose(1, 3), pose(2, 0), pose(2, 1), pose(2, 2), pose(2, 3),
        pose(3, 0), pose(3, 1), pose(3, 2), pose(3, 3)}},
      {"path", path_to_image}};
  num_images_++;
}

cv::Mat ImageDatabase::GetImage(unsigned int index) {
  std::string idx_str = std::to_string(index);
  std::string image_path = image_db_[idx_str]["path"];
  cv::Mat img = cv::imread(image_path);
  return img;
}

Eigen::Matrix4d ImageDatabase::GetPose(unsigned int index) {
  std::string idx_str = std::to_string(index);
  std::vector<int> pose_vec = image_db_[idx_str]["pose"];
  Eigen::Matrix4d pose;
  pose << pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4],
      pose_vec[5], pose_vec[6], pose_vec[7], pose_vec[8], pose_vec[9],
      pose_vec[10], pose_vec[11], pose_vec[12], pose_vec[13], pose_vec[14],
      pose_vec[15];
  return pose;
}

} // namespace relocalization
