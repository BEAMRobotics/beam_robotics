#include <ImageDatabase.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <pwd.h>
#include <sys/types.h>
#include <unistd.h>

#include <boost/filesystem.hpp>

#include <beam_cv/Utils.h>

namespace relocalization {

ImageDatabase::ImageDatabase(const std::string& database_path,
                             const std::string& image_folder) {
  database_folder_ = database_path;
  if (image_folder.empty()) {
    image_folder_ = database_folder_ + "images/";
  } else {
    image_folder_ = image_folder;
  }
  // load DBoW3 database into bow db object
  std::string DBoW3_file_path = database_path + "bow_db.dbow3";
  BEAM_INFO("Loading DBOW3 database file: {}", DBoW3_file_path);
  this->bow_db_ = std::make_shared<DBoW3::Database>(DBoW3_file_path);
  this->num_images_ = this->bow_db_->size();
  // load image db into image db json object
  std::string imagedb_file_path = database_path + "image_db.json";
  BEAM_INFO("Loading image database file: {}", imagedb_file_path);
  std::ifstream image_db_file(imagedb_file_path);
  image_db_ = json::object();
  image_db_file >> image_db_;
  // determine descriptor to use
  std::string descriptor_t = image_db_["descriptor"];
  this->descriptor_ = this->DetermineDescriptor(descriptor_t);
  // determine the detector to use
  std::string detector_t = image_db_["detector"];
  this->detector_ = this->DetermineDetector(detector_t);
  // load camera models
  BEAM_INFO("Initializing camera models.");
  for (std::string path : image_db_["cameras"]) {
    cam_model_path_map_[path] = beam_calibration::CameraModel::Create(path);
    cam_model_id_map_[path] = camera_id_;
    camera_id_++;
  }
}

ImageDatabase::ImageDatabase(std::string vocab_location,
                             std::shared_ptr<beam_cv::Descriptor> descriptor,
                             std::shared_ptr<beam_cv::Detector> detector,
                             const std::string& database_path,
                             const std::string& image_folder) {
  if (database_path.empty()) {
    // build database folder location
    auto time = std::chrono::system_clock::now();
    std::time_t date = std::chrono::system_clock::to_time_t(time);
    std::string cur_date = std::string(std::ctime(&date));
    std::replace(cur_date.begin(), cur_date.end(), ' ', '_');
    std::replace(cur_date.begin(), cur_date.end(), '\n', '_');
    struct passwd* pw = getpwuid(getuid());
    std::string HOME = pw->pw_dir;
    database_folder_ = HOME + "/" + cur_date + "/";
  } else {
    // set db location to param
    database_folder_ = database_path;
  }
  boost::filesystem::create_directory(database_folder_);
  if (image_folder.empty()) {
    image_folder_ = database_folder_ + "images/";
  } else {
    image_folder_ = image_folder;
  }
  boost::filesystem::create_directory(image_folder_);
  // load parameters
  DBoW3::Vocabulary vocab(vocab_location);
  this->bow_db_ = std::make_shared<DBoW3::Database>(vocab);
  this->SetDescriptor(descriptor);
  this->SetDetector(detector);
}

void ImageDatabase::SaveDatabase() {
  // save DBoW3 database
  std::string DBoW3_file_path = database_folder_ + "/bow_db.dbow3";
  this->bow_db_->save(DBoW3_file_path);
  // write image db to file
  std::string imagedb_file_path = database_folder_ + "/image_db.json";
  std::ofstream image_db_file_out(imagedb_file_path);
  image_db_file_out << std::setw(4) << image_db_ << std::endl;
}

void ImageDatabase::SaveVocabulary() {
  std::string file_path = database_folder_ + "/vocab.dbow3";
  this->bow_db_->getVocabulary()->save(file_path, true);
}

void ImageDatabase::SetDescriptor(
    std::shared_ptr<beam_cv::Descriptor> descriptor) {
  this->descriptor_ = descriptor;
  std::string type =
      "F" + std::string(typeid(*this->descriptor_).name()) + "vE";
  std::string repr = descriptor_repr_[type];
  image_db_["descriptor"] = repr;
}

void ImageDatabase::SetDetector(std::shared_ptr<beam_cv::Detector> detector) {
  this->detector_ = detector;
  std::string type = "F" + std::string(typeid(*this->detector_).name()) + "vE";
  std::string repr = detector_repr_[type];
  image_db_["detector"] = repr;
}

void ImageDatabase::SetVocabulary(DBoW3::Vocabulary voc) {
  this->bow_db_->setVocabulary(voc);
  // copy old json image_db
  nlohmann::json image_db_copy = image_db_;
  // clear image_db
  image_db_.clear();
  // clear dbow_db
  bow_db_->clear();
  // clear camera model id and path map
  cam_model_path_map_.clear();
  cam_model_id_map_.clear();
  // loop through every image in old image_db and add to cleared database
  int num_images_copy = num_images_;
  camera_id_ = 0;
  num_images_ = 0;
  for (unsigned int i = 0; i < num_images_copy; i++) {
    // get image and pose
    std::string image_file = image_db_copy["images"][i]["image"];
    std::string image_path = image_folder_ + image_file;
    cv::Mat img = cv::imread(image_path);
    std::vector<double> pose_vec = image_db_copy["images"][i]["pose"];

    Eigen::Matrix4d pose;
    pose << pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4],
        pose_vec[5], pose_vec[6], pose_vec[7], pose_vec[8], pose_vec[9],
        pose_vec[10], pose_vec[11], pose_vec[12], pose_vec[13], pose_vec[14],
        pose_vec[15];
    // get path to cam model
    unsigned int cam_id = image_db_copy["images"][i]["camera_model"];
    std::string path_to_cam = image_db_copy["cameras"][cam_id];
    // add image, pose and path, without rewriting the image
    this->AddImage(img, pose, path_to_cam, image_file, false);
  }
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
}

std::vector<unsigned int> ImageDatabase::QueryDatabase(cv::Mat query_image,
                                                       int N) {
  std::vector<cv::KeyPoint> kps = detector_->DetectFeatures(query_image);
  cv::Mat features = descriptor_->ExtractDescriptors(query_image, kps);
  DBoW3::QueryResults results;
  this->bow_db_->query(features, results, N);

  std::vector<uint> indices;
  for (unsigned int i = 0; i < results.size(); i++) {
    DBoW3::Result res = results[i];
    indices.push_back(res.Id);
  }
  return indices;
}

unsigned int ImageDatabase::AddImage(cv::Mat image, Eigen::Matrix4d pose,
                                     std::string camera_model_file,
                                     const std::string& image_path,
                                     bool write) {
  std::vector<cv::KeyPoint> kps = detector_->DetectFeatures(image);
  cv::Mat features = descriptor_->ExtractDescriptors(image, kps);
  unsigned int idx = this->bow_db_->add(features);
  unsigned int cam_id = GetCameraID(camera_model_file);
  // determine image file name
  std::string image_file;
  if (image_path.empty()) {
    std::string idx_string = std::to_string(idx);
    image_file = "image_" + idx_string + ".png";
  } else {
    boost::filesystem::path p(image_path);
    image_file = p.stem().string() + p.extension().string();
  }
  if (write) {
    // write image to images folder
    std::string path_to_image = image_folder_ + image_file;
    cv::imwrite(path_to_image, image);
  }
  // add image to image_db
  image_db_["images"][idx] = {
      {"pose",
       {pose(0, 0), pose(0, 1), pose(0, 2), pose(0, 3), pose(1, 0), pose(1, 1),
        pose(1, 2), pose(1, 3), pose(2, 0), pose(2, 1), pose(2, 2), pose(2, 3),
        pose(3, 0), pose(3, 1), pose(3, 2), pose(3, 3)}},
      {"image", image_file},
      {"camera_model", cam_id}};
  num_images_++;
  return idx;
}

cv::Mat ImageDatabase::GetImage(unsigned int index) {
  std::string image_file = image_db_["images"][index]["image"];
  std::string image_path = image_folder_ + "/" + image_file;
  cv::Mat img = cv::imread(image_path);
  return img;
}

Eigen::Matrix4d ImageDatabase::GetPose(unsigned int index) {
  std::vector<double> pose_vec = image_db_["images"][index]["pose"];
  Eigen::Matrix4d pose;
  pose << pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4],
      pose_vec[5], pose_vec[6], pose_vec[7], pose_vec[8], pose_vec[9],
      pose_vec[10], pose_vec[11], pose_vec[12], pose_vec[13], pose_vec[14],
      pose_vec[15];
  return pose;
}

std::shared_ptr<beam_calibration::CameraModel>
    ImageDatabase::GetCameraModel(unsigned int index) {
  unsigned int cam_id = image_db_["images"][index]["camera_model"];
  std::string path = image_db_["cameras"][cam_id];
  return cam_model_path_map_[path];
}

std::shared_ptr<beam_cv::Descriptor>
    ImageDatabase::DetermineDescriptor(std::string type) {
  if (descriptor_types_[type] == DescriptorType::ORB) {
    return std::make_shared<beam_cv::ORBDescriptor>();
  } else if (descriptor_types_[type] == DescriptorType::SIFT) {
    return std::make_shared<beam_cv::SIFTDescriptor>();
  } else if (descriptor_types_[type] == DescriptorType::BRISK) {
    return std::make_shared<beam_cv::BRISKDescriptor>();
  }
  BEAM_CRITICAL("Invalid descriptor type, type must match loaded database and "
                "vocabulary, valid types: ORB, SIFT, BRISK.");
  throw std::runtime_error{
      "Invalid descriptor type, type must match loaded database and "
      "vocabulary."};
}

std::shared_ptr<beam_cv::Detector>
    ImageDatabase::DetermineDetector(std::string type) {
  if (detector_types_[type] == DetectorType::ORB) {
    return std::make_shared<beam_cv::ORBDetector>();
  } else if (detector_types_[type] == DetectorType::SIFT) {
    return std::make_shared<beam_cv::SIFTDetector>();
  } else if (detector_types_[type] == DetectorType::FAST) {
    return std::make_shared<beam_cv::FASTDetector>();
  }
  BEAM_CRITICAL("Invalid detector type, defaulting to FAST.");
  return std::make_shared<beam_cv::FASTDetector>();
}

unsigned int ImageDatabase::GetCameraID(std::string camera_model_file) {
  unsigned int cam_id;
  // if camera model doesnt exist
  if (cam_model_path_map_.find(camera_model_file) ==
      cam_model_path_map_.end()) {
    // create model
    std::shared_ptr<beam_calibration::CameraModel> model =
        beam_calibration::CameraModel::Create(camera_model_file);
    cam_model_path_map_[camera_model_file] = model;
    cam_model_id_map_[camera_model_file] = camera_id_;
    cam_id = camera_id_;
    camera_id_++;
    // store model path in image_db
    image_db_["cameras"][cam_id] = camera_model_file;
  } else {
    // if camera model does exist, get its id
    cam_id = cam_model_id_map_[camera_model_file];
  }
  return cam_id;
}

} // namespace relocalization
