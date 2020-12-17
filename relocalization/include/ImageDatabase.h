#pragma once

#include <typeinfo>

#include <DBoW3/DBoW3.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include <beam_calibration/CameraModel.h>
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>

namespace relocalization {

// enum to represent types of descriptors available
enum class DescriptorType { ORB = 0, BRISK, SIFT };
// enum to represent types of detectors available
enum class DetectorType { ORB = 0, SIFT, FAST };

using json = nlohmann::json;

/**
 * @brief class for image database
 */
class ImageDatabase {
public:
  /**
   * @brief Default constructor
   */
  ImageDatabase() = default;

  /**
   * @brief Constructor to initialize with already gathered dbow dv and image db
   * @param database_path path to database folder
   */
  ImageDatabase(const std::string& database_path,
                const std::string& image_folder = std::string());

  /**
   * @brief Constructor to initialize empty database with given vocabulary
   * @param voc specified
   */
  ImageDatabase(std::string vocab_location,
                std::shared_ptr<beam_cv::Descriptor> descriptor,
                std::shared_ptr<beam_cv::Detector> detector,
                const std::string& database_path = std::string(),
                const std::string& image_folder = std::string());

  /**
   * @brief Default destructor
   */
  ~ImageDatabase() = default;

  /**
   * @brief Save current database to default folder
   */
  void SaveDatabase();

  void SaveVocabulary();

  /**
   * @brief Set the type of keypoint descriptor to use
   * @param type of descriptor
   */
  void SetDescriptor(std::shared_ptr<beam_cv::Descriptor> descriptor);

  /**
   * @brief Set the type of image keypoint detector
   * @param type of detector
   */
  void SetDetector(std::shared_ptr<beam_cv::Detector> detector);

  /**
   * @brief Set vocabulary to new vocab, database will need to be refilled
   * @param voc DBoW vocabulary to set
   */
  void SetVocabulary(DBoW3::Vocabulary voc);

  /**
   * @brief Retrain vocabulary given just the images in image_db_
   * the dbow_db_ will have to be refilled after retraining
   */
  void RetrainVocabulary();

  /**
   * @brief Return list of N image id's best matching query image
   * @param query_image to query database with
   */
  std::vector<unsigned int> QueryDatabase(cv::Mat query_image, int N = 2);

  /**
   * @brief Add an image to the dbow database, add its pose and id to image db,
   * its model to model db
   * @param image to add
   * @param pose of image in map
   * @param path_to_model path to camera model for image
   */
  unsigned int AddImage(cv::Mat image, Eigen::Matrix4d pose,
                        std::string camera_model_files,
                        const std::string& image_path = std::string(),
                        bool write = true);

  /**
   * @brief Get image object given an index
   * @param index to fetch
   */
  cv::Mat GetImage(unsigned int index);

  /**
   * @brief Get pose of image given index
   * @param index to fetch
   */
  Eigen::Matrix4d GetPose(unsigned int index);

  /**
   * @brief Get pose of image given index
   * @param index to fetch
   */
  std::shared_ptr<beam_calibration::CameraModel>
      GetCameraModel(unsigned int index);

protected:
  /**
   * @brief Get descriptor
   * @param string of type
   */
  std::shared_ptr<beam_cv::Descriptor> DetermineDescriptor(std::string type);

  /**
   * @brief Get detector based on type
   * @param string of type
   */
  std::shared_ptr<beam_cv::Detector> DetermineDetector(std::string type);

  /**
   * @brief Get id of camera by path, will init new camera model if it doesnt
   * exist
   * @param camera_model_file file to camera model
   */
  unsigned int GetCameraID(std::string camera_model_file);

private:
  unsigned int camera_id_ = 0;
  size_t num_images_ = 0;
  // objects storing bow db and images/poses
  std::shared_ptr<DBoW3::Database> bow_db_;
  json image_db_;
  // detector and descritpor to use
  std::shared_ptr<beam_cv::Descriptor> descriptor_;
  std::shared_ptr<beam_cv::Detector> detector_;
  // stores path to database
  std::string database_folder_;
  // stores path to image folder if not named images or is outside of database
  std::string image_folder_;
  // maps to use for descriptor and detector verification from json files
  std::map<std::string, DescriptorType> descriptor_types_ = {
      {"ORB", DescriptorType::ORB},
      {"SIFT", DescriptorType::SIFT},
      {"BRISK", DescriptorType::BRISK}};
  std::map<std::string, DetectorType> detector_types_ = {
      {"ORB", DetectorType::ORB},
      {"SIFT", DetectorType::SIFT},
      {"FAST", DetectorType::FAST}};
  std::map<std::string, std::string> descriptor_repr_ = {
      {std::string(typeid(beam_cv::ORBDescriptor()).name()), "ORB"},
      {std::string(typeid(beam_cv::SIFTDescriptor()).name()), "SIFT"},
      {std::string(typeid(beam_cv::BRISKDescriptor()).name()), "BRISK"}};
  std::map<std::string, std::string> detector_repr_ = {
      {std::string(typeid(beam_cv::ORBDetector()).name()), "ORB"},
      {std::string(typeid(beam_cv::SIFTDetector()).name()), "SIFT"},
      {std::string(typeid(beam_cv::FASTDetector()).name()), "FAST"}};
  // map to store camera model depending on the path to its config file
  std::unordered_map<std::string,
                     std::shared_ptr<beam_calibration::CameraModel>>
      cam_model_path_map_;
  std::unordered_map<std::string, int> cam_model_id_map_;
};

} // namespace relocalization
