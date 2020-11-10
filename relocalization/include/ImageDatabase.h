#pragma once

#include <DBoW3/DBoW3.h>
#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include <beam_cv/descriptors/Descriptor.h>
#include <beam_cv/detectors/Detector.h>

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
  ImageDatabase();

  /**
   * @brief Constructor to initialize with already gathered dbow dv and image db
   * @param database_config path to config file: use only if existing dbow
   * database and image database have been made
   */
  ImageDatabase(std::string database_config);

  /**
   * @brief Constructor to initialize empty database with given vocabulary
   * @param voc specifiec
   */
  ImageDatabase(std::string vocab_location,
                std::shared_ptr<beam_cv::Descriptor> descriptor,
                std::shared_ptr<beam_cv::Detector> detector);

  /**
   * @brief Default destructor
   */
  ~ImageDatabase() = default;

  /**
   * @brief Load a database from 2 file
   * @param dbow_file_path path to dbow db
   * @param imagedb_file_path path to imagedb
   */
  void LoadDatabase(std::string dbow_file_path, std::string imagedb_file_path);

  /**
   * @brief Save current database to files
   * @param dbow_file_path path to save dbow db
   * @param imagedb_file_path path to save imagedb
   */
  void SaveDatabase(std::string dbow_file_path, std::string imagedb_file_path);

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
   * @brief Retrain vocabulary given just the images in image_db_
   * the dbow_db_ will have to be refilled after retraining
   */
  void RetrainVocabulary();

  /**
   * @brief Set vocabulary to new vocab, database will need to be refilled
   * @param voc DBoW vocabulary to set
   */
  void SetVocabulary(DBoW3::Vocabulary voc);

  /**
   * @brief Return list of N image id's best matching query image
   * @param query_image to query database with
   */
  std::vector<unsigned int> QueryDatabase(cv::Mat query_image, int N);

  /**
   * @brief Add an image to the dbow database, add its pose and path to image
   * database
   * @param image to add
   * @param pose of image in map
   * @param path to image
   */
  void AddImage(cv::Mat image, Eigen::Matrix4d pose, std::string path_to_image);

  /**
   * @brief Add an image to the dbow database, add its pose to image database,
   * save image in default location and add that as path to image db
   * @param image to add
   * @param pose of image in map
   */
  void AddImage(cv::Mat image, Eigen::Matrix4d pose);

private:
  std::shared_ptr<DBoW3::Database> bow_db_;
  json image_db_;
  std::shared_ptr<beam_cv::Descriptor> descriptor_;
  std::shared_ptr<beam_cv::Detector> detector_;
  std::string image_folder_;

  std::map<std::string, DescriptorType> descriptor_types_ = {
      {"ORB", DescriptorType::ORB},
      {"SIFT", DescriptorType::SIFT},
      {"BRISK", DescriptorType::BRISK}};
  std::map<std::string, DetectorType> detector_types_ = {
      {"ORB", DetectorType::ORB},
      {"SIFT", DetectorType::SIFT},
      {"FAST", DetectorType::FAST}};
};

} // namespace relocalization
