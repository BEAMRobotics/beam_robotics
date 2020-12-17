#pragma once

#include <DBoW3/DBoW3.h>
#include <opencv2/opencv.hpp>

#include <ImageDatabase.h>
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>

namespace relocalization {

/**
 * @brief TODO
 */
class ImageSimilaritySearch {
public:
  struct Inputs {
    std::string query_imgs_directory;
    std::string db_imgs_directory;
    std::string vocabulary_path;
    std::string output_directory;
    std::string file_extension;
    std::string feature_type;
    std::string feature_model_path;
    std::string camera_model_config_path;
    int num_similar_imgs;
    int num_features;
    bool search_query_imgs_recursively;
    bool search_db_imgs_recursively;
    bool retrain_vocabulary;
  };

  /**
   * @brief constructor
   */
  ImageSimilaritySearch(const ImageSimilaritySearch::Inputs& inputs);

  /**
   * @brief Default destructor
   */
  ~ImageSimilaritySearch() = default;

  void Run();

private:
  void GetFeatureDetectorDescriptor(
      std::shared_ptr<beam_cv::Descriptor>& descriptor,
      std::shared_ptr<beam_cv::Detector>& detector);

  void SaveImages(ImageDatabase& database,
                  const std::vector<unsigned int>& images,
                  const std::string& query_file_path);

  Inputs inputs_;
};

} // namespace relocalization
