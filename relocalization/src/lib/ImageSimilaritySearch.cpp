#include <boost/filesystem.hpp>

#include <ImageSimilaritySearch.h>
#include <beam_utils/filesystem.hpp>

namespace relocalization {

ImageSimilaritySearch::ImageSimilaritySearch(
    const ImageSimilaritySearch::Inputs& inputs)
    : inputs_(inputs) {}

void ImageSimilaritySearch::Run() {
  // create detector
  std::shared_ptr<beam_cv::Detector> detector;
  std::shared_ptr<beam_cv::Descriptor> descriptor;
  GetFeatureDetectorDescriptor(descriptor, detector);

  // create database
  ImageDatabase db(inputs_.vocabulary_path, descriptor, detector);

  // get all database images
  std::vector<std::string> db_image_files =
      beam::GetFiles(inputs_.db_imgs_directory, inputs_.file_extension,
                     inputs_.search_db_imgs_recursively);
  BEAM_INFO("Found {} query images in directory {}", db_image_files.size(),
            inputs_.db_imgs_directory);

  // add each image to the database
  BEAM_INFO("Adding images to database...");
  Eigen::Matrix4d identity_pose;
  identity_pose.setIdentity();
  std::string db_directory = "/tmp/ImageSimilaritySearch/";
  boost::filesystem::create_directory(boost::filesystem::path(db_directory));
  int counter = 0;
  for (std::string filename : db_image_files) {
    // read and convert image
    cv::Mat image = imread(filename, cv::IMREAD_COLOR);
    cv::cvtColor(image, image, CV_BGR2GRAY);
    std::string save_path = db_directory + "Image_" + std::to_string(counter) +
                            inputs_.file_extension;
    db.AddImage(image, identity_pose, inputs_.camera_model_config_path,
                save_path, true);
    counter++;
  }
  BEAM_INFO("Done adding images to database.");

  // get query images
  std::vector<std::string> query_image_files =
      beam::GetFiles(inputs_.query_imgs_directory, inputs_.file_extension,
                     inputs_.search_query_imgs_recursively);
  BEAM_INFO("Found {} query images in directory {}", query_image_files.size(),
            inputs_.query_imgs_directory);

  // find similar images
  for (std::string filename : query_image_files) {
    cv::Mat image = imread(filename, cv::IMREAD_COLOR);
    cv::cvtColor(image, image, CV_BGR2GRAY);
    std::vector<unsigned int> similar_images =
        db.QueryDatabase(image, inputs_.num_similar_imgs);
    SaveImages(db, similar_images, filename);
  }
}

void ImageSimilaritySearch::SaveImages(ImageDatabase& database,
                                       const std::vector<unsigned int>& images,
                                       const std::string& query_file_path) {
  boost::filesystem::path query_path(query_file_path);
  std::string save_dir = query_file_path;
  save_dir.erase(save_dir.end() - query_path.extension().string().size(),
                 save_dir.end());
  boost::filesystem::create_directory(boost::filesystem::path(save_dir));

  int counter = 0;
  for (unsigned int n : images) {
    counter++;
    std::string new_filename = save_dir + "/IMG_" + std::to_string(counter) +
                               "_" + std::to_string(n) + inputs_.file_extension;
    cv::Mat query_image = database.GetImage(n);
    cv::imwrite(new_filename, query_image);
  }
}

// TODO: Move this to beam_cv
void ImageSimilaritySearch::GetFeatureDetectorDescriptor(
    std::shared_ptr<beam_cv::Descriptor>& descriptor,
    std::shared_ptr<beam_cv::Detector>& detector) {
  if (inputs_.feature_type == "SIFT") {
    detector = std::make_shared<beam_cv::SIFTDetector>(inputs_.num_features);
    descriptor =
        std::make_shared<beam_cv::SIFTDescriptor>(inputs_.num_features);
  } else if (inputs_.feature_type == "ORB") {
    detector = std::make_shared<beam_cv::ORBDetector>(inputs_.num_features);
    descriptor = std::make_shared<beam_cv::ORBDescriptor>();
  } else if (inputs_.feature_type == "FAST-BRISK") {
    detector = std::make_shared<beam_cv::FASTDetector>(inputs_.num_features);
    descriptor = std::make_shared<beam_cv::BRISKDescriptor>();
  } else if (inputs_.feature_type == "SUPERPOINT") {
    if (inputs_.feature_model_path.empty()) {
      BEAM_ERROR(
          "Input model_path cannot be empty for feature_type: SUPERPOINT.");
      throw std::invalid_argument{"Input feature_model_path cannot be empty."};
    }
    std::shared_ptr<beam_cv::SuperPointModel> model =
        std::make_shared<beam_cv::SuperPointModel>(inputs_.feature_model_path);
    detector = std::make_shared<beam_cv::SuperPointDetector>(
        model, inputs_.num_features);
    descriptor = std::make_shared<beam_cv::SuperPointDescriptor>(model);
  } else {
    BEAM_ERROR("Invalid feature type. Input: {}, Options: SIFT, ORB, "
               "FAST-BRISK, SUPERPOINT",
               inputs_.feature_type);
    throw std::invalid_argument{"Invalid feature type."};
  }
}

} // namespace relocalization