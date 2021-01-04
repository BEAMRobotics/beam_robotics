#include <boost/filesystem.hpp>

#include <ImageSimilaritySearch.h>
#include <beam_utils/filesystem.hpp>

namespace relocalization {

ImageSimilaritySearch::ImageSimilaritySearch(
    const ImageSimilaritySearch::Inputs& inputs)
    : inputs_(inputs) {}

void ImageSimilaritySearch::Run() {
  // create detector
  GetFeatureDetectorDescriptor();

  // create database
  std::string database_save_path = "/tmp/ImageSimilaritySearch/";
  boost::filesystem::create_directory(
      boost::filesystem::path(database_save_path));
  ImageDatabase db(inputs_.vocabulary_path, descriptor_, detector_,
                   database_save_path);

  // get all database images
  std::vector<std::string> db_image_files =
      beam::GetFiles(inputs_.db_imgs_directory, inputs_.file_extension,
                     inputs_.search_db_imgs_recursively);
  BEAM_INFO("Found {} database images in directory {}", db_image_files.size(),
            inputs_.db_imgs_directory);

  // add each image to the database
  BEAM_INFO("Adding images to database...");
  Eigen::Matrix4d identity_pose;
  identity_pose.setIdentity();
  bool first_image = true;
  int counter = 0;
  for (std::string filename : db_image_files) {
    // read and convert image
    cv::Mat image = imread(filename, cv::IMREAD_COLOR);
    cv::cvtColor(image, image, CV_BGR2GRAY);

    // reset superpoint model with image dimensions
    if (inputs_.feature_type == "SUPERPOINT" && first_image) {
      first_image = false;
      superpoint_params_.grid_size = image.rows / 5;
      detector_ = std::make_shared<beam_cv::SuperPointDetector>(
        model_, superpoint_params_);
      db.SetDetector(detector_);  
    }

    // add image
    db.AddImage(image, identity_pose, inputs_.camera_model_config_path,
                std::string(), true);
    counter++;
  }
  BEAM_INFO("Done adding images to database.");

  if (inputs_.retrain_vocabulary) {
    BEAM_INFO("Retraining vocabulary with database images...");
    db.RetrainVocabulary();
    BEAM_INFO("Done retraining vocabulary. Saving to: {}",
              database_save_path + "vocab.dbow3");
    db.SaveVocabulary();
  }

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

// void TestFunction(cv::Mat img, )

void ImageSimilaritySearch::SaveImages(ImageDatabase& database,
                                       const std::vector<unsigned int>& images,
                                       const std::string& query_file_path) {
  boost::filesystem::path query_path(query_file_path);
  std::string save_dir = query_file_path;
  save_dir.erase(save_dir.end() - query_path.extension().string().size(),
                 save_dir.end());
  boost::filesystem::create_directory(boost::filesystem::path(save_dir));
  BEAM_INFO("Storing {} similar images to {}", images.size(), save_dir + "/");

  ///////////////////////////////////////////////////////////////////////////////
  cv::Mat query_image = imread(query_file_path, cv::IMREAD_COLOR);
  cv::cvtColor(query_image, query_image, CV_BGR2GRAY);
  std::vector<cv::KeyPoint> kps_query = detector_->DetectFeatures(query_image);
  cv::Mat descriptors_query_tmp =
      descriptor_->ExtractDescriptors(query_image, kps_query);
  cv::Mat query_image_w_keypoints;
  cv::drawKeypoints(query_image, kps_query, query_image_w_keypoints,
                    cv::Scalar::all(-1),
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  std::string path_to_image = save_dir + "/IMG_QUERY.jpg";
  cv::imwrite(path_to_image, query_image_w_keypoints);
  /////////////////////////////////////////////////////////////////////////////////

  int counter = 0;
  for (unsigned int n : images) {
    counter++;
    cv::Mat queried_image = database.GetImage(n);

    ///////////////////////////////////////////////////////////////////////////////
    std::vector<cv::KeyPoint> kps_queried =
        detector_->DetectFeatures(queried_image);
    cv::Mat descriptors_queried_tmp =
        descriptor_->ExtractDescriptors(queried_image, kps_queried);
    cv::Mat queried_image_w_keypoints;
    cv::drawKeypoints(queried_image, kps_queried, queried_image_w_keypoints,
                      cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    std::string path_to_image = save_dir + "/IMG_" + std::to_string(counter) +
                                "_" + std::to_string(n) + "_keypoints" +
                                inputs_.file_extension;
    cv::imwrite(path_to_image, queried_image_w_keypoints);
    /////////////////////////////////////////////////////////////////////////////////

    std::string new_filename = save_dir + "/IMG_" + std::to_string(counter) +
                               "_" + std::to_string(n) + inputs_.file_extension;
    cv::imwrite(new_filename, queried_image);
  }
}

void ImageSimilaritySearch::GetFeatureDetectorDescriptor() {
  if (inputs_.feature_type == "SIFT") {
    detector_ = std::make_shared<beam_cv::SIFTDetector>(inputs_.num_features);
    descriptor_ =
        std::make_shared<beam_cv::SIFTDescriptor>(inputs_.num_features);
  } else if (inputs_.feature_type == "ORB") {
    detector_ = std::make_shared<beam_cv::ORBDetector>(inputs_.num_features);
    descriptor_ = std::make_shared<beam_cv::ORBDescriptor>();
  } else if (inputs_.feature_type == "FAST-BRISK") {
    detector_ = std::make_shared<beam_cv::FASTDetector>(inputs_.num_features);
    descriptor_ = std::make_shared<beam_cv::BRISKDescriptor>();
  } else if (inputs_.feature_type == "SUPERPOINT") {
    if (inputs_.feature_model_path.empty()) {
      BEAM_ERROR(
          "Input model_path cannot be empty for feature_type: SUPERPOINT.");
      throw std::invalid_argument{"Input feature_model_path cannot be empty."};
    }
    model_ =
        std::make_shared<beam_cv::SuperPointModel>(inputs_.feature_model_path);
    superpoint_params_.max_features = inputs_.num_features;
    detector_ = std::make_shared<beam_cv::SuperPointDetector>(
        model_, superpoint_params_);
    descriptor_ = std::make_shared<beam_cv::SuperPointDescriptor>(model_);
  } else {
    BEAM_ERROR("Invalid feature type. Input: {}, Options: SIFT, ORB, "
               "FAST-BRISK, SUPERPOINT",
               inputs_.feature_type);
    throw std::invalid_argument{"Invalid feature type."};
  }
}

} // namespace relocalization