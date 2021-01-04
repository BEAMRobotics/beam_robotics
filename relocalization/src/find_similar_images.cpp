
#include <gflags/gflags.h>

#include <ImageSimilaritySearch.h>
#include <beam_utils/gflags.h>

DEFINE_string(query_imgs_directory, "",
              "Path to directory containing the query images (REQUIRED)");
DEFINE_validator(query_imgs_directory, &beam::gflags::ValidateDirMustExist);
DEFINE_bool(search_query_imgs_recursively, false,
            "Set to true if you want to search for QUERY images recursively.");

DEFINE_string(db_imgs_directory, "",
              "Path to directory containing the database images (REQUIRED)");
DEFINE_validator(db_imgs_directory, &beam::gflags::ValidateDirMustExist);
DEFINE_bool(
    search_db_imgs_recursively, false,
    "Set to true if you want to search for DATABASE images recursively.");

DEFINE_string(vocabulary_path, "", "Path to dbows vocabulary tree (REQUIRED)");
DEFINE_validator(vocabulary_path, &beam::gflags::ValidateFileMustExist);
DEFINE_bool(retrain_vocabulary, false,
            "Setting to true will retrain the vocabulary with the input "
            "database images.");

DEFINE_string(camera_model_config_path, "",
              "Path to camera model config file. This is a .conf for ladybug "
              "camera or .json otherwise. (REQUIRED)");
DEFINE_validator(camera_model_config_path,
                 &beam::gflags::ValidateFileMustExist);

DEFINE_string(output_directory, "",
              "Path to output directory where top N similar images will be "
              "saved. Defaults to query_images_directory.");
DEFINE_int32(num_similar_imgs, 5, "Number of similar images to retrieve.");
DEFINE_string(file_extension, ".jpg", "File extension for images");

DEFINE_string(
    feature_type, "SIFT",
    "Feature extractor type. Options: SIFT, ORB, FAST-BRISK, SUPERPOINT");
DEFINE_string(feature_model_path, "",
              "Path to model used for learned features. This is only currently "
              "required for SUPERPOINT");
DEFINE_int32(num_features, 500,
             "Maximum number of features to extract per image.");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::string output_directory = FLAGS_output_directory;
  if (FLAGS_output_directory.empty()) {
    output_directory = FLAGS_query_imgs_directory;
  }

  relocalization::ImageSimilaritySearch::Inputs inputs{
      .query_imgs_directory = FLAGS_query_imgs_directory,
      .db_imgs_directory = FLAGS_db_imgs_directory,
      .vocabulary_path = FLAGS_vocabulary_path,
      .output_directory = output_directory,
      .file_extension = FLAGS_file_extension,
      .feature_type = FLAGS_feature_type,
      .feature_model_path = FLAGS_feature_model_path,
      .camera_model_config_path = FLAGS_camera_model_config_path,
      .num_similar_imgs = FLAGS_num_similar_imgs,
      .num_features = FLAGS_num_features,
      .search_query_imgs_recursively = FLAGS_search_query_imgs_recursively,
      .search_db_imgs_recursively = FLAGS_search_db_imgs_recursively,
      .retrain_vocabulary = FLAGS_retrain_vocabulary};

  relocalization::ImageSimilaritySearch search(inputs);
  search.Run();

  return 0;
}