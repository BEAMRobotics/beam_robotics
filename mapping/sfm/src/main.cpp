
#include <sfm/ExtractImageFeatures.h>

#include <gflags/gflags.h>

#include <beam_utils/gflags.h>

DEFINE_string(image_directory, "", "Full path to bag file (Required).");
DEFINE_validator(image_directory, &beam::gflags::ValidateDirMustExist);
DEFINE_string(output_directory, "", "Full path to poses file (Required).");
DEFINE_validator(output_directory, &beam::gflags::ValidateDirMustExist);
DEFINE_bool(search_recursively, false,
            "Set to true to search for images recursively in image_directory");
DEFINE_string(file_extension, ".jpg", "File extension for images");
DEFINE_string(
    feature_type, "SIFT",
    "Feature extractor type. Options: SIFT, ORB, FAST-BRISK, SUPERPOINT");
DEFINE_string(model_path, "",
              "Path to model used for learned features. This is only currently "
              "required for SUPERPOINT");
DEFINE_int32(num_features, 500,
             "Maximum number of features to extract per image.");
DEFINE_double(downsize_image, 0,
              "Fraction value (o to 1) to downsize the image before extracting "
              "features. This will also save the new image to "
              "output_directory. Setting to zero will not downsize.");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  sfm::ExtractImageFeatures ext(FLAGS_image_directory, FLAGS_output_directory,
                                FLAGS_search_recursively, FLAGS_file_extension,
                                FLAGS_feature_type, FLAGS_model_path,
                                FLAGS_num_features, FLAGS_downsize_image);
  ext.ExtractFeatures();

  return 0;
}