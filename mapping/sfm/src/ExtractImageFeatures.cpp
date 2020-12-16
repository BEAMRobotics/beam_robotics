
#include <sfm/ExtractImageFeatures.h>

#include <fstream>
#include <sstream>

#include <boost/filesystem.hpp>

#include <beam_utils/filesystem.hpp>

namespace sfm {

ExtractImageFeatures::ExtractImageFeatures(
    const std::string& image_directory, const std::string& output_directory,
    bool search_recursively, const std::string& file_extension,
    const std::string& feature_type, const std::string& model_path,
    int num_features, double downsize_image)
    : image_directory_(image_directory),
      output_directory_(output_directory),
      search_recursively_(search_recursively),
      file_extension_(file_extension),
      feature_type_(feature_type),
      model_path_(model_path),
      num_features_(num_features),
      downsize_image_(downsize_image) {}

void ExtractImageFeatures::ExtractFeatures() {
  std::vector<std::string> image_files =
      beam::GetFiles(image_directory_, file_extension_, search_recursively_);
  BEAM_INFO("Found {} images in directory {}", image_files.size(),
            image_directory_);

  std::shared_ptr<beam_cv::Detector> detector;
  std::shared_ptr<beam_cv::Descriptor> descriptor;
  GetFeatureDetectorDescriptor(detector, descriptor);

  for (std::string image_file : image_files) {
    // read and convert image
    cv::Mat image = imread(image_file, cv::IMREAD_COLOR);
    cv::cvtColor(image, image, CV_BGR2GRAY);
    if (downsize_image_ != 0) {
      int new_rows = static_cast<int>(image.rows * downsize_image_);
      int new_cols = static_cast<int>(image.cols * downsize_image_);
      cv::resize(image, image, cv::Size(new_cols, new_rows));
    }

    // for superpoint, we want to re-init detectors/descriptors each tiem
    if (feature_type_ == "SUPERPOINT") {
      int grid_size = image.rows / 5;
      detector = std::make_shared<beam_cv::SuperPointDetector>(
          model_, num_features_, 0.3, 0, 3, grid_size, false);
      descriptor = std::make_shared<beam_cv::SuperPointDescriptor>(model_);
    }

    // get keypoints
    std::vector<cv::KeyPoint> keypoints = detector->DetectFeatures(image);
    cv::Mat descriptors = descriptor->ExtractDescriptors(image, keypoints);

    // save results and optionally image
    boost::filesystem::path fs(image_file);
    std::string filename = fs.filename().string();
    WriteToFile(keypoints, descriptors, output_directory_ + filename + ".txt");
    if (downsize_image_ != 0) {
      cv::imwrite(output_directory_ + filename, image);
    }
  }
}

void ExtractImageFeatures::GetFeatureDetectorDescriptor(
    std::shared_ptr<beam_cv::Detector>& detector,
    std::shared_ptr<beam_cv::Descriptor>& descriptor) {
  if (feature_type_ == "SIFT") {
    detector = std::make_shared<beam_cv::SIFTDetector>(num_features_);
    descriptor = std::make_shared<beam_cv::SIFTDescriptor>(num_features_);
  } else if (feature_type_ == "ORB") {
    detector = std::make_shared<beam_cv::ORBDetector>(num_features_);
    descriptor = std::make_shared<beam_cv::ORBDescriptor>();
  } else if (feature_type_ == "FAST-BRISK") {
    detector = std::make_shared<beam_cv::FASTDetector>(num_features_);
    descriptor = std::make_shared<beam_cv::BRISKDescriptor>();
  } else if (feature_type_ == "SUPERPOINT") {
    if (model_path_.empty()) {
      BEAM_ERROR(
          "Input model_path cannot be empty for feature_type: SUPERPOINT.");
      throw std::invalid_argument{"Input model_path cannot be empty."};
    }
    model_ = std::make_shared<beam_cv::SuperPointModel>(model_path_);
    detector =
        std::make_shared<beam_cv::SuperPointDetector>(model_, num_features_);
    descriptor = std::make_shared<beam_cv::SuperPointDescriptor>(model_);
  } else {
    BEAM_ERROR("Invalid feature type. Input: {}, Options: SIFT, ORB, "
               "FAST-BRISK, SUPERPOINT",
               feature_type_);
    throw std::invalid_argument{"Invalid feature type."};
  }
}

void ExtractImageFeatures::WriteToFile(
    const std::vector<cv::KeyPoint>& keypoints, const cv::Mat& descriptors,
    const std::string& filename) {
  std::ofstream file(filename);
  file << descriptors.rows << " " << descriptors.cols << "\n";
  for (int i = 0; i < descriptors.rows; i++) {
    file << keypoints[i].pt.x << " " << keypoints[i].pt.y << " "
         << keypoints[i].size << " " << keypoints[i].angle;
    // To avoid having to figure out the value type for each descriptor, this
    // is a bit of a hack:
    // 1. convert row to stringstream
    std::stringstream buffer;
    buffer << descriptors.row(i) << std::endl;

    // 2. convert to string of format: [val1, val2, ..., valN]
    std::string outputstring = buffer.str();

    // 3. erase unwanted characters
    outputstring.erase(
        std::remove(outputstring.begin(), outputstring.end(), ','),
        outputstring.end());
    outputstring.erase(outputstring.begin(), outputstring.begin() + 1);
    outputstring.erase(outputstring.end() - 2, outputstring.end());

    // 4. output to file
    file << " " << outputstring << "\n";
  }
  file.close();
}

} // namespace sfm