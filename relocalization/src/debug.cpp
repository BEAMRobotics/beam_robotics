#include <ImageDatabase.h>
#include <ImageToImage.h>

#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/matchers/FLANNMatcher.h>

int main() {
  std::string db_path = "/home/jake/Downloads/Extended_CMU_Seasons/slice3/";
  std::shared_ptr<relocalization::ImageDatabase> id =
      std::make_shared<relocalization::ImageDatabase>(db_path,
                                                      db_path + "database");

  std::shared_ptr<beam_cv::Descriptor> desc =
      std::make_shared<beam_cv::SIFTDescriptor>();
  std::shared_ptr<beam_cv::Detector> det =
      std::make_shared<beam_cv::SIFTDetector>();
  std::shared_ptr<beam_cv::Matcher> mat =
      std::make_shared<beam_cv::FLANNMatcher>(beam_cv::FLANN::KDTree, 0.8,
                                              false, true, cv::FM_RANSAC, 5);

  cv::Mat test = cv::imread("/home/jake/test.jpg");
  std::vector<cv::KeyPoint> test_keypoint = det->DetectFeatures(test);
  cv::Mat test_descriptor = desc->ExtractDescriptors(test, test_keypoint);

  std::vector<unsigned int> matches = id->QueryDatabase(test, 10);
  for (int i = 0; i < 10; i++) {
    cv::Mat match = id->GetImage(matches[i]);
    std::vector<cv::KeyPoint> match_keypoint = det->DetectFeatures(match);
    cv::Mat match_descriptor = desc->ExtractDescriptors(match, match_keypoint);
    std::vector<cv::DMatch> matches = mat->MatchDescriptors(
        test_descriptor, match_descriptor, test_keypoint, match_keypoint);
    std::string path = "/home/jake/match" + std::to_string(i) + ".jpg";
    cv::imwrite(path, match);
    std::cout << path << ";" << matches.size() << std::endl;
  }

  relocalization::ImageToImage matcher(id, det, desc, mat);
  std::vector<std::tuple<Eigen::Vector2i, Eigen::Vector3d>> correspondences =
      matcher.Query(test);
  std::cout << correspondences.size() << std::endl;
}