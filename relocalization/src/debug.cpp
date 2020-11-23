#include <ImageDatabase.h>
#include <ImageToImage.h>

#include <beam_cv/descriptors/ORBDescriptor.h>
#include <beam_cv/detectors/ORBDetector.h>
#include <beam_cv/matchers/FLANNMatcher.h>

int main() {
  std::string db_path = "/home/jake/data/imageDB/";
  std::shared_ptr<relocalization::ImageDatabase> id =
      std::make_shared<relocalization::ImageDatabase>(db_path);
  cv::Mat test = cv::imread("/home/jake/data/imageDB/test_83.jpg");

  std::shared_ptr<beam_cv::Descriptor> desc =
      std::make_shared<beam_cv::ORBDescriptor>();
  std::shared_ptr<beam_cv::Detector> det =
      std::make_shared<beam_cv::ORBDetector>();
  std::shared_ptr<beam_cv::Matcher> mat =
      std::make_shared<beam_cv::FLANNMatcher>();

  relocalization::ImageToImage matcher(id, det, desc, mat);
  std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector3d>> correspondences =
      matcher.Query(test);
}