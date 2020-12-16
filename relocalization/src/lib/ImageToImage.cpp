#include <ImageToImage.h>
#include <beam_cv/geometry/Triangulation.h>

namespace relocalization {

ImageToImage::ImageToImage(std::shared_ptr<relocalization::ImageDatabase>& db,
                           std::shared_ptr<beam_cv::Detector> detector,
                           std::shared_ptr<beam_cv::Descriptor> descriptor,
                           std::shared_ptr<beam_cv::Matcher> matcher) {
  this->detector_ = detector;
  this->descriptor_ = descriptor;
  this->matcher_ = matcher;
  // set image database
  this->database_ = db;
  // create tracker
  this->tracker_ =
      std::make_shared<beam_cv::Tracker>(detector_, descriptor_, matcher_);
}

std::vector<std::tuple<Eigen::Vector2i, Eigen::Vector3d>>
    ImageToImage::Query(cv::Mat query_image, Eigen::Matrix4d& pose_estimate) {
  std::vector<unsigned int> image_ids =
      database_->QueryDatabase(query_image, N);
  /*
   * 1. Get 10 potential matches
   * 2. sample 2 of them and make a tracker, and triangulate all points
   * 3. get pose using ransac p3p
   * 4. determine total # of inliers from both matches used
   * 5. return pose, and correspondences associated to the best 2 matches
   */ 
  std::vector<cv::Mat> images;
  // images[0] is query image and has no pose in pose vector
  images.push_back(query_image);
  std::vector<Eigen::Matrix4d> poses;
  std::vector<std::shared_ptr<beam_calibration::CameraModel>> cam_models;
  // fill pose and image vector, in decreasing order of number of matches to query image
  for (int i = 0; i < image_ids.size(); i++) {
    cv::Mat im = database_->GetImage(image_ids[i]);
    Eigen::Matrix4d p = database_->GetPose(image_ids[i]);
    std::shared_ptr<beam_calibration::CameraModel> model =
        database_->GetCameraModel(image_ids[i]);
    poses.push_back(p);
    images.push_back(im);
    cam_models.push_back(model);
  }
  if (images.size() < 3) {
    BEAM_WARN("Query did not return enough image matches.");
    return {};
  }
  // get feature tracks for every feature in last image of sequence
  std::vector<std::tuple<Eigen::Vector2i, Eigen::Vector3d>> matches;
  std::vector<std::vector<beam_cv::FeatureTrack>> offline_track =
      tracker_->OfflineTracker(images);
  for (int j = 2; j < images.size(); j++) {
    std::vector<beam_cv::FeatureTrack> image_track = offline_track[j];

    for (int i = 0; i < image_track.size(); i++) {
      beam_cv::FeatureTrack t = image_track[i];
      std::vector<Eigen::Vector2i> pixels;
      std::vector<Eigen::Matrix4d> transforms;
      std::vector<std::shared_ptr<beam_calibration::CameraModel>> models;
      Eigen::Vector2i query_pixel_loc_2d;
      Eigen::Vector3d query_pixel_loc_3d;
      bool has_query = false;
      for (auto& lm : t) {
        if (lm.image > 0) {
          Eigen::Vector2i pixel = lm.value.cast<int>();
          Eigen::Matrix4d pose = poses[lm.image - 1];
          std::shared_ptr<beam_calibration::CameraModel> model =
              cam_models[lm.image - 1];
          pixels.push_back(pixel);
          transforms.push_back(pose);
          models.push_back(model);
        } else if (lm.image == 0) {
          has_query = true;
          query_pixel_loc_2d = lm.value.cast<int>();
        }
      }
      // triangulate point using pixels and transforms
      if (has_query && pixels.size() >= 2) {
        query_pixel_loc_3d =
            beam_cv::Triangulation::TriangulatePoint(models, transforms, pixels)
                .value();
        std::tuple<Eigen::Vector2i, Eigen::Vector3d> corr =
            std::make_pair(query_pixel_loc_2d, query_pixel_loc_3d);
        matches.push_back(corr);
      }
    }
  }
  return matches;
}

void ImageToImage::SetDescriptor(
    std::shared_ptr<beam_cv::Descriptor> descriptor) {
  descriptor_ = descriptor;
  tracker_ =
      std::make_shared<beam_cv::Tracker>(detector_, descriptor_, matcher_);
}

void ImageToImage::SetDetector(std::shared_ptr<beam_cv::Detector> detector) {
  detector_ = detector;
  tracker_ =
      std::make_shared<beam_cv::Tracker>(detector_, descriptor_, matcher_);
}

void ImageToImage::SetMatcher(std::shared_ptr<beam_cv::Matcher> matcher) {
  matcher_ = matcher;
  tracker_ =
      std::make_shared<beam_cv::Tracker>(detector_, descriptor_, matcher_);
}

} // namespace relocalization