#include <visualize_feature_tracks/feature_tracker.h>

#include <iostream>
#include <string>

#include <beam_utils/log.h>
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/matchers/Matchers.h>

namespace visualize_feature_tracks {

FeatureTracker::FeatureTracker() {
  std::shared_ptr<beam_cv::Detector> detector =
      std::make_shared<beam_cv::ORBDetector>();
  std::shared_ptr<beam_cv::Descriptor> descriptor =
      std::make_shared<beam_cv::ORBDescriptor>();
  std::shared_ptr<beam_cv::Matcher> matcher =
      std::make_shared<beam_cv::BFMatcher>();
  tracker_ = std::make_unique<beam_cv::Tracker>(detector, descriptor, matcher);
}

std::vector<beam_cv::FeatureTrack> FeatureTracker::GetTracks(const cv::Mat& image, const ros::Time& stamp) {
  tracker_->AddImage(image, stamp);
  std::vector<beam_cv::FeatureTrack> tracks = tracker_->GetTracks(num_images_);
  num_images_++;
  return tracks;
}

cv::Mat FeatureTracker::DrawTracks(const std::vector<beam_cv::FeatureTrack>& tracks, const cv::Mat& image){
    return tracker_->DrawTracks(tracks, image);
}

}  // namespace visualize_feature_tracks
