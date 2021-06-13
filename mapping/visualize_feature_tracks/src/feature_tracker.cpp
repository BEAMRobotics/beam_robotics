#include <visualize_feature_tracks/feature_tracker.h>

#include <iostream>
#include <string>

#include <beam_utils/log.h>

namespace visualize_feature_tracks {

FeatureTracker::FeatureTracker() {}

std::vector<beam_containers::LandmarkMeasurement> FeatureTracker::AddImage(
    const cv::Mat& image) {
  std::vector<beam_containers::LandmarkMeasurement> tracks;

  return tracks;
}

}  // namespace visualize_feature_tracks
