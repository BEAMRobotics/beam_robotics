#pragma once

#include <beam_containers/LandmarkMeasurement.h>

namespace visualize_feature_tracks {

class FeatureTracker {
 public:
  FeatureTracker();

  std::vector<beam_containers::LandmarkMeasurement> AddImage(const cv::Mat& image);

 private:
};

}  // namespace visualize_feature_tracks