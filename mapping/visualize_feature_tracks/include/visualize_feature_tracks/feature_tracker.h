#pragma once

#include <opencv2/opencv.hpp>
#include <ros/time.h>

#include <beam_cv/tracker/Tracker.h>

namespace visualize_feature_tracks {

class FeatureTracker {
 public:
  FeatureTracker();

  std::vector<beam_cv::FeatureTrack> GetTracks(const cv::Mat& image, const ros::Time& stamp);

  cv::Mat DrawTracks(const std::vector<beam_cv::FeatureTrack>& tracks, const cv::Mat& image);

 private:
   std::unique_ptr<beam_cv::Tracker> tracker_;
   size_t num_images_{0};
};

}  // namespace visualize_feature_tracks