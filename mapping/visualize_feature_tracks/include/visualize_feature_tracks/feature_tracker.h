#pragma once

#include <opencv2/opencv.hpp>
#include <ros/time.h>

#include <beam_cv/trackers/Tracker.h>

namespace visualize_feature_tracks {

class FeatureTracker {
 public:
  struct Params {
    // options: see TrackerTypeStringMap in Tracker.h
    std::string tracker_type{"KL"};

    // full path to tracker config json. If empty, it will use default params
    // defined in the class
    std::string tracker_config{""};

    // see DetectorTypeStringMap in Detector.h
    std::string detector_type{"ORB"};

    // full path to detector config json. If empty, it will use default params
    // defined in the class
    std::string detector_config{""};

    // options: see DescriptorTypeStringMap in Descriptor.h
    std::string descriptor_type{"ORB"};

    // full path to descriptor config json. If empty, it will use default params
    // defined in the class
    std::string descriptor_config{""};

    // options: see MatcherTypeStringMap in Matcher.h
    std::string matcher_type{"BF"};

    // full path to matcher config json. If empty, it will use default params
    // defined in the class
    std::string matcher_config{""};
  };

  /**
   * @brief Constructor that uses default params
   */
  FeatureTracker();

  /**
   * @brief constructor that requires a params object
   * @param params see struct defined above
   */
  FeatureTracker(const Params& params);

  std::vector<beam_cv::FeatureTrack> GetTracks(const cv::Mat& image,
                                               const ros::Time& stamp);

  cv::Mat DrawTracks(const std::vector<beam_cv::FeatureTrack>& tracks,
                     const cv::Mat& image);

 private:
  void Initialize();

  std::shared_ptr<beam_cv::Detector> GetDetector();

  std::shared_ptr<beam_cv::Descriptor> GetDescriptor();

  std::shared_ptr<beam_cv::Matcher> GetMatcher();

  void InitTracker(std::shared_ptr<beam_cv::Detector> detector,
                   std::shared_ptr<beam_cv::Descriptor> descriptor);

  Params params_;
  std::shared_ptr<beam_cv::Tracker> tracker_;
  size_t num_images_{0};
};

}  // namespace visualize_feature_tracks