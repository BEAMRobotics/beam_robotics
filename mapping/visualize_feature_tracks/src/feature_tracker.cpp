#include <visualize_feature_tracks/feature_tracker.h>

#include <iostream>
#include <string>

#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/matchers/Matchers.h>
#include <beam_cv/trackers/Trackers.h>
#include <beam_utils/log.h>

namespace visualize_feature_tracks {

using namespace beam_cv;

FeatureTracker::FeatureTracker() {
  Initialize();
}

FeatureTracker::FeatureTracker(const Params& params) : params_(params) {
  Initialize();
}

void FeatureTracker::Initialize() {
  std::shared_ptr<Detector> detector = GetDetector();
  std::shared_ptr<Descriptor> descriptor = GetDescriptor();
  InitTracker(detector, descriptor);
}

std::vector<FeatureTrack> FeatureTracker::GetTracks(const cv::Mat& image,
                                                    const ros::Time& stamp) {
  tracker_->AddImage(image, stamp);
  std::vector<FeatureTrack> tracks = tracker_->GetTracks(num_images_);
  num_images_++;
  return tracks;
}

cv::Mat FeatureTracker::DrawTracks(const std::vector<FeatureTrack>& tracks,
                                   const cv::Mat& image) {
  return tracker_->DrawTracks(tracks, image);
}

std::shared_ptr<Detector> FeatureTracker::GetDetector() {
  std::shared_ptr<Detector> detector;
  auto detector_type_iter =
      beam_cv::internal::DetectorStringTypeMap.find(params_.detector_type);
  DetectorType detector_type = DetectorType::ORB;
  if (detector_type_iter == beam_cv::internal::DetectorStringTypeMap.end()) {
    std::string options = GetDetectorTypes();
    BEAM_ERROR("Invalid detector type, using default: ORB. Options: {}",
               options);
  } else {
    detector_type = detector_type_iter->second;
  }
  return Detector::Create(detector_type, params_.detector_config);
}

std::shared_ptr<Descriptor> FeatureTracker::GetDescriptor() {
  if (params_.descriptor_type == "NONE") {
    if (params_.tracker_type == "DESCMATCHING") {
      BEAM_ERROR("Must supply descriptor for tracker of type DESCMATCHING. "
                 "Using default: ORB.");
      params_.descriptor_type = "ORB";
    } else {
      return nullptr;
    }
  }

  std::shared_ptr<Descriptor> descriptor;
  auto descriptor_type_iter =
      beam_cv::internal::DescriptorStringTypeMap.find(params_.descriptor_type);
  DescriptorType descriptor_type = DescriptorType::ORB;
  if (descriptor_type_iter ==
      beam_cv::internal::DescriptorStringTypeMap.end()) {
    std::string options = GetDescriptorTypes();
    options += ", NONE";
    BEAM_ERROR("Invalid descriptor type, using default: ORB. Options: {}",
               options);
  } else {
    descriptor_type = descriptor_type_iter->second;
  }
  return Descriptor::Create(descriptor_type, params_.descriptor_config);
}

std::shared_ptr<Matcher> FeatureTracker::GetMatcher() {
  std::shared_ptr<Matcher> matcher;
  auto matcher_type_iter = MatcherTypeStringMap.find(params_.matcher_type);
  MatcherType matcher_type = MatcherType::BF;
  if (matcher_type_iter == MatcherTypeStringMap.end()) {
    std::string options = GetMatcherTypes();
    BEAM_ERROR("Invalid matcher type, using default: BF. Options: {}", options);
  } else {
    matcher_type = matcher_type_iter->second;
  }
  return Matcher::Create(matcher_type, params_.matcher_config);
}

void FeatureTracker::InitTracker(std::shared_ptr<Detector> detector,
                                 std::shared_ptr<Descriptor> descriptor) {
  // get type
  auto tracker_type_iter = TrackerTypeStringMap.find(params_.tracker_type);
  TrackerType tracker_type = TrackerType::KL;
  if (tracker_type_iter == TrackerTypeStringMap.end()) {
    std::string options = GetTrackerTypes();
    BEAM_ERROR("Invalid tracker type, using default: KL. Options: {}", options);
  } else {
    tracker_type = tracker_type_iter->second;
  }

  // instantiate pointer
  if (tracker_type == TrackerType::KL) {
    KLTracker::Params params;
    params.LoadFromJson(params_.tracker_config);
    tracker_ = std::make_shared<KLTracker>(params, detector, descriptor, 10);
  } else if (tracker_type == TrackerType::DESCMATCHING) {
    std::shared_ptr<Matcher> matcher = GetMatcher();
    tracker_ = std::make_shared<DescMatchingTracker>(detector, descriptor,
                                                     matcher, 10);
  } else {
    BEAM_ERROR("Tracker type not supported by visualize_feature_tracks.");
    throw std::invalid_argument{"Unsupported tracker type."};
  }
}

} // namespace visualize_feature_tracks
