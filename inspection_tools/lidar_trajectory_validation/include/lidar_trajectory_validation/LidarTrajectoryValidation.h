#pragma once

#include <rosbag/bag.h>

#include <beam_calibration/TfTree.h>
#include <beam_filtering/Utils.h>
#include <beam_mapping/Poses.h>
#include <beam_matching/Matchers.h>
#include <beam_utils/pointclouds.h>

class LidarTrajectoryValidation {
public:
  struct Inputs {
    std::string poses;
    std::string extrinsics;
    std::string config;
    std::string matcher_config;
    std::string bag;
    std::string output;
    std::string timestamps;
  };

  struct Config {
    std::string lidar_type; // VELODYNE OR OUSTER
    std::string lidar_frame_id;
    std::string lidar_topic;
    double aggregation_distance_m;
    std::vector<beam_filtering::FilterParamsType> filters;
    std::map<ros::Time, ros::Time> loop_closure_times; // tgt -> ref
    void LoadFromJsons(const std::string& config_path,
                       const std::string& timestamps_path);
  };

  struct LoopClosureResult {
    Eigen::Matrix4d T_Ref_Tgt_Aligned;
    Eigen::Matrix4d T_Ref_Tgt_Init;
    Eigen::Matrix4d T_Diff;
    ros::Time time_reference;
    ros::Time time_target;
  };

  explicit LidarTrajectoryValidation(const Inputs& inputs);

  ~LidarTrajectoryValidation() = default;

  void Run();

private:
  void OpenBag();

  void SetupRegistration();

  void LoadPoses();

  void LoadExtrinsics();

  void SetupOutput();

  void GetMeasurements();

  void GetMeasurement(const ros::Time& target_time,
                      const ros::Time& reference_time);

  std::pair<ros::Time, ros::Time>
      GetAggregationTimeWindow(const ros::Time& center_time) const;

  bool AggregateScansFromBag(const ros::Time& center_time,
                             PointCloudPtr& output_cloud,
                             Eigen::Matrix4d& T_World_Cloud) const;

  void OutputResults();

  void OutputMaps() const;

  Inputs inputs_;
  Config config_;

  beam_calibration::TfTree poses_tree_;
  beam_mapping::Poses poses_;
  beam_calibration::TfTree extrinsics_;
  Eigen::Matrix4d T_B_L_;
  std::unique_ptr<beam_matching::Matcher<PointCloudPtr>> matcher_;

  std::vector<LoopClosureResult> results_;
  rosbag::Bag bag_;
  std::string registration_results_path_;
};
