#include <lidar_trajectory_validation/LidarTrajectoryValidation.h>

#include <filesystem>

#include <beam_utils/se3.h>
#include <nlohmann/json.hpp>
#include <pcl/common/transforms.h>
#include <rosbag/view.h>

namespace {
nlohmann::json GetTransformJson(const Eigen::Matrix4d& T) {
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Vector3d t = T.block(0, 3, 3, 1);
  Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2);
  Eigen::Quaterniond q(R);

  nlohmann::json J;
  J["T"] = beam::EigenTransformToVector(T);
  J["txyz_m"] = std::vector<double>{t[0], t[1], t[2]};
  J["qwxyz"] = std::vector<double>{q.w(), q.x(), q.y(), q.z()};
  J["RPY_deg"] = std::vector<double>{
      beam::Rad2Deg(rpy[0]), beam::Rad2Deg(rpy[1]), beam::Rad2Deg(rpy[2])};
  return J;
}
} // namespace

using namespace beam_matching;

void LidarTrajectoryValidation::Config::LoadFromJsons(
    const std::string& config_path, const std::string& timestamps_path) {
  BEAM_INFO("Reading config from: {}", config_path);
  nlohmann::json J;
  if (!beam::ReadJson(config_path, J)) {
    throw std::runtime_error{"invalid json file path"};
  }
  beam::ValidateJsonKeysOrThrow(
      {"lidar_frame_id", "lidar_topic", "aggregation_distance_m", "filters"},
      J);

  lidar_frame_id = J["lidar_frame_id"];
  lidar_topic = J["lidar_topic"];
  aggregation_distance_m = J["aggregation_distance_m"];
  lidar_type = J["lidar_type"];
  filters = beam_filtering::LoadFilterParamsVector(J["filters"]);

  BEAM_INFO("Reading timestamps from: {}", timestamps_path);
  nlohmann::json J_timestamps;
  if (!beam::ReadJson(timestamps_path, J_timestamps)) {
    throw std::runtime_error{"invalid json file path"};
  }
  beam::ValidateJsonKeysOrThrow({"loop_timestamps"}, J_timestamps);
  for (const auto& timestamp_J : J_timestamps["loop_timestamps"]) {
    beam::ValidateJsonKeysOrThrow({"reference_s", "target_s"}, timestamp_J);
    ros::Time t_ref(timestamp_J["reference_s"]);
    ros::Time t_tgt(timestamp_J["target_s"]);
    loop_closure_times.emplace(t_tgt, t_ref);
  }
}

LidarTrajectoryValidation::LidarTrajectoryValidation(const Inputs& inputs)
    : inputs_(inputs) {
  config_.LoadFromJsons(inputs_.config, inputs_.timestamps);
  LoadPoses();
  LoadExtrinsics();
  BEAM_INFO("Done initializing LidarTrajectoryValidation");
}

void LidarTrajectoryValidation::LoadPoses() {
  // Load previous poses file specified in labeler json
  BEAM_INFO("Loading poses form {}", inputs_.poses);
  poses_.LoadFromFile(inputs_.poses);
  const auto& poses = poses_.GetPoses();
  const auto& timestamps = poses_.GetTimeStamps();
  BEAM_INFO("Filling TF tree with {} poses", poses.size());
  for (int i = 0; i < poses.size(); i++) {
    Eigen::Affine3d T(poses[i]);
    poses_tree_.AddTransform(T, poses_.GetFixedFrame(), poses_.GetMovingFrame(),
                             timestamps[i]);
  }
}

void LidarTrajectoryValidation::LoadExtrinsics() {
  BEAM_INFO("Loading extrinsic from {}", inputs_.extrinsics);
  extrinsics_.LoadJSON(inputs_.extrinsics);
  T_B_L_ =
      extrinsics_
          .GetTransformEigen(poses_.GetMovingFrame(), config_.lidar_frame_id)
          .matrix();
}

void LidarTrajectoryValidation::Run() {
  OpenBag();
  SetupRegistration();
  SetupOutput();
  GetMeasurements();
  OutputResults();
}

void LidarTrajectoryValidation::SetupRegistration() {
  const auto& m_conf = inputs_.matcher_config;
  auto matcher_type = GetTypeFromConfig(m_conf);
  if (matcher_type == MatcherType::LOAM) {
    BEAM_CRITICAL("LOAM matcher type not implemented");
    throw std::runtime_error{"invalid matcher config"};
  } else if (matcher_type == MatcherType::ICP) {
    matcher_ = std::make_unique<IcpMatcher>(IcpMatcher::Params(m_conf));
  } else if (matcher_type == MatcherType::GICP) {
    matcher_ = std::make_unique<GicpMatcher>(GicpMatcher::Params(m_conf));
  } else if (matcher_type == MatcherType::NDT) {
    matcher_ = std::make_unique<NdtMatcher>(NdtMatcher::Params(m_conf));
  } else {
    BEAM_ERROR("Invalid matcher type");
    throw std::invalid_argument{"invalid json"};
  }
}

void LidarTrajectoryValidation::OpenBag() {
  BEAM_INFO("Loading bag: {}", inputs_.bag);
  try {
    bag_.open(inputs_.bag, rosbag::bagmode::Read);
  } catch (rosbag::BagException& ex) {
    BEAM_CRITICAL("Bag exception : {}}", ex.what());
    throw std::runtime_error{"invalid bag"};
  }
}

void LidarTrajectoryValidation::SetupOutput() {
  if (!std::filesystem::exists(inputs_.output)) {
    BEAM_ERROR("Output directory does not exist: {}", inputs_.output);
    throw std::invalid_argument{"invalid output directory"};
  }
  registration_results_path_ =
      beam::CombinePaths(inputs_.output, "Registrations");
  std::filesystem::create_directory(registration_results_path_);
}

void LidarTrajectoryValidation::GetMeasurements() {
  int counter = 0;
  for (const auto& [tgt_time, ref_time] : config_.loop_closure_times) {
    counter++;
    BEAM_INFO("processing loop closure time {}/{}", counter,
              config_.loop_closure_times.size());
    GetMeasurement(tgt_time, ref_time);
  }
  BEAM_INFO("Done getting measurements");
}

void LidarTrajectoryValidation::GetMeasurement(
    const ros::Time& target_time, const ros::Time& reference_time) {
  LoopClosureResult result;
  result.time_reference = reference_time;
  result.time_target = target_time;
  std::string target_time_str = std::to_string(target_time.toSec());
  std::string output_path =
      beam::CombinePaths(registration_results_path_, target_time_str);

  BEAM_INFO("Aggregating clouds for registration");
  PointCloudPtr reference = std::make_shared<PointCloud>();
  PointCloudPtr target = std::make_shared<PointCloud>();

  Eigen::Matrix4d T_World_Ref;
  Eigen::Matrix4d T_World_Tgt;
  if (!AggregateScansFromBag(reference_time, reference, T_World_Ref)) {
    BEAM_ERROR("Unable to get measurement from time {} to {}",
               std::to_string(reference_time.toSec()),
               std::to_string(target_time.toSec()));
    throw std::invalid_argument{"Invalid loop closure time"};
  }
  if (!AggregateScansFromBag(target_time, target, T_World_Tgt)) {
    BEAM_ERROR("Unable to get measurement from time {} to {}",
               std::to_string(reference_time.toSec()),
               std::to_string(target_time.toSec()));
    throw std::invalid_argument{"Invalid loop closure time"};
  }

  result.T_Ref_Tgt_Init = beam::InvertTransform(T_World_Ref) * T_World_Tgt;

  // filter clouds
  BEAM_INFO("Filtering aggregate clouds");
  PointCloudPtr target_filtered = std::make_shared<PointCloud>();
  PointCloudPtr reference_filtered = std::make_shared<PointCloud>();
  *target_filtered =
      beam_filtering::FilterPointCloud<pcl::PointXYZ>(*target, config_.filters);
  *reference_filtered = beam_filtering::FilterPointCloud<pcl::PointXYZ>(
      *reference, config_.filters);

  // transform target into reference
  PointCloudPtr target_filtered_in_ref = std::make_shared<PointCloud>();
  pcl::transformPointCloud(*target_filtered, *target_filtered_in_ref,
                           Eigen::Affine3d(result.T_Ref_Tgt_Init));

  // run registration
  BEAM_INFO("Running registration");
  matcher_->SetRef(reference_filtered);
  matcher_->SetTarget(target_filtered_in_ref);
  bool match_success = matcher_->Match();
  result.T_Ref_Tgt_Aligned = matcher_->ApplyResult(result.T_Ref_Tgt_Init);
  result.T_Diff = matcher_->GetResult().matrix();
  results_.push_back(result);

  // output results
  BEAM_INFO("Outputting registration results to {}", output_path);
  std::filesystem::create_directory(output_path);
  matcher_->SaveResults(output_path);

  // save filtered vs unfiltered clouds
  PointCloud ref_in_w_unfiltered;
  PointCloud ref_in_w_filtered;
  PointCloud tgt_in_w_unfiltered;
  PointCloud tgt_in_w_filtered;

  std::string path_ref_in_w_unfiltered =
      beam::CombinePaths(output_path, "ref_in_w_unfiltered.pcd");
  std::string path_ref_in_w_filtered =
      beam::CombinePaths(output_path, "ref_in_w_filtered.pcd");
  std::string path_tgt_in_w_unfiltered =
      beam::CombinePaths(output_path, "tgt_in_w_unfiltered.pcd");
  std::string path_tgt_in_w_filtered =
      beam::CombinePaths(output_path, "tgt_in_w_filtered.pcd");

  auto T_World_Tgt_Aligned = T_World_Ref * result.T_Ref_Tgt_Aligned;
  pcl::transformPointCloud(*reference, ref_in_w_unfiltered,
                           Eigen::Affine3d(T_World_Ref));
  pcl::transformPointCloud(*reference_filtered, ref_in_w_filtered,
                           Eigen::Affine3d(T_World_Ref));
  pcl::transformPointCloud(*target, tgt_in_w_unfiltered,
                           Eigen::Affine3d(T_World_Tgt_Aligned));
  pcl::transformPointCloud(*target_filtered, tgt_in_w_filtered,
                           Eigen::Affine3d(T_World_Tgt_Aligned));

  beam::SavePointCloud(path_ref_in_w_unfiltered, ref_in_w_unfiltered);
  beam::SavePointCloud(path_ref_in_w_filtered, ref_in_w_filtered);
  beam::SavePointCloud(path_tgt_in_w_unfiltered, tgt_in_w_unfiltered);
  beam::SavePointCloud(path_tgt_in_w_filtered, tgt_in_w_filtered);
  BEAM_INFO("Done saving clouds");
}

bool LidarTrajectoryValidation::AggregateScansFromBag(
    const ros::Time& center_time, PointCloudPtr& output_cloud,
    Eigen::Matrix4d& T_World_Cloud) const {
  auto time_window = GetAggregationTimeWindow(center_time);
  ros::Time start_time = time_window.first;
  ros::Time end_time = time_window.second;

  output_cloud = std::make_shared<PointCloud>();
  rosbag::View view(bag_, rosbag::TopicQuery(config_.lidar_topic), start_time,
                    end_time);
  if (view.size() == 0) {
    BEAM_ERROR("Empty bag view with timestamp range: [{}, {}]s and topic: {}",
               std::to_string(start_time.toSec()),
               std::to_string(end_time.toSec()), config_.lidar_topic);
    return false;
  }

  // get start pose
  Eigen::Affine3d T_World_BaselinkStart = poses_tree_.GetTransformEigen(
      poses_.GetFixedFrame(), poses_.GetMovingFrame(), start_time);
  Eigen::Matrix4d T_LidarStart_World =
      beam::InvertTransform(T_B_L_) * T_World_BaselinkStart.inverse().matrix();
  T_World_Cloud =
      T_World_BaselinkStart.matrix() * beam::InvertTransform(T_B_L_);
  for (auto iter = view.begin(); iter != view.end(); iter++) {
    auto sensor_msg = iter->instantiate<sensor_msgs::PointCloud2>();
    if (!sensor_msg) {
      BEAM_CRITICAL("Invalid message type, check your topics");
      return false;
    }
    ros::Time scan_time = sensor_msg->header.stamp;

    if (config_.lidar_type == "VELODYNE") {
      pcl::PointCloud<PointXYZIRT> cloud;
      beam::ROSToPCL(cloud, *sensor_msg);
      for (const auto& p : cloud.points) {
        // get pose at point time
        ros::Time pt = scan_time + ros::Duration(p.time);
        if (pt < start_time || pt > end_time) { continue; }
        Eigen::Affine3d T_World_BaselinkStart = poses_tree_.GetTransformEigen(
            poses_.GetFixedFrame(), poses_.GetMovingFrame(), pt);
        Eigen::Matrix4d T_LidarStart_LidarK =
            T_LidarStart_World * T_World_BaselinkStart * T_B_L_;

        // convert to start time of aggregate and add
        Eigen::Vector4d p_deskewed =
            T_LidarStart_LidarK * Eigen::Vector4d(p.x, p.y, p.z, 1);
        pcl::PointXYZ p_new;
        p_new.x = p_deskewed[0];
        p_new.y = p_deskewed[1];
        p_new.z = p_deskewed[2];
        output_cloud->points.push_back(p_new);
      }
    } else if (config_.lidar_type == "OUSTER") {
      pcl::PointCloud<PointXYZITRRNR> cloud;
      beam::ROSToPCL(cloud, *sensor_msg);
      for (const auto& p : cloud.points) {
        // get pose at point time
        ros::Time pt = scan_time + ros::Duration(p.time);
        Eigen::Affine3d T_World_BaselinkStart = poses_tree_.GetTransformEigen(
            poses_.GetFixedFrame(), poses_.GetMovingFrame(), pt);
        Eigen::Matrix4d T_LidarStart_LidarK =
            T_LidarStart_World * T_World_BaselinkStart.matrix() * T_B_L_;

        // convert to start time of aggregate and add
        Eigen::Vector4d p_deskewed =
            T_LidarStart_LidarK * Eigen::Vector4d(p.x, p.y, p.z, 1);
        pcl::PointXYZ p_new;
        p_new.x = p_deskewed[0];
        p_new.y = p_deskewed[1];
        p_new.z = p_deskewed[2];
        output_cloud->points.push_back(p_new);
      }
    } else {
      BEAM_ERROR("Invalid input lidar type: {}, options: VELODYNE, OUSTER",
                 config_.lidar_type);
      throw std::runtime_error{"invalid input lidar type"};
    }
  }

  return true;
}

std::pair<ros::Time, ros::Time>
    LidarTrajectoryValidation::GetAggregationTimeWindow(
        const ros::Time& center_time) const {
  // use poses to get time window for scans to aggregate
  ros::Time center_time_exact;
  Eigen::Matrix4d T_W_B_center;
  int center_iter = -1;
  auto pose_timestamps = poses_.GetTimeStamps();
  for (int i = 0; i < pose_timestamps.size(); i++) {
    const ros::Time& time = pose_timestamps.at(i);
    if (time >= center_time) {
      center_time_exact = time;
      center_iter = i;
      T_W_B_center = poses_.GetPoses().at(i);
      break;
    }
  }

  if (center_iter == -1) {
    BEAM_ERROR("Cannot get a time window for center time {}, is this time "
               "correct? Poses time window: [{}, {}]",
               std::to_string(center_time.toSec()),
               std::to_string(pose_timestamps.front().toSec()),
               std::to_string(pose_timestamps.back().toSec()));
    throw std::runtime_error("cannot get time window");
  }

  // get end time by starting from middle and incrementing until the change in
  // pose is sufficient. Set end time to start with the last pose. If this isn't
  // sufficiently far enough from center, then start time will compensate for it
  ros::Time time_end = pose_timestamps.back();
  Eigen::Matrix4d T_W_B_end = poses_.GetPoses().back();
  for (int i = center_iter + 1; i < pose_timestamps.size(); i++) {
    const ros::Time& time = pose_timestamps.at(i);
    Eigen::Affine3d T_W_B_i = poses_tree_.GetTransformEigen(
        poses_.GetFixedFrame(), poses_.GetMovingFrame(), time);
    Eigen::Matrix4d T_Bc_Bi =
        beam::InvertTransform(T_W_B_center) * T_W_B_i.matrix();
    double dist = T_Bc_Bi.block(0, 3, 3, 1).norm();
    if (dist >= config_.aggregation_distance_m / 2) {
      time_end = time;
      break;
    }
  }

  // get start time by starting from center iter and going closer to the
  // beginning
  ros::Time time_start(0);
  for (int i = center_iter - 1; i >= 0; i--) {
    const ros::Time& time = pose_timestamps.at(i);
    Eigen::Affine3d T_W_B_i = poses_tree_.GetTransformEigen(
        poses_.GetFixedFrame(), poses_.GetMovingFrame(), time);
    auto T_Be_Bi = beam::InvertTransform(T_W_B_end) * T_W_B_i.matrix();
    double dist = T_Be_Bi.block(0, 3, 3, 1).norm();
    if (dist >= config_.aggregation_distance_m) {
      time_start = time;
      break;
    }
  }

  // if the start time is still 0, then the center time is too close to the
  // start. Therefore we start at time 0 and re-calculate the end time
  if (time_start == ros::Time(0)) {
    time_end = ros::Time(0);
    time_start = pose_timestamps.at(0);
    Eigen::Matrix4d T_W_B_start = poses_.GetPoses().at(0);
    for (int i = 0; i < pose_timestamps.size(); i++) {
      const ros::Time& time = pose_timestamps.at(i);
      Eigen::Affine3d T_W_B_i = poses_tree_.GetTransformEigen(
          poses_.GetFixedFrame(), poses_.GetMovingFrame(), time);
      auto T_Bs_Bi = beam::InvertTransform(T_W_B_start) * T_W_B_i.matrix();
      double dist = T_Bs_Bi.block(0, 3, 3, 1).norm();
      if (dist >= config_.aggregation_distance_m) {
        time_end = time;
        break;
      }
    }
  }

  if (time_end == ros::Time(0)) {
    BEAM_ERROR("Cannot get a time window for center time {} with a aggregation "
               "distance of at least {}m",
               std::to_string(center_time.toSec()),
               config_.aggregation_distance_m);
    throw std::runtime_error("cannot get time window");
  }

  return std::make_pair(time_start, time_end);
}

void LidarTrajectoryValidation::OutputResults() {
  BEAM_INFO("Outputting results");

  nlohmann::json J;
  J["baselink_frame_id"] = poses_.GetMovingFrame();
  J["lidar_frame_id"] = config_.lidar_frame_id;
  J["map_frame_id"] = poses_.GetFixedFrame();

  std::vector<nlohmann::json> results_J;
  double t_norm_sum = 0;
  double angle_abs_deg_sum = 0;
  for (const LoopClosureResult& result : results_) {
    nlohmann::json result_J;
    result_J["time_reference_s"] =
        std::to_string(result.time_reference.toSec());
    result_J["time_target_s"] = std::to_string(result.time_target.toSec());
    result_J["T_Ref_Tgt_Aligned"] = GetTransformJson(result.T_Ref_Tgt_Aligned);
    result_J["T_Ref_Tgt_Init"] = GetTransformJson(result.T_Ref_Tgt_Init);
    result_J["T_Diff"] = GetTransformJson(result.T_Diff);

    // get translation errors
    double t_norm = result.T_Diff.block(0, 3, 3, 1).norm();
    t_norm_sum += t_norm;
    result_J["t_norm"] = t_norm;

    // get rotation errors
    Eigen::Matrix3d R = result.T_Diff.block(0, 0, 3, 3);
    Eigen::AngleAxisd aa(R);
    double angle_abs_deg = std::abs(beam::Rad2Deg(aa.angle()));
    result_J["angle_abs_deg"] = angle_abs_deg;
    angle_abs_deg_sum += angle_abs_deg;

    // add to results vector
    results_J.push_back(result_J);
  }
  J["mean_translation_norm_m"] = t_norm_sum / results_.size();
  J["mean_rotation_deg"] = angle_abs_deg_sum / results_.size();
  J["registrations"] = results_J;

  std::string output_json =
      beam::CombinePaths(inputs_.output, "trajectory_validation_results.json");
  BEAM_INFO("Saving results to: {}", output_json);
  std::ofstream filejson(output_json);
  filejson << std::setw(4) << J << std::endl;

  BEAM_INFO("Done outputting results");
}
