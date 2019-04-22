#include "inspection/LadybugLabeler.h"
#include <beam_utils/time.hpp>

namespace inspection {

ros::Time TimePointToRosTime(const TimePoint& time_point) {
  ros::Time ros_time;
  ros_time.fromNSec(time_point.time_since_epoch() /
                    std::chrono::nanoseconds(1));
  return ros_time;
}

LadybugLabeler::LadybugLabeler(std::string config_file_location)
    : json_labeler_filepath_(config_file_location) {
  // Process core json file
  ProcessJSONConfig();

  // Load map
  if (pcl::io::loadPCDFile<BridgePoint>(map_file_name_, *defect_pointcloud_) ==
      -1) {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
  }

  // Load previous poses file specified in labeler json
  final_poses_ = beam_containers::ReadPoseFile(poses_file_name_);

  // Fill tf tree object with robot poses & extrinsics
  FillTFTree();

  if (output_clouds_for_nick) { OutputCloudsForNick(); }
}

/**
 * @todo-Steve This is a temporary location for this feature
 */
void LadybugLabeler::OutputCloudsForNick() {
  std::vector<int> indices(cameras_[0].img_paths.size());

  int num_cams = cameras_.size();
  defect_clouds_.resize(num_cams);
  for (size_t cam = 0; cam < num_cams; cam++) {
    for (size_t i = 0; i < cameras_[cam].img_paths.size(); i++) {
      img_bridge_.LoadFromJSON(cameras_[cam].img_paths[i]);
      Camera* camera = &(cameras_[cam]);
      // First we get the folder path we want to save to
      std::string output_path = camera->folder_path_ + "/ImageBridge" +
                                std::to_string(img_bridge_.GetImageSeq());

      // Next we get timestamp and get map in image frame
      std::string to_frame = camera->cam_intrinsics->GetFrameId();
      std::string from_frame = "map";
      ros::Time tf_time = TimePointToRosTime(img_bridge_.GetTimePoint());
      geometry_msgs::TransformStamped transform_msg =
          tf_tree.GetTransform(to_frame, from_frame, tf_time);
      auto transformed_cloud = boost::make_shared<DefectCloud>();

      tf::Transform tf_;
      tf::transformMsgToTF(transform_msg.transform, tf_);
      pcl_ros::transformPointCloud(*defect_pointcloud_, *transformed_cloud,
                                   tf_);

      PointCloudXYZ::Ptr xyz_cloud_out = boost::make_shared<PointCloudXYZ>();
      pcl::copyPointCloud(*transformed_cloud, *xyz_cloud_out);

      std::cout << output_path << std::endl;
      pcl::io::savePCDFileBinary(output_path + "/Cloud.pcd", *xyz_cloud_out);
    }
  }
}

void LadybugLabeler::ProcessJSONConfig() {
  // Start by loading the root json file & assigning variables
  std::ifstream json_config_stream(json_labeler_filepath_);
  json_config_stream >> json_config_;
  images_file_name_ = json_config_["params"]["images_path"];
  map_file_name_ = json_config_["params"]["map_path"];
  poses_file_name_ = json_config_["params"]["poses_path"];
  extrinsics_file_name_ = json_config_["params"]["extrinsics_path"];
  path_to_camera_calib_ = json_config_["params"]["intrinsics_path"];

  // Next we load in the CamerasList.json
  nlohmann::json json_cameras_list;
  std::ifstream camera_list_stream(images_file_name_ + "/CamerasList.json");
  camera_list_stream >> json_cameras_list;
  std::cout << json_cameras_list << std::endl;

  // Now we create Camera objects for each camera defined in the
  // CamerasList.json file.
  for (const auto& camera_name : json_cameras_list["Items"]) {
    std::string camera_folder_path =
        images_file_name_ + "/" + std::string(camera_name);
    std::cout << camera_folder_path << std::endl;

    camera_list_.emplace_back(camera_name);
    Camera cam{camera_folder_path, camera_name,
               json_config_["params"]["intrinsics_path"]};
    cameras_.push_back(std::move(cam));
  }
}

void LadybugLabeler::FillTFTree() {
  ros::Time start_time = TimePointToRosTime(final_poses_.front().first);
  ros::Time end_time = TimePointToRosTime(final_poses_.back().first);
  tf_tree.start_time_ = start_time;
  ros::Duration dur = end_time - start_time;
  std::cout << "Start time of poses: " << start_time << std::endl;
  std::cout << "End time of poses: " << end_time << std::endl;
  std::cout << "Duration of poses is : " << dur << std::endl;

  tf::Transform tf_transform;
  geometry_msgs::TransformStamped tf_msg;
  for (const auto& pose : final_poses_) {
    tf_msg = tf2::eigenToTransform(pose.second);
    tf_msg.header.stamp = TimePointToRosTime(pose.first);
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "hvlp_link";
    tf_tree.AddTransform(tf_msg);
    tf2_buffer_.setTransform(tf_msg, "t");
  }

  tf_tree.LoadJSON(extrinsics_file_name_);
  std::cout << "Loaded calibrations from : " << tf_tree.GetCalibrationDate()
            << std::endl;
}

} // end namespace inspection
