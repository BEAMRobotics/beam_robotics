#include "inspection/MapLabeler.h"

namespace inspection {

ros::Time TimePointToRosTime(const TimePoint& time_point) {
  ros::Time ros_time;
  ros_time.fromNSec(time_point.time_since_epoch() /
                    std::chrono::nanoseconds(1));
  return ros_time;
}

MapLabeler::MapLabeler(const std::string config_file_location)
    : json_file_path_(config_file_location) {

  // Load JSON file
  std::ifstream i(json_file_path_);
  i >> json_config_;
  images_file_name_ = json_config_["params"]["images_path"];
  map_file_name_ = json_config_["params"]["map_path"];
  poses_file_name_ = json_config_["params"]["poses_path"];
  extrinsics_file_name_ = json_config_["params"]["extrinsics"];
  path_to_camera_calib_ = json_config_["params"]["camera_intrinsics"];

  // Load PointCloud Data
  if (pcl::io::loadPCDFile<BridgePoint>(map_file_name_, *defect_pointcloud_) ==
      -1) {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
  }

  // Load Extrinsic Calibrations
  tf_tree.LoadJSON(extrinsics_file_name_);
  std::cout << "Loaded calibrations from : " << tf_tree.GetCalibrationDate()
            << std::endl;


//  Eigen::Affine3d cam_tf = tf_tree.GetTransform("camera" , "velodyne_horz");



  LoadPrevPoses();

  ros::Time start_time = TimePointToRosTime(final_poses_.front().first);
  ros::Time end_time = TimePointToRosTime(final_poses_.back().first);
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
  //  std::cout << tf2_buffer_._allFramesAsDot() << std::endl;

  // Loading images
//  beam_containers::ImageBridge img;
  img_bridge_.LoadFromJSON("/home/steve/2019_02_13_19_44_Structures_Lab/images/camera0/ImageBridge1/");
  cv::imwrite("/home/steve/test.jpg", img_bridge_.GetBGRImage());


  beam_calibration::Pinhole pinhole;

  pinhole.LoadJSON(path_to_camera_calib_);
  std::cout << pinhole << std::endl;

  // For each image, get timestamp
  // Get map->velodyne transform for image
  // Get map->camera frame transform for image
  // Transform point cloud from map frame into image frame
  // Project each point into mask images
  // Check point mask color
  // Assign field on point cloud

  cv::Mat bgr_img = img_bridge_.GetBGRImage();
//  cv::Mat bgr_mask = img_bridge_.GetBGRMask();
//  bool is_distorted = img_bridge_.GetBGRIsDistorted();
  TimePoint img_time = img_bridge_.GetTimePoint();
  ros::Time ros_img_time = TimePointToRosTime(img_time);
  std::cout << "Image time: " << ros_img_time << std::endl;
  auto cloud = TransformMapToImage(ros_img_time);

  int count = 0;

  for (const auto& point : *cloud) {
    beam::Vec3 vec3{point.x, point.y, point.z};
    auto vec2 = pinhole.ProjectPoint(vec3);
    auto vec2_norm = pinhole.ProjectDistortedPoint(vec3);
//    std::cout << "Projected pixel location = [" << vec2_norm[0] << ", " << vec2_norm[1]
//    << "]....[" << vec2[0] << ", " << vec2[1] << "]." << std::endl;
    auto img_dims = pinhole.GetImgDims();

    if (vec2[0] < 0 || vec2[1] < 0) continue;
    if (vec2[0] < img_dims[0] && vec2[1] < img_dims[1]){
      count++;
    }
//    int img = bgr_img.at<int>(vec2[0], vec2[1]);
  }

  std::cout << "Total pixels in point cloud = " << count << std::endl;
  viewer = boost::make_shared<pcl::visualization::PCLVisualizer>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::copyPointCloud(*defect_pointcloud_, *cloud2);
  viewer->addPointCloud<pcl::PointXYZ> (cloud2, "sample cloud1");



/*
  viewer->initCameraParameters ();
  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerRGBField<BridgePoint> rgb(defect_pointcloud_);
  viewer->addPointCloud<BridgePoint> (defect_pointcloud_, rgb, "sample cloud1", v1);*/


}

DefectCloud::Ptr MapLabeler::TransformMapToImage(ros::Time tf_time) {

/*  geometry_msgs::TransformStamped transform_msg =
      tf2_buffer_.lookupTransform("map", "base_link", tf_time);*/

std::string to_frame = "map";
std::string from_frame = "hvlp_link";

  geometry_msgs::TransformStamped transform_msg =
      tf_tree.GetTransform(to_frame, from_frame, tf_time);

  std::cout << "Transform lookup time = " << tf_time << std::endl;
  std::cout << "Transform = " << transform_msg << std::endl;

  auto transformed_cloud = boost::make_shared<DefectCloud>();

  tf::Transform tf_;
  tf::transformMsgToTF(transform_msg.transform, tf_);
  pcl_ros::transformPointCloud(*defect_pointcloud_, *transformed_cloud, tf_);

  std::cout << "Successfully transformed point cloud from map frame to image frame..." << std::endl;
  std::cout << "Number of points = " << transformed_cloud->width << std::endl;

  return transformed_cloud;
}

void MapLabeler::LoadPrevPoses() {
  Eigen::Affine3d PoseK;
  uint64_t tk;
  TimePoint timepointk;
  std::vector<std::pair<uint64_t, Eigen::Matrix4d>> poses_;

  poses_ = ReadPoseFile(poses_file_name_);
  LOG_INFO("Loaded a total of %d poses.", poses_.size());

  for (uint64_t k = 0; k < poses_.size(); k++) {
    // get resulting transform and timestamp for pose k
    tk = poses_[k].first;
    PoseK.matrix() = poses_[k].second;
    TimePoint timepointk{std::chrono::duration_cast<TimePoint::duration>(
        std::chrono::nanoseconds(tk))};

    // Add result to final pose measurement container
    final_poses_.push_back(std::make_pair(timepointk, PoseK));
    //    final_poses_.emplace_back(timepointk, 0, PoseK);
  }
}

std::vector<std::pair<uint64_t, Eigen::Matrix4d>>
    MapLabeler::ReadPoseFile(const std::string filename) {
  // declare variables
  std::ifstream infile;
  std::string line;
  Eigen::Matrix4d Tk;
  uint64_t tk;
  std::vector<std::pair<uint64_t, Eigen::Matrix4d>> poses;

  // open file
  infile.open(filename);

  // extract poses
  while (!infile.eof()) {
    // get timestamp k
    std::getline(infile, line, ',');
    try {
      tk = std::stod(line);
    } catch (const std::invalid_argument& e) {
      LOG_INFO("Invalid argument, probably at end of file");
      return poses;
    }

    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        if (i == 3 && j == 3) {
          std::getline(infile, line, '\n');
          Tk(i, j) = std::stod(line);
        } else {
          std::getline(infile, line, ',');
          Tk(i, j) = std::stod(line);
        }
      }
    }
    poses.push_back(std::make_pair(tk, Tk));
  }
  return poses;
}

} // end namespace inspection
