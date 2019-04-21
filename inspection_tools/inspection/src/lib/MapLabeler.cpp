#include "inspection/MapLabeler.h"
#include <beam_containers/Utilities.h>

namespace inspection {

ros::Time TimePointToRosTime(const TimePoint& time_point) {
  ros::Time ros_time;
  ros_time.fromNSec(time_point.time_since_epoch() /
                    std::chrono::nanoseconds(1));
  return ros_time;
}

MapLabeler::MapLabeler(std::string config_file_location)
    : json_labeler_filepath_(config_file_location) {

  // Process core json file
  ProcessJSONConfig();

  // Load map
  if (pcl::io::loadPCDFile<BridgePoint>(map_file_name_, *defect_pointcloud_) ==
      -1) {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
  }

  // Load previous poses file specified in labeler json
  LoadPrevPoses();
  auto poses = beam_containers::ReadPoseFile(poses_file_name_);
  for (int i = 0; i < poses.size(); i++){
    std::cout << poses[i].first.time_since_epoch().count() << " ----- ";
    std::cout << final_poses_[i].first.time_since_epoch().count()<< std::endl;
  }

  // Fill tf tree object with robot poses & extrinsics
  FillTFTree();

  /**
   * Load Image Containers
   */
  std::vector<int> indices = {0, 5, 10};
  int num_cams = cameras_.size();
  defect_clouds_.resize(num_cams);
  for (size_t cam = 0; cam < num_cams; cam++) {
    for (size_t i = 0; i < indices.size(); i++) {
      std::cout << cameras_[cam].img_paths[indices[i]] << std::endl;
      img_bridge_.LoadFromJSON(cameras_[cam].img_paths[indices[i]]);

      beam::HighResolutionTimer timer;

      Camera* camera = &(cameras_[cam]);

      DefectCloud::Ptr colored_cloud = ProjectImgToMap(img_bridge_, camera);
      defect_clouds_[cam].push_back(colored_cloud);
      std::cout << "Elapsed time: " << timer.elapsed() << std::endl;

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb =
          boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
      pcl::copyPointCloud(*colored_cloud, *cloud_rgb);

      rgb_clouds.push_back(cloud_rgb);
    }
  }
}

void MapLabeler::ProcessJSONConfig() {
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
    std::string camera_folder_path = images_file_name_ + "/" + std::string(camera_name);
    std::cout << camera_folder_path << std::endl;

    camera_list_.emplace_back(camera_name);
    Camera cam{camera_folder_path, camera_name,
               json_config_["params"]["intrinsics_path"]};
    cameras_.push_back(std::move(cam));
  }
}

void MapLabeler::FillTFTree() {
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

void MapLabeler::SaveLabeledClouds() {
  std::string root_cloud_folder = images_file_name_ + "/../clouds";
  for (size_t cam = 0; cam < defect_clouds_.size(); cam++) {
    int cloud_number = 1;
    for (const auto& cloud : defect_clouds_[cam]) {
      std::string file_name =
          cameras_[cam].camera_id + "_" + std::to_string(cloud_number) + ".pcd";
      pcl::io::savePCDFileBinary(root_cloud_folder + "/" + file_name, *cloud);
      cloud_number++;
    }
    std::cout << "Saved " << cloud_number - 1
              << " labeled point clouds from Camera: "
              << cameras_[cam].camera_id << std::endl;
  }
}

void MapLabeler::DrawColoredClouds() {
  int id = 0;
  std::vector<
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>>
      rgb_fields;
  for (auto& cloud : rgb_clouds) {
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
        cloud);
    rgb_fields.push_back(rgb);
    std::cout << "Adding point cloud with id: " << id << std::endl;
    std::cout << "  Point cloud size = " << cloud->points.size() << std::endl;
    viewer->addPointCloud<pcl::PointXYZRGB>(rgb_clouds[id], rgb_fields[id],
                                            std::to_string(id));
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, std::to_string(id));
    id++;
  }

  viewer->setBackgroundColor(1, 1, 1);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
}

DefectCloud::Ptr MapLabeler::TransformMapToImageFrame(ros::Time tf_time,
                                                      std::string frame_id) {
  std::string to_frame = frame_id;
  std::string from_frame = "map"; //"F1_link";

  geometry_msgs::TransformStamped transform_msg =
      tf_tree.GetTransform(to_frame, from_frame, tf_time);

  //  std::cout << "Transform lookup time = " << tf_time << std::endl;
  //  std::cout << "Transform = " << transform_msg << std::endl;

  auto transformed_cloud = boost::make_shared<DefectCloud>();

  tf::Transform tf_;
  tf::transformMsgToTF(transform_msg.transform, tf_);
  pcl_ros::transformPointCloud(*defect_pointcloud_, *transformed_cloud, tf_);

  tf_temp_ = tf_.inverse();
  std::cout
      << "Successfully transformed point cloud from map frame to image frame"
      << frame_id << "." << std::endl;

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

void MapLabeler::PlotFrames(std::string frame_id, PCLViewer viewer) {
  std::vector<Eigen::Affine3f> coord_frames;

  for (const auto& pose : final_poses_) {
    // beam::chronoToRosTime(pose.first);
    ros::Time time = TimePointToRosTime(pose.first); //
    std::string to_frame = "map";
    geometry_msgs::TransformStamped g_tf_stamped =
        tf_tree.GetTransform(to_frame, frame_id, time);

    Eigen::Isometry3d eig = tf2::transformToEigen(g_tf_stamped);
    Eigen::Affine3f affine_tf(eig.cast<float>());

    std::stringstream unique_id;
    unique_id << frame_id << "_" << time.sec;

    /*    std::cout << "Adding affine_tf : " << unique_id.str()
                  << " with transform: " << g_tf_stamped.transform.translation.x
                  << std::endl;*/

    viewer->addCoordinateSystem(0.5, affine_tf, unique_id.str());
  }
}

DefectCloud::Ptr MapLabeler::ProjectImgToMap(beam_containers::ImageBridge img,
                                             Camera* camera) {
  cv::Mat bgr_img = img.GetBGRImage();
  TimePoint img_time = img_bridge_.GetTimePoint();
  ros::Time ros_img_time = TimePointToRosTime(img_time);
  beam::Vec2 img_dims = camera->cam_intrinsics->GetImgDims();

  // Get map in camera frame
  DefectCloud::Ptr map_cloud = TransformMapToImageFrame(
      ros_img_time, camera->cam_intrinsics->GetFrameId());

  auto xyz_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  pcl::copyPointCloud(*map_cloud, *xyz_cloud);

  camera->colorizer->SetPointCloud(xyz_cloud);

  camera->colorizer->SetImage(img.GetBGRImage());

  // Get colored cloud & remove uncolored points
  auto xyzrgb_cloud = camera->colorizer->ColorizePointCloud();
  xyzrgb_cloud->points.erase(
      std::remove_if(xyzrgb_cloud->points.begin(), xyzrgb_cloud->points.end(),
                     [](auto& point) {
                       return (point.r == 0 && point.g == 0 && point.b == 0);
                     }),
      xyzrgb_cloud->points.end());

  std::cout << "XYZRGB cloud size = " << xyzrgb_cloud->width << std::endl;
  std::cout << "XYZRGB points size = " << xyzrgb_cloud->points.size()
            << std::endl;

  DefectCloud::Ptr colored_cloud = boost::make_shared<DefectCloud>();
  pcl::copyPointCloud(*xyzrgb_cloud, *colored_cloud);

  DefectCloud::Ptr cloud3 = boost::make_shared<DefectCloud>();
  pcl_ros::transformPointCloud(*colored_cloud, *cloud3, tf_temp_);

  return cloud3;
}

} // end namespace inspection
