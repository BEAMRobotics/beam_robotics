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
  /**
   * Load JSON
   */
  std::ifstream json_config_stream(json_file_path_);
  json_config_stream >> json_config_;
  images_file_name_ = json_config_["params"]["images_path"];

  std::ifstream camera_list_stream(images_file_name_ +
                                   std::string("/CamerasList.json"));
  json json_cameras_list;
  camera_list_stream >> json_cameras_list;
  std::cout << json_cameras_list << std::endl;
  for (const auto& camera : json_cameras_list["Items"]) {
    std::stringstream camera_folder_path;
    camera_folder_path << images_file_name_ << "/" << std::string(camera);
    //    std::string camera_folder_path = images_file_name_ +
    //    std::string(camera);
    std::cout << camera_folder_path.str() << std::endl;
    camera_list.emplace_back(camera);
    Camera cam{camera_folder_path.str(), "F1",
               json_config_["params"]["intrinsics_path"]};
    cameras_.push_back(cam);
  }

  map_file_name_ = json_config_["params"]["map_path"];
  poses_file_name_ = json_config_["params"]["poses_path"];
  extrinsics_file_name_ = json_config_["params"]["extrinsics_path"];
  path_to_camera_calib_ = json_config_["params"]["intrinsics_path"];

  for (const auto str : cameras_[0].img_paths) {
    std::cout << str << std::endl;
  }
  /**
   * Load PCD file
   */
  if (pcl::io::loadPCDFile<BridgePoint>(map_file_name_, *defect_pointcloud_) ==
      -1) {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
  }

  /**
   * Load previous poses, send these poses to the tf_tree
   */
  LoadPrevPoses();
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

  /**
   * Load camera calibrations
   */
  /*  beam_calibration::Pinhole pinhole;
    pinhole.LoadJSON(path_to_camera_calib_);
    std::cout << pinhole << std::endl;*/

  /**
   * Load Image Containers
   */
  img_bridge_.LoadFromJSON(cameras_[0].img_paths[0]);
  /*  img_bridge_.LoadFromJSON("/home/steve/2019_02_13_19_44_Structures_Lab/images/"
                             "camera0/ImageBridge1/");*/

  DefectCloud::Ptr colored_cloud =
      ProjectImgToMap(img_bridge_, &cameras_[0].cam_intrinsics);

  /*
   DefectCloud::Ptr cloud3 = boost::make_shared<DefectCloud>();

   pcl_ros::transformPointCloud(*colored_cloud, *cloud3, tf_temp_);*/

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::copyPointCloud(*colored_cloud, *cloud_rgb);

  viewer->setBackgroundColor(1, 1, 1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      cloud_rgb);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud_rgb, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud_rgb, rgb, "sample cloud1");

  /*
    viewer->initCameraParameters ();
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0, 0, 0, v1);
    viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerRGBField<BridgePoint>
    rgb(defect_pointcloud_); viewer->addPointCloud<BridgePoint>
    (defect_pointcloud_, rgb, "sample cloud1", v1);*/
}

DefectCloud::Ptr MapLabeler::TransformMapToImageFrame(ros::Time tf_time) {
  /*  geometry_msgs::TransformStamped transform_msg =
        tf2_buffer_.lookupTransform("map", "base_link", tf_time);*/

  std::string to_frame = "F1_link";
  std::string from_frame = "map"; //"F1_link";

  geometry_msgs::TransformStamped transform_msg =
      tf_tree.GetTransform(to_frame, from_frame, tf_time);

  std::cout << "Transform lookup time = " << tf_time << std::endl;
  std::cout << "Transform = " << transform_msg << std::endl;

  auto transformed_cloud = boost::make_shared<DefectCloud>();

  tf::Transform tf_;
  tf::transformMsgToTF(transform_msg.transform, tf_);
  pcl_ros::transformPointCloud(*defect_pointcloud_, *transformed_cloud, tf_);

  tf_temp_ = tf_.inverse();
  std::cout
      << "Successfully transformed point cloud from map frame to image frame..."
      << std::endl;
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

    std::cout << "Adding affine_tf : " << unique_id.str()
              << " with transform: " << g_tf_stamped.transform.translation.x
              << std::endl;

    viewer->addCoordinateSystem(0.5, affine_tf, unique_id.str());
  }
}

DefectCloud::Ptr
    MapLabeler::ProjectImgToMap(beam_containers::ImageBridge img,
                                beam_calibration::Intrinsics* cam_intrinsics) {
  cv::Mat bgr_img = img.GetBGRImage();
  TimePoint img_time = img_bridge_.GetTimePoint();
  ros::Time ros_img_time = TimePointToRosTime(img_time);
  beam::Vec2 img_dims = cam_intrinsics->GetImgDims();
  std::cout << cam_intrinsics->GetImgDims() << std::endl;

  DefectCloud::Ptr map_cloud = TransformMapToImageFrame(ros_img_time);
  std::cout << "Map cloud size = " << map_cloud->width << std::endl;
  DefectCloud::Ptr colored_cloud = boost::make_shared<DefectCloud>();

  int count = 0;
  for (auto& point : *map_cloud) {
    if (point.z < 0) continue;

    beam::Vec3 point_coords{point.x, point.y, point.z};
    beam::Vec2 vec2 = cam_intrinsics->ProjectDistortedPoint(point_coords);

    if (vec2[0] < 0 || vec2[1] < 0) continue;
    if (vec2[0] < img_dims[0] && vec2[1] < img_dims[1]) {
      cv::Vec3b pixel_colour =
          bgr_img.at<cv::Vec3b>(cv::Point(vec2[0], vec2[1]));
      BridgePoint colored_point = {point.x, point.y, point.z};
      colored_point.b = (uint8_t)pixel_colour.val[0];
      colored_point.g = (uint8_t)pixel_colour.val[1];
      colored_point.r = (uint8_t)pixel_colour.val[2];
      if (colored_point.b < 10 && colored_point.r < 10 &&
          colored_point.g < 10) {
        continue;
      }
      colored_cloud->push_back(colored_point);
    }
  }

  DefectCloud::Ptr cloud3 = boost::make_shared<DefectCloud>();
  pcl_ros::transformPointCloud(*colored_cloud, *cloud3, tf_temp_);
  return cloud3;
}

} // end namespace inspection
