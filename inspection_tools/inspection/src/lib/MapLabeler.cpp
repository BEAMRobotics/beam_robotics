#include "inspection/MapLabeler.h"
#include <beam_containers/Utilities.h>
#include <beam_utils/time.hpp>
#include <boost/filesystem.hpp>

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
  final_poses_ = beam_containers::ReadPoseFile(poses_file_name_);

  // Fill tf tree object with robot poses & extrinsics
  FillTFTree();

  /**
   * Load Image Containers
   */
  //  std::vector<int> indices = {0, 5, 10};
  std::vector<int> indices(50);
  std::iota(indices.begin(), indices.end(), 0);

  int num_cams = cameras_.size();
  defect_clouds_.resize(num_cams);
  for (size_t cam = 0; cam < num_cams; cam++) {
    int number_of_images = cameras_[cam].img_paths.size();
    for (size_t image_index = 0; image_index < number_of_images;
         image_index++) {
      beam::HighResolutionTimer timer;

      //      std::cout << cameras_[cam].img_paths[image_index] << std::endl;
      img_bridge_.LoadFromJSON(cameras_[cam].img_paths[image_index]);
      Camera* camera = &(cameras_[cam]);
      DefectCloud::Ptr colored_cloud = ProjectImgToMap(img_bridge_, camera);
      //      std::cout << colored_cloud->points.size() << std::endl;
      defect_clouds_[cam].push_back(colored_cloud);

      std::cout << "Elapsed time: " << timer.elapsed() << std::endl;

      PointCloudXYZRGB::Ptr cloud_rgb = boost::make_shared<PointCloudXYZRGB>();
      pcl::copyPointCloud(*colored_cloud, *cloud_rgb);

      rgb_clouds.push_back(cloud_rgb);
    }
  }

  cloud_combiner_.CombineClouds(defect_clouds_);
}

void MapLabeler::DrawFinalMap() {
  int id = 0;
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
      rgb_field;

  PointCloudXYZRGB::Ptr rgb_pc = boost::make_shared<PointCloudXYZRGB>();
  pcl::copyPointCloud(*cloud_combiner_.GetCombinedCloud(), *rgb_pc);

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      rgb_pc);

  viewer->addPointCloud<pcl::PointXYZRGB>(rgb_pc, rgb, "Final");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Final");

  viewer->setBackgroundColor(1, 1, 1);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
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
  //  std::cout << json_cameras_list << std::endl;

  // Now we create Camera objects for each camera defined in the
  // CamerasList.json file.
  for (const auto& camera_name : json_cameras_list["Items"]) {
    std::string camera_folder_path =
        images_file_name_ + "/" + std::string(camera_name);
    //    std::cout << camera_folder_path << std::endl;

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
  using namespace boost::filesystem;
  std::string root_cloud_folder = images_file_name_ + "/../clouds";
  boost::filesystem::path path = root_cloud_folder;
  if (!is_directory(path)) {
    std::cout << "No cloud folder in root, creating cloud folder" << std::endl;
    create_directories(path);
  }

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

  pcl::io::savePCDFileBinary(root_cloud_folder + "/_final.pcd",
                             *cloud_combiner_.GetCombinedCloud());
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
  std::string from_frame = "map";

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

void MapLabeler::PlotFrames(std::string frame_id, PCLViewer viewer) {
  std::vector<Eigen::Affine3f> coord_frames;

  for (const auto& pose : final_poses_) {
    // beam::chronoToRosTime(pose.first);
    ros::Time time = TimePointToRosTime(pose.first); //
    std::string to_frame = "map";
    geometry_msgs::TransformStamped g_tf_stamped =
        tf_tree.GetTransform(to_frame, frame_id, time);

    Eigen::Affine3d eig = tf2::transformToEigen(g_tf_stamped);
    Eigen::Affine3f affine_tf(eig.cast<float>());

    std::stringstream unique_id;
    unique_id << frame_id << "_" << time.sec;

    /*    std::cout << "Adding affine_tf : " << unique_id.str()
                  << " with transform: " << g_tf_stamped.transform.translation.x
                  << std::endl;*/

    viewer->addCoordinateSystem(0.5, affine_tf, unique_id.str());
  }
}

DefectCloud::Ptr
    MapLabeler::ProjectImgToMap(beam_containers::ImageBridge img_container,
                                Camera* camera) {
  TimePoint img_time = img_bridge_.GetTimePoint();
  ros::Time ros_img_time = TimePointToRosTime(img_time);
  beam::Vec2 img_dims = camera->cam_intrinsics->GetImgDims();

  // Get map in camera frame
  DefectCloud::Ptr map_cloud = TransformMapToImageFrame(
      ros_img_time, camera->cam_intrinsics->GetFrameId());
  auto xyz_cloud = boost::make_shared<PointCloudXYZ>();
  pcl::copyPointCloud(*map_cloud, *xyz_cloud);
  camera->colorizer->SetPointCloud(xyz_cloud);

  DefectCloud::Ptr return_cloud = boost::make_shared<DefectCloud>();

  if (img_container.IsBGRImageSet()) {
    cv::Mat bgr_img = img_container.GetBGRImage();
    camera->colorizer->SetImage(bgr_img);

    // Get colored cloud & remove uncolored points

    auto xyzrgb_cloud = camera->colorizer->ColorizePointCloud();
    //    xyzrgb_cloud = camera->colorizer->ColorizePointCloud();
    xyzrgb_cloud->points.erase(
        std::remove_if(xyzrgb_cloud->points.begin(), xyzrgb_cloud->points.end(),
                       [](auto& point) {
                         return (point.r == 0 && point.g == 0 && point.b == 0);
                       }),
        xyzrgb_cloud->points.end());

    xyzrgb_cloud->width = xyzrgb_cloud->points.size();
    std::cout << "XYZRGB cloud size = " << xyzrgb_cloud->width << std::endl;
    std::cout << "XYZRGB points size = " << xyzrgb_cloud->points.size()
              << std::endl;

    DefectCloud::Ptr colored_cloud = boost::make_shared<DefectCloud>();
    pcl::copyPointCloud(*xyzrgb_cloud, *return_cloud);
  }
  if (img_container.IsBGRMaskSet()) {
    cv::Mat bgr_mask = img_container.GetBGRMask();
  }

  pcl_ros::transformPointCloud(*return_cloud, *return_cloud, tf_temp_);

  return return_cloud;
}

} // end namespace inspection
