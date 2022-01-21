#define PCL_NO_PRECOMPILE

#include <inspection/MapLabeler.h>

#include <thread>

#include <boost/filesystem.hpp>
#include <tf2_eigen/tf2_eigen.h>

#include <beam_mapping/Poses.h>
#include <beam_utils/time.h>

namespace inspection {

MapLabeler::Camera::Camera(const nlohmann::json& camera_config_json,
                           const std::string& cam_imgs_folder,
                           const std::string& intrin_folder)
    : camera_name_(camera_config_json.at("Name")),
      cam_imgs_folder_(cam_imgs_folder),
      cam_intrinsics_path_(intrin_folder + camera_name_ + ".json") {
  BEAM_DEBUG("Creating camera: {}", camera_name_);

  cam_model_ = beam_calibration::CameraModel::Create(cam_intrinsics_path_);

  // Next we create/fill in a string vector which will store the path to
  // each image folder for our camera (this is used for instantiating image
  // container objects)
  boost::filesystem::path p{cam_imgs_folder_};
  BEAM_DEBUG("    Getting image paths for camera...");
  for (const auto& imgs : camera_config_json.at("Images")) {
    std::string img_type = imgs.at("Type");
    for (const auto& ids : imgs.at("IDs")) {
      if (ids == "All") {
        for (auto& entry : boost::make_iterator_range(
                 boost::filesystem::directory_iterator(p), {})) {
          std::string path = entry.path().string();
          img_paths_.emplace_back(path);
          BEAM_DEBUG("      Adding path: {}", path);
        }
      } else {
        img_paths_.emplace_back(cam_imgs_folder_ + "/" + img_type +
                                std::string(ids));
        camera_pose_ids_.push_back(std::stoi(std::string(ids)));
        BEAM_DEBUG("      Adding path: {}", img_paths_.back());
      }
    }
  }
  std::sort(img_paths_.begin(), img_paths_.end());
  BEAM_DEBUG("    Total image paths: {}", img_paths_.size());

  if (camera_config_json.at("Colorizer") == "Projection") {
    colorizer_ = beam_colorize::Colorizer::Create(
        beam_colorize::ColorizerType::PROJECTION);
    colorizer_type_ = "Projection";
  } else if (camera_config_json.at("Colorizer") == "RayTrace") {
    colorizer_ = beam_colorize::Colorizer::Create(
        beam_colorize::ColorizerType::RAY_TRACE);
    colorizer_type_ = "RayTrace";
  }
  BEAM_DEBUG("    Creating {} colorizer object", colorizer_type_);

  colorizer_->SetIntrinsics(cam_model_);
  colorizer_->SetDistortion(true);
  BEAM_DEBUG("    Sucessfully constructed camera: {}!", camera_name_);
}

MapLabeler::MapLabeler(const std::string& images_directory,
                       const std::string& map, const std::string& poses,
                       const std::string& intrinsics_directory,
                       const std::string& extrinsics,
                       const std::string& config_file_location) {
  images_folder_ = images_directory;
  map_path_ = map;
  poses_path_ = poses;
  extrinsics_path_ = extrinsics;
  json_labeler_filepath_ = config_file_location;

  BEAM_INFO("Loading config from: {}", config_file_location);

  // Process core json file
  ProcessJSONConfig();

  // Load map
  if (pcl::io::loadPCDFile<BridgePoint>(map_path_, *defect_pointcloud_) == -1) {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
  }

  // Load previous poses file specified in labeler json
  beam_mapping::Poses poses_container;
  poses_container.LoadFromJSON(poses_path_);
  final_poses_ = poses_container.GetPoses();
  final_timestamps_ = poses_container.GetTimeStamps();
}

void MapLabeler::Run() {
  // Fill tf tree object with robot poses & extrinsics
  FillTFTree();

  // Main loop for labeling
  int num_cams = cameras_.size();
  defect_clouds_.resize(num_cams);
  for (size_t cam = 0; cam < num_cams; cam++) {
    int num_images = cameras_[cam].img_paths_.size();
    for (size_t img_index = 0; img_index < num_images; img_index++) {
      beam::HighResolutionTimer timer;
      img_bridge_.LoadFromJSON(cameras_[cam].img_paths_[img_index]);
      Camera* camera = &(cameras_[cam]);
      DefectCloud::Ptr colored_cloud = ProjectImgToMap(img_bridge_, camera);
      defect_clouds_[cam].push_back(colored_cloud);

      BEAM_INFO("[Cam: {}/{}, Image: {}/{}] Finished coloring in {} seconds.",
                cam + 1, num_cams, img_index + 1, num_images, timer.elapsed());

      PointCloudXYZRGB::Ptr cloud_rgb = std::make_shared<PointCloudXYZRGB>();
      pcl::copyPointCloud(*colored_cloud, *cloud_rgb);

      rgb_clouds_.push_back(cloud_rgb);
    }
  }
  FillCameraPoses();
  std::vector<std::vector<Eigen::Affine3f>> transforms;
  for (auto& camera : cameras_) { transforms.push_back(camera.transforms_); }
  cloud_combiner_.CombineClouds(defect_clouds_, transforms);
}

void MapLabeler::PrintConfiguration() {
  BEAM_INFO("------------------ Map Labeler Configuration ------------------");
  BEAM_INFO("---------------------------------------------------------------");
  BEAM_INFO("Using poses file: {}", poses_path_);
  BEAM_INFO("Number of poses: {}", final_poses_.size());
  BEAM_INFO("Using point cloud map: {}", map_path_);
  BEAM_INFO("Map number of points: {}", defect_pointcloud_->size());
  BEAM_INFO("Strategy for combining clouds: {}", cloud_combiner_type_);
  BEAM_INFO("Saving final map to: {}", final_map_name_);
  BEAM_INFO("Saving individual labeled clouds: {}",
            output_individual_clouds_ ? "True" : "False");
  BEAM_INFO("Number of cameras: {}", cameras_.size());
  BEAM_INFO("Sensor extrinsics: {}", extrinsics_path_);
  for (size_t cam = 0; cam < cameras_.size(); cam++) {
    BEAM_INFO("Camera: {} info...", cam);
    BEAM_INFO("   Cam ID: {}", cameras_[cam].camera_name_);
    BEAM_INFO("   Camera frame: {}", cameras_[cam].cam_model_->GetFrameID());
    BEAM_INFO("   Camera colorizer: {}", cameras_[cam].colorizer_type_);
    BEAM_INFO("   Number of images: {}", cameras_[cam].img_paths_.size());
  }
  BEAM_INFO(
      "--------------------------------------------------------------- \n {}",
      "\n");
}

void MapLabeler::DrawFinalMap() {
  int id = 0;
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
      rgb_field;

  PointCloudXYZRGB::Ptr rgb_pc = std::make_shared<PointCloudXYZRGB>();
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
  try {
    BEAM_DEBUG("Processing MapLabeler json config file from : {} ...",
               json_labeler_filepath_);

    std::ifstream json_config_stream(json_labeler_filepath_);
    json_config_stream >> json_config_;

    output_individual_clouds_ = json_config_.at("output_individual_clouds");
    depth_enhancement_ = json_config_.at("depth_enhancement");
    if (!json_config_.at("final_map_name").empty())
      final_map_name_ = json_config_.at("final_map_name");
    if (!json_config_.at("cloud_combiner").empty())
      cloud_combiner_type_ = json_config_.at("cloud_combiner");
    nlohmann::json cameras_json = json_config_.at("cameras");

    BEAM_DEBUG("MapLabeler JSON - Images path: {}", images_folder_);
    BEAM_DEBUG("MapLabeler JSON - Map path: {}", map_path_);
    BEAM_DEBUG("MapLabeler JSON - Poses path: {}", poses_path_);
    BEAM_DEBUG("MapLabeler JSON - Extrinsics: {}", extrinsics_path_);
    BEAM_DEBUG("MapLabeler JSON - Intrinsics folder: {}", intrinsics_folder_);
    BEAM_DEBUG("Creating {} camera objects for labeling...",
               cameras_json.size());

    for (const auto& camera : cameras_json) {
      if (!camera.at("Enabled")) continue;
      std::string camera_name = camera.at("Name");
      std::string cam_imgs_folder = images_folder_ + "/" + camera_name;
      Camera cam{camera, cam_imgs_folder, intrinsics_folder_};
      cameras_.push_back(std::move(cam));
    }

  } catch (nlohmann::json::exception& e) {
    BEAM_CRITICAL("Error processing JSON file: Message {}, ID: {}", e.what(),
                  e.id);
  }
  BEAM_INFO("Successfully loaded config");
}

void MapLabeler::FillTFTree() {
  ros::Time start_time = final_timestamps_.front();
  ros::Time end_time = final_timestamps_.back();
  tf_tree_.start_time = start_time;
  ros::Duration dur = end_time - start_time;

  BEAM_DEBUG("Filling TF Tree - Poses start time: {}",
             std::to_string(start_time.toSec()));
  BEAM_DEBUG("Filling TF Tree - Poses end time: {}",
             std::to_string(end_time.toSec()));
  BEAM_DEBUG("Filling TF Tree - Poses duration: {}", dur.toSec());

  BEAM_DEBUG("Filling TF tree with {} dynamic transforms (i.e., poses)",
             final_poses_.size());
  geometry_msgs::TransformStamped tf_msg;
  for (int i = 0; i < final_poses_.size(); i++) {
    tf_msg = tf2::eigenToTransform(Eigen::Affine3d(final_poses_[i]));
    tf_msg.header.stamp = final_timestamps_[i];
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "hvlp_link";
    tf_tree_.AddTransform(tf_msg);
  }

  BEAM_DEBUG("Filling TF tree with extrinsic transforms");
  tf_tree_.LoadJSON(extrinsics_path_);
}

void MapLabeler::SaveLabeledClouds() {
  using namespace boost::filesystem;
  std::string root_cloud_folder = images_folder_ + "/../clouds";
  BEAM_INFO("Saving labeled clouds to: {}", root_cloud_folder);

  boost::filesystem::path path = root_cloud_folder;
  if (!is_directory(path)) {
    BEAM_INFO("No cloud folder in {}, creating...", root_cloud_folder);
    create_directories(path);
  }

  for (size_t cam = 0; cam < defect_clouds_.size(); cam++) {
    int cloud_number = 1;
    for (const auto& cloud : defect_clouds_[cam]) {
      std::string file_name = cameras_[cam].camera_name_ + "_" +
                              std::to_string(cloud_number) + ".pcd";
      if (cloud->points.size() > 0) {
        pcl::io::savePCDFileBinary(root_cloud_folder + "/" + file_name, *cloud);
        cloud_number++;
      }
    }

    BEAM_DEBUG("Saved {} labeled clouds from Camera: {}", cloud_number - 1,
               cameras_[cam].camera_name_);
  }

  std::string labeled_map_path = root_cloud_folder + "/_labeled_map.pcd";
  pcl::io::savePCDFileBinary(labeled_map_path,
                             *cloud_combiner_.GetCombinedCloud());
  BEAM_DEBUG("Saved combined labeled cloud to: {}", labeled_map_path);
}

DefectCloud::Ptr MapLabeler::TransformMapToImageFrame(ros::Time tf_time,
                                                      std::string frame_id) {
  std::string to_frame = frame_id;
  std::string from_frame = "map";

  geometry_msgs::TransformStamped transform_msg =
      tf_tree_.GetTransformROS(to_frame, from_frame, tf_time);

  auto transformed_cloud = std::make_shared<DefectCloud>();

  tf::Transform tf_;
  tf::transformMsgToTF(transform_msg.transform, tf_);
  pcl_ros::transformPointCloud(*defect_pointcloud_, *transformed_cloud, tf_);

  tf_temp_ = tf_.inverse();
  BEAM_DEBUG("Transformed map cloud from map frame to image frame: {}",
             frame_id);

  return transformed_cloud;
}

void MapLabeler::PlotFrames(std::string frame_id, PCLViewer viewer) {
  std::vector<Eigen::Affine3f> coord_frames;
  for (ros::Time time : final_timestamps_) {
    std::string to_frame = "map";
    geometry_msgs::TransformStamped g_tf_stamped =
        tf_tree_.GetTransformROS(to_frame, frame_id, time);

    Eigen::Affine3d eig = tf2::transformToEigen(g_tf_stamped);
    Eigen::Affine3f affine_tf(eig.cast<float>());

    std::stringstream unique_id;
    unique_id << frame_id << "_" << time.sec;

    viewer->addCoordinateSystem(0.5, affine_tf, unique_id.str());
  }
}

void MapLabeler::FillCameraPoses() {
  std::string to_frame = "map";
  for (auto& camera : cameras_) {
    std::string cam_frame = camera.cam_model_->GetFrameID();
    if (camera.camera_pose_ids_.size() > 0) {
      // if config file specifies specific poses then only add those
      for (const auto& pose_id : camera.camera_pose_ids_) {
        ros::Time time = final_timestamps_[pose_id - 1];
        geometry_msgs::TransformStamped g_tf_stamped =
            tf_tree_.GetTransformROS(to_frame, cam_frame, time);
        Eigen::Affine3d eig = tf2::transformToEigen(g_tf_stamped);
        Eigen::Affine3f affine_tf(eig.cast<float>());
        camera.transforms_.push_back(affine_tf);
      }
    } else {
      // if no poses are specified, add every pose
      for (ros::Time time : final_timestamps_) {
        geometry_msgs::TransformStamped g_tf_stamped =
            tf_tree_.GetTransformROS(to_frame, cam_frame, time);
        Eigen::Affine3d eig = tf2::transformToEigen(g_tf_stamped);
        Eigen::Affine3f affine_tf(eig.cast<float>());
        camera.transforms_.push_back(affine_tf);
      }
    }
  }
}

DefectCloud::Ptr
    MapLabeler::ProjectImgToMap(beam_containers::ImageBridge img_container,
                                Camera* camera) {
  BEAM_DEBUG("Projecting image to map");
  ros::Time ros_img_time = beam::ChronoToRosTime(img_bridge_.GetTimePoint());

  // Get map in camera frame
  DefectCloud::Ptr map_cloud =
      TransformMapToImageFrame(ros_img_time, camera->cam_model_->GetFrameID());
  auto xyz_cloud = std::make_shared<PointCloudXYZ>();
  pcl::copyPointCloud(*map_cloud, *xyz_cloud);

  // Set up the camera colorizer with the point cloud which is being labeled
  BEAM_DEBUG("Setting map cloud in colorizer");

  camera->colorizer_->SetPointCloud(xyz_cloud);

  DefectCloud::Ptr return_cloud = std::make_shared<DefectCloud>();

  // Color point cloud with BGR images
  if (img_container.IsBGRImageSet()) {
    BEAM_DEBUG("Setting BGR image in colorizer");
    camera->colorizer_->SetImage(img_container.GetBGRImage());

    // Get colored cloud & remove uncolored points
    BEAM_DEBUG("Coloring point cloud");

    auto xyzrgb_cloud = camera->colorizer_->ColorizePointCloud();
    BEAM_DEBUG("Finished colorizing point cloud");
    xyzrgb_cloud->points.erase(
        std::remove_if(xyzrgb_cloud->points.begin(), xyzrgb_cloud->points.end(),
                       [](auto& point) {
                         return (point.r == 0 && point.g == 0 && point.b == 0);
                       }),
        xyzrgb_cloud->points.end());
    xyzrgb_cloud->width = xyzrgb_cloud->points.size();
    BEAM_DEBUG("Labeled cloud size: {}", xyzrgb_cloud->points.size());

    pcl::copyPointCloud(*xyzrgb_cloud, *return_cloud);
  }

  // Enhance defects with depth completion
  if (img_container.IsBGRMaskSet() && depth_enhancement_) {
    BEAM_DEBUG("Performing depth map extraction.");
    std::shared_ptr<beam_cv::DepthMap> dm =
        std::make_shared<beam_cv::DepthMap>(camera->cam_model_, xyz_cloud);
    dm->ExtractDepthMap(0.025);
    cv::Mat depth_image = dm->GetDepthImage();

    BEAM_DEBUG("Removing Sparse defect points.");
    std::vector<beam_containers::PointBridge> new_points;
    cv::Mat defect_mask = img_container.GetBGRMask();
    for (auto& p : return_cloud->points) {
      Eigen::Vector3d point(p.x, p.y, p.z);
      bool in_image = false;
      Eigen::Vector2d coords;
      if (!intrinsics_->ProjectPoint(point, coords, in_image) || !in_image) {
        new_points.push_back(p);
      }

      uint16_t col = coords(0, 0);
      uint16_t row = coords(1, 0);
      if (defect_mask->at<uchar>(row, col) == 0) { new_points.push_back(p); }
    }
    return_cloud->points = new_points;

    BEAM_DEBUG("Performing depth completion.");
    cv::Mat depth_image_dense = depth_image.clone();
    beam_cv::IPBasic(depth_image_dense);

    BEAM_DEBUG("Adding dense defect points.");
    for (int row = 0; row < depth_image_dense->rows; row++) {
      for (int col = 0; col < depth_image_dense->cols; col++) {
        if (defect_mask->at<uchar>(row, col) != 0) {
          double depth = depth_image_dense.at<double>(row, col);
          Eigen::Vector2i pixel(col, row);
          Eigen::Vector3d ray;
          if (!model_->BackProject(pixel, ray)) { continue; }
          ray = ray.normalized();
          Eigen::Vector3d point = ray * depth;
          beam_containers::PointBridge point_bridge;
          point_bridge.x = point[0];
          point_bridge.y = point[1];
          point_bridge.z = point[2];
          return_cloud->points.push_back(point_bridge);
        }
      }
    }
  }

  // Color point cloud with Mask
  if (img_container.IsBGRMaskSet()) {
    camera->colorizer_->SetImage(img_container.GetBGRMask());

    // Get labeled cloud & remove unlabeled points
    DefectCloud::Ptr labeled_cloud = camera->colorizer_->ColorizeMask();
    labeled_cloud->points.erase(std::remove_if(labeled_cloud->points.begin(),
                                               labeled_cloud->points.end(),
                                               [](auto& point) {
                                                 return (point.crack == 0 &&
                                                         point.delam == 0 &&
                                                         point.corrosion == 0);
                                               }),
                                labeled_cloud->points.end());
    labeled_cloud->width = labeled_cloud->points.size();

    // Now we need to go back into the final defect cloud (return_cloud) and
    // label the points that have defects
    pcl::search::KdTree<BridgePoint> kdtree;
    kdtree.setInputCloud(return_cloud);
    for (const auto& search_point : *labeled_cloud) {
      std::vector<int> nn_point_ids(1);
      std::vector<float> nn_point_distances(1);
      if (kdtree.nearestKSearch(search_point, 1, nn_point_ids,
                                nn_point_distances) > 0) {
        return_cloud->points[nn_point_ids[0]].crack += search_point.crack;
        return_cloud->points[nn_point_ids[0]].delam += search_point.delam;
        return_cloud->points[nn_point_ids[0]].corrosion +=
            search_point.corrosion;
      }
    }
  }

  // Transform the cloud back into the map frame
  pcl_ros::transformPointCloud(*return_cloud, *return_cloud, tf_temp_);

  return return_cloud;
}

} // end namespace inspection
