#define PCL_NO_PRECOMPILE

#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>

#include <beam_containers/Utilities.h>
#include <beam_utils/time.hpp>
#include <boost/filesystem.hpp>
#include <thread>

#include "inspection/MapLabeler.h"

using namespace std::literals::chrono_literals;

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
  if (pcl::io::loadPCDFile<BridgePoint>(map_path_, *defect_pointcloud_) == -1) {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
  }

  // Load previous poses file specified in labeler json
  final_poses_ = beam_containers::ReadPoseFile(poses_path_);
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

      PointCloudXYZRGB::Ptr cloud_rgb = boost::make_shared<PointCloudXYZRGB>();
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
  try {
    BEAM_DEBUG("Processing MapLabeler json config file from : {} ...",
               json_labeler_filepath_);

    std::ifstream json_config_stream(json_labeler_filepath_);
    json_config_stream >> json_config_;

    images_folder_ = json_config_.at("images_folder");
    intrinsics_folder_ = json_config_.at("intrinsics_folder");
    map_path_ = json_config_.at("map_path");
    poses_path_ = json_config_.at("poses_path");
    extrinsics_path_ = json_config_.at("extrinsics_path");
    output_individual_clouds_ = json_config_.at("output_individual_clouds");
    if (!json_config_.at("final_map_name").empty())
      final_map_name_ = json_config_.at("final_map_name");
    if (!json_config_.at("cloud_combiner").empty())
      cloud_combiner_type_ = json_config_.at("cloud_combiner");
    json cameras_json = json_config_.at("cameras");

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

  } catch (json::exception& e) {
    BEAM_CRITICAL("Error processing JSON file: Message {}, ID: {}", e.what(),
                  e.id);
  }
  BEAM_INFO("Successfully loaded config");
}

void MapLabeler::FillTFTree() {
  ros::Time start_time = TimePointToRosTime(final_poses_.front().first);
  ros::Time end_time = TimePointToRosTime(final_poses_.back().first);
  tf_tree_.start_time_ = start_time;
  ros::Duration dur = end_time - start_time;

  BEAM_DEBUG("Filling TF Tree - Poses start time: {}",
             std::to_string(start_time.toSec()));
  BEAM_DEBUG("Filling TF Tree - Poses end time: {}",
             std::to_string(end_time.toSec()));
  BEAM_DEBUG("Filling TF Tree - Poses duration: {}", dur.toSec());

  BEAM_DEBUG("Filling TF tree with {} dynamic transforms (i.e., poses)",
             final_poses_.size());
  geometry_msgs::TransformStamped tf_msg;
  for (const auto& pose : final_poses_) {
    tf_msg = tf2::eigenToTransform(pose.second);
    tf_msg.header.stamp = TimePointToRosTime(pose.first);
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
      pcl::io::savePCDFileBinary(root_cloud_folder + "/" + file_name, *cloud);
      cloud_number++;
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

  auto transformed_cloud = boost::make_shared<DefectCloud>();

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

  for (const auto& pose : final_poses_) {
    ros::Time time = TimePointToRosTime(pose.first);
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
        auto pose = final_poses_[pose_id - 1];
        ros::Time time = TimePointToRosTime(pose.first);
        geometry_msgs::TransformStamped g_tf_stamped =
            tf_tree_.GetTransformROS(to_frame, cam_frame, time);
        Eigen::Affine3d eig = tf2::transformToEigen(g_tf_stamped);
        Eigen::Affine3f affine_tf(eig.cast<float>());
        camera.transforms_.push_back(affine_tf);
      }
    } else {
      // if no poses are specified, add every pose
      for (const auto& pose : final_poses_) {
        ros::Time time = TimePointToRosTime(pose.first);
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
  ros::Time ros_img_time = TimePointToRosTime(img_bridge_.GetTimePoint());

  // Get map in camera frame
  DefectCloud::Ptr map_cloud =
      TransformMapToImageFrame(ros_img_time, camera->cam_model_->GetFrameID());
  auto xyz_cloud = boost::make_shared<PointCloudXYZ>();
  pcl::copyPointCloud(*map_cloud, *xyz_cloud);

  // Set up the camera colorizer with the point cloud which is being labeled
  BEAM_DEBUG("Setting map cloud in colorizer");

  camera->colorizer_->SetPointCloud(xyz_cloud);

  DefectCloud::Ptr return_cloud = boost::make_shared<DefectCloud>();

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
