#define PCL_NO_PRECOMPILE

#include <inspection/MapLabeler.h>

#include <boost/filesystem.hpp>
#include <thread>

#include <tf2_eigen/tf2_eigen.h>

#include <beam_depth/DepthCompletion.h>
#include <beam_depth/DepthMap.h>
#include <beam_mapping/Poses.h>
#include <beam_utils/pointclouds.h>
#include <beam_utils/time.h>

namespace inspection {

void MapLabeler::Inputs::Print() {
  BEAM_INFO("Images directory: {}", images_directory);
  BEAM_INFO("Map file path: {}", map);
  BEAM_INFO("Poses file path: {}", poses);
  BEAM_INFO("Intrinsics root folder: {}", intrinsics_directory);
  BEAM_INFO("Extrinsics file path: {}", extrinsics);
  BEAM_INFO("Config file path: {}", config_file_location);
  BEAM_INFO("Poses moving frame override: {}", poses_moving_frame_override);
}

MapLabeler::MapLabeler(const Inputs& inputs) : inputs_(inputs) {
  ProcessJSONConfig();

  // Load map
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(inputs_.map, *map_) == -1) {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
  }

  // Fill tf tree object with robot poses & extrinsics
  FillTFTrees();

  // fill camera poses using image timestamps and tf trees
  FillCameraPoses();
}

void MapLabeler::RunFullPipeline() const {
  // map cameral_name -> DefectCloudsMap
  std::unordered_map<std::string, DefectCloudsMapType> defect_clouds;
  LabelColor(defect_clouds);
  LabelDefects(defect_clouds);
  CombineClouds(defect_clouds);
}

void MapLabeler::LabelColor(
    std::unordered_map<std::string, DefectCloudsMapType>& defect_clouds) const {
  for (size_t cam_index = 0; cam_index < cameras_.size(); cam_index++) {
    // get defects for this camera
    const Camera& cam = cameras_[cam_index];
    if (defect_clouds.find(cam.name) == defect_clouds.end()) {
      defect_clouds.emplace(cam.name, DefectCloudsMapType());
    }
    const DefectCloudsMapType& camera_defects = defect_clouds.at(cam.name);

    // color each image for this camera
    for (size_t img_index = 0; img_index < cam.images.size(); img_index++) {
      const Image& img = cam.images[img_index];
      int64_t img_time_ns =
          static_cast<int64_t>(img.image_container.GetRosTime().toNSec());
      beam::HighResolutionTimer timer;
      if (camera_defects.find(img_time_ns) == camera_defects.end()) {
        DefectCloud::Ptr labeled_cloud = std::make_shared<DefectCloud>();
        camera_defects.emplace(img_time_ns, labeled_cloud);
      }
      ProjectImgRGBToMap(camera_defects.at(img_time_ns), img, cam);
      BEAM_INFO("[Cam: {}/{}, Image: {}/{}] Finished coloring in {} seconds.",
                cam_index + 1, cameras_.size(), img_index + 1,
                cam.images.size(), timer.elapsed());
    }
  }
}

void MapLabeler::LabelDefects(
    std::unordered_map<std::string, DefectCloudsMapType>& defect_clouds) const {
  for (size_t cam_index = 0; cam_index < cameras_.size(); cam_index++) {
    // get defects for this camera
    const Camera& cam = cameras_[cam_index];
    if (defect_clouds.find(cam.name) == defect_clouds.end()) {
      defect_clouds.emplace(cam.name, DefectCloudsMapType());
    }
    const DefectCloudsMapType& camera_defects = defect_clouds.at(cam.name);

    // color each image for this camera
    for (size_t img_index = 0; img_index < cam.images.size(); img_index++) {
      const Image& img = cam.images[img_index];
      int64_t img_time_ns =
          static_cast<int64_t>(img.image_container.GetRosTime().toNSec());
      beam::HighResolutionTimer timer;
      if (camera_defects.find(img_time_ns) == camera_defects.end()) {
        DefectCloud::Ptr labeled_cloud = std::make_shared<DefectCloud>();
        camera_defects.emplace(img_time_ns, labeled_cloud);
      }
      ProjectImgRGBMaskToMap(camera_defects.at(img_time_ns), img, cam);
      BEAM_INFO(
          "[Cam: {}/{}, Image: {}/{}] Finished labeling mask in {} seconds.",
          cam_index + 1, cameras_.size(), img_index + 1, cam.images.size(),
          timer.elapsed());
    }
  }
}
void MapLabeler::CombineClouds(
    const std::unordered_map<std::string, DefectCloudsMapType>& defect_clouds)
    const {
  std::vector<std::vector<Eigen::Affine3d>> all_transforms;
  for (auto& camera : cameras_) {
    std::vector<Eigen::Affine3d> camera_transforms;
    for (const Image& image : camera.images) {
      camera_transforms.push_back(image.T_MAP_CAMERA);
    }
    all_transforms.push_back(camera_transforms);
  }
  cloud_combiner_.CombineClouds(defect_clouds, all_transforms);
}

void MapLabeler::PrintConfiguration() {
  BEAM_INFO("------------------ Map Labeler Configuration ------------------");
  BEAM_INFO("---------------------------------------------------------------");
  inputs_.Print();

  BEAM_INFO("Colorizer type: {}", colorizer_type_);
  BEAM_INFO("Strategy for combining clouds: {}", cloud_combiner_type_);
  BEAM_INFO("Saving final map to: {}", final_map_name_);
  BEAM_INFO("Run depth enhancement: {}", depth_enhancement_ ? "True" : "False");

  BEAM_INFO("Map number of points: {}", map_->size());
  BEAM_INFO("Number of cameras: {}", cameras_.size());
  int counter = 0;
  for (const auto& cam : cameras_) {
    BEAM_INFO("Camera: {} info...", counter++);
    BEAM_INFO("   Cam Name: {}", cam.name);
    BEAM_INFO("   Intrinsics path: {}", cam.intrinsics_path);
    BEAM_INFO("   Camera frame: {}", cam.cam_model->GetFrameID());
    BEAM_INFO("   Number of images: {}", cam.images.size());
  }
  BEAM_INFO(
      "--------------------------------------------------------------- \n {}",
      "\n");
}

void MapLabeler::DrawFinalMap() {
  using namespace std::literals::chrono_literals;

  PointCloudXYZRGB::Ptr rgb_pc = std::make_shared<PointCloudXYZRGB>();
  pcl::copyPointCloud(*cloud_combiner_.GetCombinedCloud(), *rgb_pc);

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      rgb_pc);

  pcl::visualization::PCLVisualizer::Ptr viewer =
      std::make_shared<pcl::visualization::PCLVisualizer>();
  viewer->addPointCloud<pcl::PointXYZRGB>(rgb_pc, rgb, "Final");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Final");

  viewer->setBackgroundColor(1, 1, 1);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(100ms);
  }
}

void MapLabeler::ProcessJSONConfig() {
  BEAM_DEBUG("Processing MapLabeler json config file from : {} ...",
             inputs_.config_file_location);
  nlohmann::json J;
  if (!beam::ReadJson(inputs_.config_file_location, J)) {
    throw std::runtime_error{"invalid config file path"};
  }

  nlohmann::json cameras_json;
  try {
    depth_enhancement_ = J.at("depth_enhancement");
    final_map_name_ = J.at("final_map_name");
    cloud_combiner_type_ = J.at("cloud_combiner");
    colorizer_type_ = J.at("colorizer");
    nlohmann::json tmp = J.at("cameras");
    cameras_json = tmp;
  } catch (nlohmann::json::exception& e) {
    BEAM_CRITICAL("Error processing JSON file: Message {}, ID: {}", e.what(),
                  e.id);
  }

  BEAM_DEBUG("Images path: {}", inputs_.images_directory);
  BEAM_DEBUG("Map path: {}", inputs_.map);
  BEAM_DEBUG("Poses path: {}", inputs_.poses);
  BEAM_DEBUG("Extrinsics: {}", inputs_.extrinsics);
  BEAM_DEBUG("Intrinsics folder: {}", inputs_.intrinsics_directory);
  BEAM_DEBUG("Colorizer type: {}", colorizer_type_);
  BEAM_DEBUG("Creating {} camera objects for labeling...", cameras_json.size());

  cameras_ = LoadCameras(cameras_json, inputs_.images_directory,
                         inputs_.intrinsics_directory, colorizer_type_);

  BEAM_INFO("Successfully loaded config");
}

void MapLabeler::FillTFTrees() {
  // Load previous poses file specified in labeler json
  beam_mapping::Poses poses_container;
  poses_container.LoadFromJSON(inputs_.poses);
  const auto& final_poses = poses_container.GetPoses();
  const auto& final_timestamps = poses_container.GetTimeStamps();
  poses_fixed_frame_ = poses_container.GetFixedFrame();
  if (inputs_.poses_moving_frame_override.empty()) {
    poses_moving_frame_ = poses_container.GetMovingFrame();
  } else {
    BEAM_INFO("overriding moving frame in pose. Changing from {} to {}",
              poses_container.GetMovingFrame(),
              inputs_.poses_moving_frame_override);
    poses_moving_frame_ = inputs_.poses_moving_frame_override;
  }

  BEAM_INFO("Filling TF tree with {} poses", final_poses.size());
  ros::Time start_time = final_timestamps.front();
  poses_tree_.start_time = start_time;
  for (int i = 0; i < final_poses.size(); i++) {
    poses_tree_.AddTransform(Eigen::Affine3d(final_poses[i]),
                             poses_fixed_frame_, poses_moving_frame_,
                             final_timestamps[i]);
  }

  BEAM_DEBUG("Filling TF tree with extrinsic transforms");
  extinsics_tree_.LoadJSON(inputs_.extrinsics);
}

void MapLabeler::SaveLabeledClouds(const std::string& output_folder) {
  BEAM_INFO("Saving labeled individual clouds to: {}", output_folder);

  for (size_t cam = 0; cam < defect_clouds_.size(); cam++) {
    int cloud_number = 1;
    for (const auto& cloud : defect_clouds_[cam]) {
      std::string file_name =
          cameras_[cam].name + "_" + std::to_string(cloud_number) + ".pcd";
      if (cloud->points.size() > 0) {
        pcl::io::savePCDFileBinary(output_folder + "/" + file_name, *cloud);
        cloud_number++;
      }
    }

    BEAM_DEBUG("Saved {} labeled clouds from Camera: {}", cloud_number - 1,
               cameras_[cam].name);
  }
}

void MapLabeler::SaveFinalMap(const std::string& output_folder) {
  std::string labeled_map_path = output_folder + "/labeled_map.pcd";
  pcl::io::savePCDFileBinary(labeled_map_path,
                             *cloud_combiner_.GetCombinedCloud());
  BEAM_DEBUG("Saved combined labeled cloud to: {}", labeled_map_path);
}

void MapLabeler::SaveCameraPoses(const std::string& output_folder) {
  BEAM_INFO("Saving camera poses to: {}", output_folder);
  std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> camera_poses_clouds;
  std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> baselink_poses_clouds;

  for (const Camera& cam : cameras_) {
    // get extrinsics
    std::string cam_frame_id = cam.cam_model->GetFrameID();
    Eigen::Matrix4d T_CAM_BASELINK =
        extinsics_tree_.GetTransformEigen(cam_frame_id, poses_moving_frame_)
            .matrix();

    auto camera_frames = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBL>>();
    auto baselink_frames =
        std::make_shared<pcl::PointCloud<pcl::PointXYZRGBL>>();
    for (int image_num = 0; image_num < cam.images.size(); image_num++) {
      Eigen::Matrix4d T_WORLD_CAM =
          cam.images.at(image_num).T_MAP_CAMERA.matrix();
      Eigen::Matrix4d T_WORLD_BASELINK = T_WORLD_CAM * T_CAM_BASELINK;

      // draw camera frames
      pcl::PointCloud<pcl::PointXYZRGBL> new_frame = beam::CreateFrameCol(
          cam.images.at(image_num).image_container.GetRosTime());
      beam::MergeFrameToCloud(*camera_frames, new_frame, T_WORLD_CAM);

      // add camera frustums
      pcl::PointCloud<pcl::PointXYZRGBL> frustum =
          cam.cam_model->CreateCameraFrustum(
              cam.images.at(image_num).image_container.GetRosTime(),
              draw_points_increment_, frustum_lengh_);
      beam::MergeFrameToCloud(*camera_frames, frustum, T_WORLD_CAM);

      // draw baselink frames
      beam::MergeFrameToCloud(*baselink_frames, new_frame, T_WORLD_BASELINK);
    }
    camera_poses_clouds.push_back(camera_frames);
    baselink_poses_clouds.push_back(baselink_frames);
  }

  for (int poses_counter = 0; poses_counter < camera_poses_clouds.size();
       poses_counter++) {
    std::string filename =
        cameras_.at(poses_counter).name + "_poses_in_camera_frame.pcd";
    std::string save_path = output_folder + "/" + filename;
    pcl::io::savePCDFileBinary(save_path,
                               *camera_poses_clouds.at(poses_counter));
    BEAM_INFO("Saved camera poses file to: {}", save_path);
    std::string filename2 =
        cameras_.at(poses_counter).name + "_poses_in_baselink_frame.pcd";
    std::string save_path2 = output_folder + "/" + filename2;
    pcl::io::savePCDFileBinary(save_path2,
                               *baselink_poses_clouds.at(poses_counter));
    BEAM_INFO("Saved image baselink poses file to: {}", save_path2);
  }
}

void MapLabeler::SaveImages(const std::string& output_folder) {
  BEAM_INFO("Saving images used for labeling to: {}", output_folder);
  for (int cam_num = 0; cam_num < cameras_.size(); cam_num++) {
    const Camera& cam = cameras_.at(cam_num);
    for (int image_num = 0; image_num < cam.images.size(); image_num++) {
      std::string dir = output_folder + "/cam" + std::to_string(cam_num) +
                        "_img" + std::to_string(image_num);
      boost::filesystem::create_directories(dir);
      cam.images.at(image_num).image_container.Write(dir);
    }
  }
}

void MapLabeler::FillCameraPoses() {
  for (auto& camera : cameras_) {
    camera.FillPoses(extinsics_tree_, poses_tree_, poses_moving_frame_,
                     poses_fixed_frame_);
  }
}

void MapLabeler::ProjectImgRGBToMap(DefectCloud::Ptr& defect_cloud,
                                    const Image& image, const Camera& camera) {
  if (!image.image_container.IsBGRImageSet()) {
    BEAM_DEBUG("no BGR image available for cam {}, image time: {}s",
               camera.name, image.image_container.GetRosTime().toSec());
    return;
  }
  BEAM_DEBUG("Projecting image to map");

  // Get map in camera frame
  DefectCloud::Ptr map_in_camera_frame = std::make_shared<DefectCloud>();
  pcl::transformPointCloud(*defect_cloud, *map_in_camera_frame,
                           image.T_MAP_CAMERA.inverse());


  // Color point cloud with BGR images
  DefectCloud::Ptr labeled_cloud_in_camera_frame =
      std::make_shared<DefectCloud>();
  const auto& image_container = image.image_container;
  if (image_container.IsBGRImageSet()) {
    BEAM_DEBUG("Setting BGR image in colorizer");
    camera.colorizer->SetDistortion(image_container.GetBGRIsDistorted());
    camera.colorizer->SetImage(image_container.GetBGRImage());

    // Get colored cloud & remove uncolored points
    BEAM_DEBUG("Coloring point cloud");

    auto xyzrgb_cloud = camera.colorizer->ColorizePointCloud(map_in_camera_frame);
    BEAM_DEBUG("Finished colorizing point cloud");
    xyzrgb_cloud->points.erase(
        std::remove_if(xyzrgb_cloud->points.begin(), xyzrgb_cloud->points.end(),
                       [](auto& point) {
                         return (point.r == 0 && point.g == 0 && point.b == 0);
                       }),
        xyzrgb_cloud->points.end());
    xyzrgb_cloud->width = xyzrgb_cloud->points.size();
    BEAM_DEBUG("Labeled cloud size: {}", xyzrgb_cloud->points.size());
    pcl::copyPointCloud(*xyzrgb_cloud, *labeled_cloud_in_camera_frame);
  }
}

void MapLabeler::ProjectImgRGBMaskToMap(DefectCloud::Ptr& defect_cloud,
                                        const Image& image,
                                        const Camera& camera) {
  if (image.image_container.IsBGRMaskSet()) {
    BEAM_DEBUG("no BGR image mask available for cam {}, image time: {}s",
               camera.name, image.image_container.GetRosTime().toSec());
    return;
  }

  BEAM_DEBUG("Projecting image mask to map");
  // Enhance defects with depth completion
  if (depth_enhancement_) {
    BEAM_DEBUG("Performing depth map extraction.");
    beam::HighResolutionTimer timer;
    std::shared_ptr<beam_depth::DepthMap> dm =
        std::make_shared<beam_depth::DepthMap>(camera.cam_model,
                                               map_in_camera_frame);
    dm->ExtractDepthMap(depth_map_extraction_thresh_);
    cv::Mat depth_image = dm->GetDepthImage();
    BEAM_DEBUG("Time elapsed: {}", timer.elapsed());

    BEAM_DEBUG("Removing Sparse defect points.");
    std::vector<beam_containers::PointBridge,
                Eigen::aligned_allocator<beam_containers::PointBridge>>
        new_points;
    cv::Mat defect_mask = image_container.GetBGRMask();
    for (auto& p : labeled_cloud_in_camera_frame->points) {
      Eigen::Vector3d point(p.x, p.y, p.z);
      bool in_image = false;
      Eigen::Vector2d coords;
      if (!camera.cam_model->ProjectPoint(point, coords, in_image) ||
          !in_image) {
        new_points.push_back(p);
      }

      uint16_t col = coords(0, 0);
      uint16_t row = coords(1, 0);
      if (defect_mask.at<uchar>(row, col) == 0) { new_points.push_back(p); }
    }
    labeled_cloud_in_camera_frame->points = new_points;

    BEAM_DEBUG("Performing depth completion.");
    cv::Mat depth_image_dense = depth_image.clone();
    beam_depth::IPBasic(depth_image_dense);

    BEAM_DEBUG("Adding dense defect points.");
    for (int row = 0; row < depth_image_dense.rows; row++) {
      for (int col = 0; col < depth_image_dense.cols; col++) {
        if (defect_mask.at<uchar>(row, col) != 0) {
          double depth = depth_image_dense.at<double>(row, col);
          Eigen::Vector2i pixel(col, row);
          Eigen::Vector3d ray;
          if (!camera.cam_model->BackProject(pixel, ray)) { continue; }
          ray = ray.normalized();
          Eigen::Vector3d point = ray * depth;
          beam_containers::PointBridge point_bridge;
          point_bridge.x = point[0];
          point_bridge.y = point[1];
          point_bridge.z = point[2];
          labeled_cloud_in_camera_frame->push_back(point_bridge);
        }
      }
    }
  }

  // Color point cloud with Mask
  camera.colorizer->SetImage(image_container.GetBGRMask());

  // Get labeled cloud & remove unlabeled points
  DefectCloud::Ptr labeled_cloud = camera.colorizer->ColorizeMask();
  labeled_cloud->points.erase(
      std::remove_if(labeled_cloud->points.begin(), labeled_cloud->points.end(),
                     [](auto& point) {
                       return (point.crack == 0 && point.delam == 0 &&
                               point.corrosion == 0);
                     }),
      labeled_cloud->points.end());
  labeled_cloud->width = labeled_cloud->points.size();

  // Now we need to go back into the final defect cloud and
  // label the points that have defects
  pcl::search::KdTree<BridgePoint> kdtree;
  kdtree.setInputCloud(labeled_cloud_in_camera_frame);
  for (const auto& search_point : *labeled_cloud) {
    std::vector<int> nn_point_ids(1);
    std::vector<float> nn_point_distances(1);
    if (kdtree.nearestKSearch(search_point, 1, nn_point_ids,
                              nn_point_distances) > 0) {
      labeled_cloud_in_camera_frame->points[nn_point_ids[0]].crack +=
          search_point.crack;
      labeled_cloud_in_camera_frame->points[nn_point_ids[0]].delam +=
          search_point.delam;
      labeled_cloud_in_camera_frame->points[nn_point_ids[0]].corrosion +=
          search_point.corrosion;
    }
  }

  // Transform the cloud back into the map frame
  DefectCloud::Ptr labeled_cloud_in_map_frame = std::make_shared<DefectCloud>();
  pcl::transformPointCloud(*labeled_cloud_in_camera_frame,
                           *labeled_cloud_in_map_frame, image.T_MAP_CAMERA);
  return labeled_cloud_in_map_frame;
}

void MapLabeler::OutputSummary(const std::string& output_folder) {
  std::string json_file_name = output_folder + "/summary.json";
  BEAM_INFO("Saving labeling summary to: {}", json_file_name);
  std::vector<nlohmann::json> cameraJsons;
  for (int cam_iter = 0; cam_iter < cameras_.size(); cam_iter++) {
    const Camera& camera = cameras_[cam_iter];
    // fill image jsons
    std::vector<nlohmann::json> imageJsons;
    for (int img_iter = 0; img_iter < cameras_[cam_iter].images.size();
         img_iter++) {
      DefectCloud::Ptr defect_cloud = defect_clouds_.at(cam_iter).at(img_iter);
      DefectCloudStats stats = GetDefectCloudStats(defect_cloud);
      nlohmann::json statsJson;
      statsJson["size"] = stats.size;
      statsJson["cracks"] = stats.cracks;
      statsJson["delams"] = stats.delams;
      statsJson["spalls"] = stats.spalls;
      statsJson["corrosions"] = stats.corrosions;

      nlohmann::json imageJson;
      imageJson["defect_statistics"] = statsJson;

      const Image& image = camera.images[img_iter];
      imageJson["image_info_path"] = image.image_info_path;
      imageJson["bgr_image_set"] = image.image_container.IsBGRImageSet();
      imageJson["bgr_mask_set"] = image.image_container.IsBGRMaskSet();
      imageJson["ir_image_set"] = image.image_container.IsIRImageSet();
      imageJson["ir_mask_set"] = image.image_container.IsIRMaskSet();
      imageJson["timestamp_in_s"] = image.image_container.GetRosTime().toSec();
      imageJsons.push_back(imageJson);
    }

    // fill camera json
    nlohmann::json cameraJson;
    cameraJson["name"] = camera.name;
    cameraJson["intrinsics_path"] = camera.intrinsics_path;
    cameraJson["frame_id"] = camera.cam_model->GetFrameID();
    cameraJson["images"] = imageJsons;
    cameraJsons.push_back(cameraJson);
  }

  nlohmann::json J;
  J["cameras"] = cameraJsons;
  std::ofstream filejson(json_file_name);
  filejson << std::setw(4) << J << std::endl;
}

DefectCloudStats
    MapLabeler::GetDefectCloudStats(const DefectCloud::Ptr& cloud) {
  DefectCloudStats stats;
  stats.size = cloud->size();
  for (auto cloud_iter = cloud->begin(); cloud_iter != cloud->end();
       cloud_iter++) {
    if (cloud_iter->crack > 0) { stats.cracks++; }
    if (cloud_iter->corrosion > 0) { stats.corrosions++; }
    if (cloud_iter->spall > 0) { stats.spalls++; }
    if (cloud_iter->delam > 0) { stats.delams++; }
  }
  return stats;
}

void MapLabeler::OutputConfig(const std::string& output_folder) {
  std::string config_file_output = output_folder + "/config_copy.json";
  BEAM_INFO("Saving config to: {}", config_file_output);
  std::ifstream src(inputs_.config_file_location, std::ios::binary);
  std::ofstream dst(config_file_output, std::ios::binary);
  dst << src.rdbuf();
}

} // end namespace inspection
