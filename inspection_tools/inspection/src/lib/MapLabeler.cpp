#define PCL_NO_PRECOMPILE

#include <inspection/MapLabeler.h>

#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <beam_depth/DepthCompletion.h>
#include <beam_depth/DepthMap.h>
#include <beam_mapping/Poses.h>
#include <beam_utils/pointclouds.h>
#include <beam_utils/time.h>
#include <inspection/CloudCombiner.h>

namespace inspection {

void MapLabeler::Inputs::Print() const {
  BEAM_INFO("Images directory: {}", images_directory);
  BEAM_INFO("Map file path: {}", map);
  BEAM_INFO("Poses file path: {}", poses);
  BEAM_INFO("Intrinsics root folder: {}", intrinsics_directory);
  BEAM_INFO("Extrinsics file path: {}", extrinsics);
  BEAM_INFO("Config file path: {}", config_file_location);
  BEAM_INFO("Poses moving frame override: {}", poses_moving_frame_override);
  BEAM_INFO(
      "Defect detector precisions: crack {}, corrosion {}, spall {} delam {}",
      crack_detector_precision, corrosion_detector_precision,
      spall_detector_precision, delam_detector_precision);
}

MapLabeler::MapLabeler(const Inputs& inputs) : inputs_(inputs) {
  BEAM_INFO("Initializing MapLabeler");
  ProcessJSONConfig();

  if (!inputs_.map.empty()) {
    if (pcl::io::loadPCDFile<BridgePoint>(inputs_.map, *input_map_) == -1) {
      BEAM_CRITICAL("Couldn't read input map: {}", inputs_.map);
      throw std::runtime_error{"unable to read input map"};
    } else {
      BEAM_WARN("No input map provided, you must call "
                "GetDefectClouds(path_to_clouds)");
    }
  }

  // fill point ids in cloud
  for (uint64_t id = 0; id < input_map_->size(); id++){
    input_map_->at(id).map_point_id = id;
  }

  FillTFTrees();
  FillCameraPoses();
  BEAM_INFO("Done initializing MapLabeler");
}

DefectCloud::Ptr MapLabeler::RunFullPipeline(const std::string& output_folder,
                                             bool save_labeled_clouds,
                                             bool output_summary) const {
  BEAM_INFO("Running full MapLabeler pipeline");
  std::unordered_map<std::string, DefectCloudsMapType> defect_clouds_in_cam =
      GetDefectClouds();
  LabelColor(defect_clouds_in_cam, false);
  LabelDefects(defect_clouds_in_cam, true);
  DefectCloud::Ptr final_map =
      CombineClouds(defect_clouds_in_cam, output_folder);
  if (save_labeled_clouds && !output_folder.empty()) {
    SaveLabeledClouds(defect_clouds_in_cam, output_folder);
  }
  if (output_summary && !output_folder.empty()) {
    OutputSummary(defect_clouds_in_cam, output_folder);
  }

  BEAM_INFO("Done running MapLabeler pipeline");
  return final_map;
}

std::unordered_map<std::string, DefectCloudsMapType>
    MapLabeler::ReadDefectClouds(const std::string& defect_clouds_json) const {
  nlohmann::json J;
  if (!beam::ReadJson(defect_clouds_json, J)) {
    BEAM_CRITICAL("Cannot read json, exiting");
    throw std::runtime_error{"Cannot read json"};
  }

  boost::filesystem::path path(defect_clouds_json);
  std::string clouds_dir = path.parent_path().string();

  std::unordered_map<std::string, std::vector<nlohmann::json>> cameras_map = J;
  std::unordered_map<std::string, DefectCloudsMapType> defect_clouds_in_cam;
  for (const auto& [cam_name, map_info_jsons] : cameras_map) {
    const Camera& cam = GetCameraByName(cam_name);
    DefectCloudsMapType camera_clouds;
    for (const nlohmann::json& map_info_json : map_info_jsons) {
      std::string filename = map_info_json["filename"];
      int64_t timestamp_ns = map_info_json["timestamp_ns"];
      std::string filepath = beam::CombinePaths(clouds_dir, filename);
      DefectCloud cloud_in_map;
      if (pcl::io::loadPCDFile<BridgePoint>(filepath, cloud_in_map) == -1) {
        BEAM_CRITICAL("Couldn't read input cloud: {}", filepath);
        throw std::runtime_error{"unable to read input cloud"};
      }

      const Eigen::Affine3d& T_MAP_CAMERA =
          cam.GetImageByTimestamp(timestamp_ns).T_MAP_CAMERA;
      DefectCloud::Ptr cloud_in_cam = std::make_shared<DefectCloud>();
      pcl::transformPointCloud(cloud_in_map, *cloud_in_cam,
                               T_MAP_CAMERA.inverse());
      camera_clouds.emplace(timestamp_ns, cloud_in_cam);
    }
    defect_clouds_in_cam.emplace(cam_name, camera_clouds);
  }
  return defect_clouds_in_cam;
}

std::unordered_map<std::string, DefectCloudsMapType>
    MapLabeler::GetDefectClouds() const {
  if (input_map_->empty()) {
    BEAM_CRITICAL("Cannot get defect clouds with empty map");
    throw std::runtime_error("empty map");
  }

  std::unordered_map<std::string, DefectCloudsMapType> defect_clouds_in_cam;
  for (size_t cam_index = 0; cam_index < cameras_.size(); cam_index++) {
    const Camera& cam = cameras_[cam_index];

    // create dummy projection to check map points viewable by each image
    beam_colorize::Projection projection_colorizer;
    projection_colorizer.SetIntrinsics(cam.cam_model);

    DefectCloudsMapType camera_defects;
    for (size_t img_index = 0; img_index < cam.images.size(); img_index++) {
      const auto& image = cam.images[img_index];
      const auto& image_container = image.image_container;
      projection_colorizer.SetDistortion(image_container.GetBGRIsDistorted());

      int64_t time_in_ns = image_container.GetRosTime().toNSec();
      DefectCloud::Ptr cloud_in_cam = GetMapVisibleByCam(
          projection_colorizer, image.T_MAP_CAMERA.inverse());
      camera_defects.emplace(time_in_ns, cloud_in_cam);
    }
    defect_clouds_in_cam.emplace(cam.name, camera_defects);
  }
  return defect_clouds_in_cam;
}

DefectCloud::Ptr
    MapLabeler::GetMapVisibleByCam(const beam_colorize::Projection& projection,
                                   const Eigen::Affine3d& T_CAMERA_MAP) const {
  if (input_map_->empty()) {
    BEAM_CRITICAL("cannot call GetMapVisibleByCam with empty input map");
    throw std::runtime_error{"cannot call function"};
  }

  // Get map in camera frame
  DefectCloud::Ptr map_in_camera_frame = std::make_shared<DefectCloud>();
  pcl::transformPointCloud<BridgePoint>(*input_map_, *map_in_camera_frame,
                                        T_CAMERA_MAP);

  // create projection map
  beam_colorize::ProjectionMap projection_map =
      projection.CreateProjectionMap(map_in_camera_frame);

  // create visible map
  DefectCloud::Ptr visible_map_in_cam = std::make_shared<DefectCloud>();
  for (auto v_iter = projection_map.VBegin(); v_iter != projection_map.VEnd();
       v_iter++) {
    for (auto u_iter = v_iter->second.begin(); u_iter != v_iter->second.end();
         u_iter++) {
      visible_map_in_cam->push_back(map_in_camera_frame->at(u_iter->second.id));
    }
  }

  return visible_map_in_cam;
}

void MapLabeler::LabelColor(
    std::unordered_map<std::string, DefectCloudsMapType>& defect_clouds_in_cam,
    bool remove_unlabeled) const {
  for (size_t cam_index = 0; cam_index < cameras_.size(); cam_index++) {
    // get defects for this camera
    const Camera& cam = cameras_[cam_index];
    if (defect_clouds_in_cam.find(cam.name) == defect_clouds_in_cam.end()) {
      BEAM_ERROR("no camera with the name \"{}\" in defect clouds", cam.name);
      continue;
    }
    DefectCloudsMapType& camera_defects = defect_clouds_in_cam.at(cam.name);

    // color each image for this camera
    for (size_t img_index = 0; img_index < cam.images.size(); img_index++) {
      const Image& img = cam.images[img_index];
      int64_t img_time_ns =
          static_cast<int64_t>(img.image_container.GetRosTime().toNSec());
      beam::HighResolutionTimer timer;
      if (camera_defects.find(img_time_ns) == camera_defects.end()) {
        BEAM_ERROR(
            "no image with timestamp {}Ns in defect clouds for camera {}",
            img_time_ns, cam.name);
        continue;
      }
      ProjectImgRGBToMap(camera_defects.at(img_time_ns), img, cam,
                         remove_unlabeled);
      BEAM_INFO("[Cam: {}/{}, Image: {}/{}] Finished coloring in {} seconds.",
                cam_index + 1, cameras_.size(), img_index + 1,
                cam.images.size(), timer.elapsed());
    }
  }
}

void MapLabeler::LabelDefects(
    std::unordered_map<std::string, DefectCloudsMapType>& defect_clouds_in_cam,
    bool remove_unlabeled) const {
  for (size_t cam_index = 0; cam_index < cameras_.size(); cam_index++) {
    // get defects for this camera
    const Camera& cam = cameras_[cam_index];
    if (defect_clouds_in_cam.find(cam.name) == defect_clouds_in_cam.end()) {
      BEAM_ERROR("no camera with the name \"{}\" in defect clouds", cam.name);
      continue;
    }
    DefectCloudsMapType& camera_defects = defect_clouds_in_cam.at(cam.name);

    // color each image for this camera
    for (size_t img_index = 0; img_index < cam.images.size(); img_index++) {
      const Image& img = cam.images[img_index];
      int64_t img_time_ns =
          static_cast<int64_t>(img.image_container.GetRosTime().toNSec());
      beam::HighResolutionTimer timer;
      if (camera_defects.find(img_time_ns) == camera_defects.end()) {
        BEAM_ERROR(
            "no image with timestamp {}Ns in defect clouds for camera {}",
            img_time_ns, cam.name);
        continue;
      }
      ProjectImgRGBMaskToMap(camera_defects.at(img_time_ns), img, cam,
                             remove_unlabeled);
      BEAM_INFO(
          "[Cam: {}/{}, Image: {}/{}] Finished labeling mask in {} seconds.",
          cam_index + 1, cameras_.size(), img_index + 1, cam.images.size(),
          timer.elapsed());
    }
  }
}

DefectCloud::Ptr MapLabeler::CombineClouds(
    const std::unordered_map<std::string, DefectCloudsMapType>&
        defect_clouds_in_cam,
    const std::string& output_dir) const {
  BEAM_INFO("Combining maps");
  std::unordered_map<std::string, TransformMapType> transforms;
  for (const auto& camera : cameras_) {
    TransformMapType camera_transforms;
    for (const Image& image : camera.images) {
      int64_t time_in_ns = image.image_container.GetRosTime().toNSec();
      camera_transforms.emplace(time_in_ns, image.T_MAP_CAMERA);
    }
    transforms.emplace(camera.name, camera_transforms);
  }
  inspection::CloudCombiner cloud_combiner(
      inputs_.crack_detector_precision, inputs_.corrosion_detector_precision,
      inputs_.spall_detector_precision, inputs_.delam_detector_precision);
  cloud_combiner.CombineClouds(defect_clouds_in_cam, transforms);
  BEAM_INFO("Done combining maps");
  cloud_combiner.OutputStatistics(
      beam::CombinePaths(output_dir, "cloud_combiner_stats.json"));
  return cloud_combiner.GetCombinedCloud();
}

void MapLabeler::PrintConfiguration() const {
  BEAM_INFO("------------------ Map Labeler Configuration ------------------");
  BEAM_INFO("---------------------------------------------------------------");
  inputs_.Print();

  BEAM_INFO("Colorizer type: {}", colorizer_type_);
  BEAM_INFO("Strategy for combining clouds: {}", cloud_combiner_type_);
  BEAM_INFO("Saving final map to: {}", final_map_name_);
  BEAM_INFO("Run depth enhancement: {}", depth_enhancement_ ? "True" : "False");

  BEAM_INFO("Map number of input map points: {}", input_map_->size());
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

void MapLabeler::DrawFinalMap(const DefectCloud::Ptr& map) const {
  using namespace std::literals::chrono_literals;
  BEAM_INFO("Displaying final map");

  PointCloudXYZRGB::Ptr rgb_pc = std::make_shared<PointCloudXYZRGB>();
  pcl::copyPointCloud(*map, *rgb_pc);

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
  BEAM_INFO("Done displaying map");
}

void MapLabeler::ProcessJSONConfig() {
  BEAM_INFO("Processing MapLabeler json config file from : {} ...",
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

  BEAM_INFO("Images path: {}", inputs_.images_directory);
  BEAM_INFO("Map path: {}", inputs_.map);
  BEAM_INFO("Poses path: {}", inputs_.poses);
  BEAM_INFO("Extrinsics: {}", inputs_.extrinsics);
  BEAM_INFO("Intrinsics folder: {}", inputs_.intrinsics_directory);
  BEAM_INFO("Colorizer type: {}", colorizer_type_);
  BEAM_INFO("Creating {} camera objects for labeling...", cameras_json.size());

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

void MapLabeler::SaveLabeledClouds(
    const std::unordered_map<std::string, DefectCloudsMapType>&
        defect_clouds_in_cam,
    const std::string& output_folder) const {
  std::string save_path = beam::CombinePaths(output_folder, "labeled_clouds");
  boost::filesystem::create_directories(save_path);
  BEAM_INFO("Saving labeled individual clouds to: {}", save_path);
  std::string json_file_name =
      beam::CombinePaths(save_path, "labeled_clouds_info.json");

  nlohmann::json J;
  for (const auto& [cam_name, cam_defects] : defect_clouds_in_cam) {
    nlohmann::json camera_json;
    const Camera& cam = GetCameraByName(cam_name);
    for (const auto& [timestamp, cloud_in_cam] : cam_defects) {
      if (cloud_in_cam->empty()) {
        BEAM_WARN("empty cloud detected (cam: {}, timestamp: {}) not "
                  "outputting cloud");
        continue;
      }

      // transform pointcloud into map frame
      const Eigen::Affine3d& T_MAP_CAMERA =
          cam.GetImageByTimestamp(timestamp).T_MAP_CAMERA;
      DefectCloud::Ptr cloud_in_map = std::make_shared<DefectCloud>();
      pcl::transformPointCloud(*cloud_in_cam, *cloud_in_map, T_MAP_CAMERA);

      // save cloud
      std::string file_name =
          cam_name + "_" + std::to_string(timestamp) + ".pcd";
      std::string filepath = beam::CombinePaths(save_path, file_name);
      nlohmann::json cloud_json;
      cloud_json["timestamp_ns"] = timestamp;
      cloud_json["filename"] = file_name;
      camera_json.push_back(cloud_json);
      pcl::io::savePCDFileBinary(filepath, *cloud_in_map);
    }
    J[cam_name] = camera_json;
    BEAM_DEBUG("Saved {} labeled clouds from Camera: {}",
               defect_clouds_in_cam.size(), cam_name);
  }

  BEAM_DEBUG("Saving labeled clouds info json to: {}", json_file_name);
  std::ofstream filejson(json_file_name);
  filejson << std::setw(4) << J << std::endl;
}

void MapLabeler::SaveFinalMap(const DefectCloud::Ptr& map,
                              const std::string& output_folder) const {
  std::string labeled_map_path =
      beam::CombinePaths(output_folder, "labeled_map.pcd");
  pcl::io::savePCDFileBinary(labeled_map_path, *map);
  BEAM_INFO("Saved combined labeled cloud to: {}", labeled_map_path);
}

void MapLabeler::SaveCameraPoses(const std::string& output_folder) const {
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
    std::string save_path = beam::CombinePaths(output_folder, filename);
    pcl::io::savePCDFileBinary(save_path,
                               *camera_poses_clouds.at(poses_counter));
    BEAM_INFO("Saved camera poses file to: {}", save_path);
    std::string filename2 =
        cameras_.at(poses_counter).name + "_poses_in_baselink_frame.pcd";
    std::string save_path2 = beam::CombinePaths(output_folder, filename2);
    pcl::io::savePCDFileBinary(save_path2,
                               *baselink_poses_clouds.at(poses_counter));
    BEAM_INFO("Saved image baselink poses file to: {}", save_path2);
  }
}

void MapLabeler::SaveImages(const std::string& output_folder) const {
  BEAM_INFO("Saving images used for labeling to: {}", output_folder);
  for (int cam_num = 0; cam_num < cameras_.size(); cam_num++) {
    const Camera& cam = cameras_.at(cam_num);
    for (int image_num = 0; image_num < cam.images.size(); image_num++) {
      std::string filename =
          "cam" + std::to_string(cam_num) + "_img" + std::to_string(image_num);
      std::string dir = beam::CombinePaths(output_folder, filename);
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

void MapLabeler::ProjectImgRGBToMap(DefectCloud::Ptr& defect_cloud_in_cam,
                                    const Image& image, const Camera& camera,
                                    bool remove_unlabeled) const {
  const auto& image_container = image.image_container;
  if (!image_container.IsBGRImageSet()) {
    BEAM_DEBUG("no BGR image available for cam {}, image time: {}s",
               camera.name, image.image_container.GetRosTime().toSec());
    return;
  }
  BEAM_DEBUG("Projecting image to map");

  // Color point cloud with BGR images
  camera.colorizer->SetDistortion(image_container.GetBGRIsDistorted());
  camera.colorizer->SetImage(image_container.GetBGRImage());
  camera.colorizer->ColorizePointCloud(defect_cloud_in_cam);

  if (!remove_unlabeled) { return; }

  defect_cloud_in_cam->points.erase(
      std::remove_if(defect_cloud_in_cam->points.begin(),
                     defect_cloud_in_cam->points.end(),
                     [](auto& point) {
                       return (point.r == 0 && point.g == 0 && point.b == 0,
                               point.crack == 0, point.spall == 0,
                               point.corrosion == 0, point.delam == 0,
                               point.thermal == 0);
                     }),
      defect_cloud_in_cam->points.end());
}

void MapLabeler::ProjectImgRGBMaskToMap(DefectCloud::Ptr& defect_cloud_in_cam,
                                        const Image& image,
                                        const Camera& camera,
                                        bool remove_unlabeled) const {
  const auto& image_container = image.image_container;
  if (image_container.IsBGRMaskSet()) {
    BEAM_DEBUG("no BGR image mask available for cam {}, image time: {}s",
               camera.name, image.image_container.GetRosTime().toSec());
    return;
  }
  BEAM_DEBUG("Projecting image mask to map");

  // Enhance defects with depth completion
  if (depth_enhancement_) {
    // copy point cloud and transform to camera frame
    PointCloud::Ptr xyz_cloud_in_cam = std::make_shared<PointCloud>();
    pcl::copyPointCloud(*defect_cloud_in_cam, *xyz_cloud_in_cam);

    BEAM_DEBUG("Performing depth map extraction.");
    beam::HighResolutionTimer timer;
    std::shared_ptr<beam_depth::DepthMap> dm =
        std::make_shared<beam_depth::DepthMap>(camera.cam_model,
                                               xyz_cloud_in_cam);
    dm->ExtractDepthMapProjectionNoOcclusions();
    cv::Mat depth_image = dm->GetDepthImage();
    BEAM_DEBUG("Time elapsed: {}", timer.elapsed());

    BEAM_DEBUG("Performing depth completion.");
    cv::Mat depth_image_dense = depth_image.clone();
    beam_depth::IPBasic(depth_image_dense);
    dm->SetDepthImage(depth_image_dense);
    pcl::PointCloud<pcl::PointXYZ>::Ptr comp_cloud = dm->ExtractPointCloud();

    BEAM_DEBUG("Adding depth completed defect points to defect cloud.");
    cv::Mat defect_mask = image_container.GetBGRMask();
    for (const auto& p : comp_cloud->points) {
      Eigen::Vector3d p_in_cam(p.x, p.y, p.z);
      bool in_image = false;
      Eigen::Vector2d coords;
      if (!camera.cam_model->ProjectPoint(p_in_cam, coords, in_image) ||
          !in_image || defect_mask.at<uchar>(coords[1], coords[0]) == 0) {
        continue;
      }
      beam_containers::PointBridge point_bridge;
      point_bridge.x = p_in_cam[0];
      point_bridge.y = p_in_cam[1];
      point_bridge.z = p_in_cam[2];
      defect_cloud_in_cam->push_back(point_bridge);
    }
  }

  // Color point cloud with Mask
  camera.colorizer->SetDistortion(image_container.GetBGRIsDistorted());
  camera.colorizer->SetImage(image_container.GetBGRMask());
  camera.colorizer->ColorizeMask(defect_cloud_in_cam);

  if (!remove_unlabeled) { return; }

  defect_cloud_in_cam->points.erase(
      std::remove_if(defect_cloud_in_cam->points.begin(),
                     defect_cloud_in_cam->points.end(),
                     [](auto& point) {
                       return (point.r == 0 && point.g == 0 && point.b == 0,
                               point.crack == 0, point.spall == 0,
                               point.corrosion == 0, point.delam == 0,
                               point.thermal == 0);
                     }),
      defect_cloud_in_cam->points.end());
}

void MapLabeler::OutputSummary(
    const std::unordered_map<std::string, DefectCloudsMapType>&
        defect_clouds_in_cam,
    const std::string& output_folder) const {
  std::string json_file_name =
      beam::CombinePaths(output_folder, "summary.json");
  BEAM_INFO("Saving labeling summary to: {}", json_file_name);
  std::vector<nlohmann::json> cameraJsons;
  for (const auto& [cam_name, cam_defects] : defect_clouds_in_cam) {
    const Camera& camera = GetCameraByName(cam_name);

    // fill image jsons
    std::vector<nlohmann::json> imageJsons;
    for (const auto& [timestamp, defect_cloud_in_cam] : cam_defects) {
      DefectCloudStats stats = GetDefectCloudStats(defect_cloud_in_cam);
      nlohmann::json statsJson;
      statsJson["size"] = stats.size;
      statsJson["cracks"] = stats.cracks;
      statsJson["delams"] = stats.delams;
      statsJson["spalls"] = stats.spalls;
      statsJson["corrosions"] = stats.corrosions;

      nlohmann::json imageJson;
      imageJson["defect_statistics"] = statsJson;

      const Image& image = camera.GetImageByTimestamp(timestamp);
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
    MapLabeler::GetDefectCloudStats(const DefectCloud::Ptr& cloud) const {
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

void MapLabeler::OutputConfig(const std::string& output_folder) const {
  std::string config_file_output =
      beam::CombinePaths(output_folder, "config_copy.json");
  BEAM_INFO("Saving config to: {}", config_file_output);
  std::ifstream src(inputs_.config_file_location, std::ios::binary);
  std::ofstream dst(config_file_output, std::ios::binary);
  dst << src.rdbuf();
}

const Camera&
    MapLabeler::GetCameraByName(const std::string& camera_name) const {
  for (const Camera& cam : cameras_) {
    if (cam.name == camera_name) { return cam; }
  }
  BEAM_CRITICAL("no camera found by the name {}", camera_name);
  throw std::runtime_error{"no camera found"};
}
} // end namespace inspection
