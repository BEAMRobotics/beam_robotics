#include <inspection/CameraToMapAligner.h>

#include <beam_mapping/Poses.h>

namespace inspection {

CameraToMapAligner::CameraToMapAligner(const Inputs& inputs) : inputs_(inputs) {
  // Load map
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(inputs_.map, *map_) == -1) {
    BEAM_ERROR("Couldn't read file map pcd file: {}", inputs_.map);
  }

  // Fill tf tree object with poses & extrinsics
  FillTFTrees();

  // load intrinsics
  camera_model_ = beam_calibration::CameraModel::Create(inputs_.intrinsics);
}

void CameraToMapAligner::FillTFTrees() {
  // Load previous poses file specified in labeler json
  beam_mapping::Poses poses_container;
  poses_container.LoadFromFile(inputs_.poses);
  const auto& poses = poses_container.GetPoses();
  const auto& timestamps = poses_container.GetTimeStamps();
  poses_fixed_frame_ = poses_container.GetFixedFrame();
  if (inputs_.poses_moving_frame_override.empty()) {
    poses_moving_frame_ = poses_container.GetMovingFrame();
  } else {
    BEAM_INFO("overriding moving frame in pose. Changing from {} to {}",
              poses_container.GetMovingFrame(),
              inputs_.poses_moving_frame_override);
    poses_moving_frame_ = inputs_.poses_moving_frame_override;
  }

  BEAM_INFO("Filling TF tree with {} poses", poses.size());
  ros::Time start_time = timestamps.front();
  poses_tree_.start_time = start_time;
  for (int i = 0; i < poses.size(); i++) {
    poses_tree_.AddTransform(Eigen::Affine3d(poses[i]), poses_fixed_frame_,
                             poses_moving_frame_, timestamps[i]);
  }

  BEAM_DEBUG("Filling TF tree with extrinsic transforms");
  extinsics_original_.LoadJSON(inputs_.extrinsics);
  extinsics_edited_ = extinsics_original_;
}

void CameraToMapAligner::Run() {
  LoadImageContainer();
  AddFixedCoordinateSystems();
  while (!viewer_->wasStopped()) {
    if (GetUserInput()) {
      UpdateExtrinsics();
      UpdateMap();
      UpdateViewer();
    }
    viewer_->spinOnce(100);
  }
}

void CameraToMapAligner::AddFixedCoordinateSystems() {
  // add map coordinate frame
  viewer->addCoordinateSystem(coordinateFrameScale_,
                              poses_fixed_frame_ + " (Map)");

  // add reference coordinate frame
  Eigen::Matrix4f T_map_reference =
      (T_map_moving_ * T_moving_reference_).cast<float>();
  viewer->addCoordinateSystem(coordinateFrameScale_,
                              Eigen::Affine3f(T_map_reference),
                              inputs_.reference_frame + " (Ref)");

  // add original camera coordinate frame
  Eigen::Matrix4f T_map_camera =
      (T_map_moving_ * T_moving_reference_ * T_reference_camera_).cast<float>();
  viewer->addCoordinateSystem(coordinateFrameScale_,
                              Eigen::Affine3f(T_map_camera),
                              inputs_.reference_frame + "CameraFrameOrig");
}

void CameraToMapAligner::LoadImageContainer() {
  // todo: set T_map_moving_ and T_moving_reference_ and T_reference_camera_
}

bool CameraToMapAligner::GetUserInput() {
  // todo
}

void CameraToMapAligner::UpdateExtrinsics() {
  // calculate T_reference_camera_
}

void CameraToMapAligner::UpdateMap() {
  // todo
}

void CameraToMapAligner::UpdateViewer() {
  // add point cloud
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      map_colored_);
  viewer->removeAllPointClouds();
  viewer->addPointCloud<PointTypeCol>(map_colored_, rgb, "Map");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Map");
  viewer->setBackgroundColor(1, 1, 1);
  
  // Update coordinate frame
  Eigen::Matrix4f T_map_camera = (T_map_moving_ * T_moving_reference_ * T_reference_camera_).cast<float>();
  viewer->removeCoordinateSystem("CameraFrameUpdated");
  viewer->addCoordinateSystem(coordinateFrameScale_, Eigen::Affine3f(T_map_camera), "CameraFrameUpdated");


  viewer->initCameraParameters();
}

// todo move content to UpdateMap
DefectCloud::Ptr CameraToMapAligner::ProjectImgToMap(const Image& image,
                                                     const Camera& camera) {
  BEAM_DEBUG("Projecting image to map");

  // Get map in camera frame
  PointCloud::Ptr map_in_camera_frame = std::make_shared<PointCloud>();
  pcl::transformPointCloud(*map_, *map_in_camera_frame,
                           image.T_MAP_CAMERA.inverse());

  // Set up the camera colorizer with the point cloud which is being labeled
  BEAM_DEBUG("Setting map cloud in colorizer");

  camera.colorizer->SetPointCloud(map_in_camera_frame);

  DefectCloud::Ptr labeled_cloud_in_camera_frame =
      std::make_shared<DefectCloud>();

  // Color point cloud with BGR images
  const auto& image_container = image.image_container;
  if (image_container.IsBGRImageSet()) {
    BEAM_DEBUG("Setting BGR image in colorizer");
    camera.colorizer->SetDistortion(image_container.GetBGRIsDistorted());
    camera.colorizer->SetImage(image_container.GetBGRImage());

    // Get colored cloud & remove uncolored points
    BEAM_DEBUG("Coloring point cloud");

    auto xyzrgb_cloud = camera.colorizer->ColorizePointCloud();
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

  // Enhance defects with depth completion
  if (image_container.IsBGRMaskSet() && depth_enhancement_) {
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
  if (image_container.IsBGRMaskSet()) {
    camera.colorizer->SetImage(image_container.GetBGRMask());

    // Get labeled cloud & remove unlabeled points
    DefectCloud::Ptr labeled_cloud = camera.colorizer->ColorizeMask();
    labeled_cloud->points.erase(std::remove_if(labeled_cloud->points.begin(),
                                               labeled_cloud->points.end(),
                                               [](auto& point) {
                                                 return (point.crack == 0 &&
                                                         point.delam == 0 &&
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
  }

  // Transform the cloud back into the map frame
  DefectCloud::Ptr labeled_cloud_in_map_frame = std::make_shared<DefectCloud>();
  pcl::transformPointCloud(*labeled_cloud_in_camera_frame,
                           *labeled_cloud_in_map_frame, image.T_MAP_CAMERA);
  return labeled_cloud_in_map_frame;
}

} // end namespace inspection
