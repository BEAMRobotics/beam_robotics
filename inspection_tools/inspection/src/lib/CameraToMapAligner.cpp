#include <inspection/CameraToMapAligner.h>

#include <pcl/common/transforms.h>

#include <beam_mapping/Poses.h>

namespace inspection {

CameraToMapAligner::CameraToMapAligner(const Inputs& inputs) : inputs_(inputs) {
  viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(inputs_.map, *map_) == -1) {
    BEAM_ERROR("Couldn't read file map pcd file: {}", inputs_.map);
  }
  camera_model_ = beam_calibration::CameraModel::Create(inputs_.intrinsics);
  LoadImageContainer();
  FillTfTrees();
  AddFixedCoordinateSystems();
  BEAM_INFO("Done initializing CameraToMapAligner");
}

void CameraToMapAligner::LoadImageContainer() {
  BEAM_INFO("Reading image container from json: {}",
            inputs_.image_container_root);
  image_container_.LoadFromJSON(inputs_.image_container_root);
}

void CameraToMapAligner::FillTfTrees() {
  // Load previous poses file specified in labeler json
  BEAM_INFO("Loading poses form {}", inputs_.poses);
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
    Eigen::Affine3d T(poses[i]);
    poses_tree_.AddTransform(T, poses_fixed_frame_, poses_moving_frame_,
                             timestamps[i]);
  }

  BEAM_INFO("Filling TF tree with extrinsic from {}", inputs_.extrinsics);
  beam_calibration::TfTree extinsics;
  extinsics.LoadJSON(inputs_.extrinsics);

  BEAM_INFO("Getting transform from {} (reference) to {} (moving frame) from "
            "extrinsics.",
            inputs_.reference_frame, poses_moving_frame_);
  Eigen::Matrix4d T_moving_reference =
      extinsics.GetTransformEigen(poses_moving_frame_, inputs_.reference_frame)
          .matrix();

  // load image position
  ros::Time image_time = image_container_.GetRosTime();
  BEAM_INFO("Getting transform from {} (moving frame) to {} (map/fixed frame) "
            "at time {}s from poses",
            poses_moving_frame_, poses_fixed_frame_, image_time.toSec());
  Eigen::Matrix4d T_map_moving =
      poses_tree_
          .GetTransformEigen(poses_fixed_frame_, poses_moving_frame_,
                             image_time)
          .matrix();
  T_map_reference_ = T_map_moving * T_moving_reference;

  // set initial transform to be edited
  BEAM_INFO("setting initial transform from {} (camera) to {} (reference)",
            image_container_.GetBGRFrameId(), inputs_.reference_frame);
  T_reference_camera_ = extinsics
                            .GetTransformEigen(inputs_.reference_frame,
                                               image_container_.GetBGRFrameId())
                            .matrix();
  T_reference_camera_original_ = T_reference_camera_;
}

void CameraToMapAligner::AddFixedCoordinateSystems() {
  // add map coordinate frame
  BEAM_INFO("Adding map coordinate system to viewer");
  viewer_->addText3D("MapFrame", pcl::PointXYZ(), text_scale_, text_rgb_[0],
                     text_rgb_[1], text_rgb_[2]);
  Eigen::Affine3f T_identity(Eigen::Matrix4f::Identity());
  viewer_->addCoordinateSystem(coordinateFrameScale_, T_identity,
                               poses_fixed_frame_ + " (Map)");

  // add reference coordinate frame
  BEAM_INFO("Adding reference frame coordinate system to viewer");
  pcl::PointXYZ t(T_map_reference_(0, 3), T_map_reference_(1, 3),
                  T_map_reference_(2, 3));
  viewer_->addText3D("RefFrame", t, text_scale_, text_rgb_[0], text_rgb_[1],
                     text_rgb_[2]);
  viewer_->addCoordinateSystem(coordinateFrameScale_,
                               Eigen::Affine3f(T_map_reference_.cast<float>()),
                               inputs_.reference_frame + " (Ref)");

  // add original camera coordinate frame
  BEAM_INFO("Adding initial camera frame coordinate system to viewer");
  Eigen::Matrix4d T_map_camera = T_map_reference_ * T_reference_camera_;
  t = pcl::PointXYZ(T_map_camera(0, 3), T_map_camera(1, 3), T_map_camera(2, 3));
  viewer_->addText3D("CamFrame", t, text_scale_, text_rgb_[0], text_rgb_[1],
                     text_rgb_[2]);
  viewer_->addCoordinateSystem(coordinateFrameScale_,
                               Eigen::Affine3f(T_map_camera.cast<float>()),
                               inputs_.reference_frame + "CameraFrameOrig");

  BEAM_INFO("Done adding coordinate frames to viewer");
}

void CameraToMapAligner::Run() {
  std::function<void(const pcl::visualization::KeyboardEvent&)> keyboard_cb =
      [this](const pcl::visualization::KeyboardEvent& event) {
        keyboardEventOccurred(event);
      };
  BEAM_INFO("Registering keyboard callback");
  viewer_->registerKeyboardCallback(keyboard_cb);
  UpdateMap();
  UpdateViewer();
  PrintIntructions();
  while (!viewer_->wasStopped()) {
    viewer_->spinOnce(10);
    if (quit_) {
      BEAM_INFO("CameraToMapAligner complete.");
      break;
    }
  }
}

void CameraToMapAligner::keyboardEventOccurred(
    const pcl::visualization::KeyboardEvent& event) {
  bool update_trans = true;
  Eigen::Vector3d trans{0, 0, 0};
  Eigen::Vector3d rot{0, 0, 0};
  if (event.getKeySym() == "a" && event.keyDown()) {
    trans[0] = sensitivity_t_ / 1000;
  } else if (event.getKeySym() == "s" && event.keyDown()) {
    trans[1] = sensitivity_t_ / 1000;
  } else if (event.getKeySym() == "d" && event.keyDown()) {
    trans[2] = sensitivity_t_ / 1000;
  } else if (event.getKeySym() == "z" && event.keyDown()) {
    trans[0] = -sensitivity_t_ / 1000;
  } else if (event.getKeySym() == "x" && event.keyDown()) {
    trans[1] = -sensitivity_t_ / 1000;
  } else if (event.getKeySym() == "v" && event.keyDown()) {
    trans[2] = -sensitivity_t_ / 1000;
  } else if (event.getKeySym() == "k" && event.keyDown()) {
    rot[0] = sensitivity_r_ * M_PI / 180;
  } else if (event.getKeySym() == "l" && event.keyDown()) {
    rot[1] = sensitivity_r_ * M_PI / 180;
  } else if (event.getKeySym() == "semicolon" && event.keyDown()) {
    rot[2] = sensitivity_r_ * M_PI / 180;
  } else if (event.getKeySym() == "m" && event.keyDown()) {
    rot[0] = -sensitivity_r_ * M_PI / 180;
  } else if (event.getKeySym() == "comma" && event.keyDown()) {
    rot[1] = -sensitivity_r_ * M_PI / 180;
  } else if (event.getKeySym() == "period" && event.keyDown()) {
    rot[2] = -sensitivity_r_ * M_PI / 180;
  } else if (event.getKeySym() == "Up" && event.keyDown()) {
    sensitivity_t_ += 0.1;
    std::cout << "increasing translational sensitivity\n";
    update_trans = false;
  } else if (event.getKeySym() == "Down" && event.keyDown()) {
    sensitivity_t_ -= 0.1;
    std::cout << "decreasing translational sensitivity\n";
    update_trans = false;
  } else if (event.getKeySym() == "Right" && event.keyDown()) {
    sensitivity_r_ += 0.1;
    std::cout << "increasing rotation sensitivity\n";
    update_trans = false;
  } else if (event.getKeySym() == "Left" && event.keyDown()) {
    sensitivity_r_ -= 0.1;
    std::cout << "decreasing rotation sensitivity\n";
    update_trans = false;
  } else if (event.getKeySym() == "r" && event.keyDown()) {
    T_reference_camera_ = T_reference_camera_original_;
    update_trans = false;
    UpdateMap();
    UpdateViewer();
  } else if (event.getKeySym() == "End" && event.keyDown()) {
    OutputUpdatedTransform();
    update_trans = false;
    quit_ = true;
  } else {
    update_trans = false;
  }

  if (update_trans) {
    UpdateExtrinsics(trans, rot);
    UpdateMap();
    UpdateViewer();
  }
  PrintIntructions();
}

void CameraToMapAligner::UpdateExtrinsics(const Eigen::Vector3d& trans,
                                          const Eigen::Vector3d& rot) {
  Eigen::Matrix4d T_pert = Eigen::Matrix4d::Identity();
  T_pert.block(0, 0, 3, 3) = beam::LieAlgebraToR(rot);
  T_pert.block(0, 3, 3, 1) = trans;
  T_reference_camera_ = T_reference_camera_ * T_pert;
}

void CameraToMapAligner::PrintIntructions() {
  std::cout << "Current sensitivity [trans - mm, rot - deg]: ["
            << sensitivity_t_ << ", " << sensitivity_r_ << "]\n"
            << "Press up/down arrow keys to increase/decrease translation "
               "sensitivity\n"
            << "Press right/left arrow keys to increase/decrease rotation "
               "sensitivity\n"
            << "Press buttons 'a/s/d' to increase translational DOF in x/y/z\n"
            << "Press buttons 'z/x/v' to decrease translational DOF in x/y/z\n"
            << "Press buttons 'k/l/;' to increase rotational DOF about x/y/z\n"
            << "Press buttons 'm/,/.' to decrease rotational DOF about x/y/z\n"
            << "Press 'r' to reset extrinsics\n"
            << "Press 'End' button to save final transform\n";
}

void CameraToMapAligner::UpdateMap() {
  BEAM_INFO("Updating map");
  Eigen::Matrix4d T_map_camera = T_map_reference_ * T_reference_camera_;
  Eigen::Matrix4d T_camera_map = beam::InvertTransform(T_map_camera);
  BEAM_INFO("Creating colorizer");
  std::unique_ptr<beam_colorize::Colorizer> colorizer =
      beam_colorize::Colorizer::Create(
          beam_colorize::ColorizerType::PROJECTION);
  colorizer->SetIntrinsics(camera_model_);

  if (image_container_.IsBGRImageSet()) {
    BEAM_INFO("Setting distortion");
    colorizer->SetDistortion(image_container_.GetBGRIsDistorted());
    BEAM_INFO("Setting BGR image in colorizer");
    colorizer->SetImage(image_container_.GetBGRImage());
    BEAM_INFO("Done setting image");
  } else {
    BEAM_ERROR("image container does not have a BGR image, cannot run app.");
    throw std::runtime_error("invalid image container");
  }

  PointCloud::Ptr map_in_camera_frame = std::make_shared<PointCloud>();
  pcl::transformPointCloud(*map_, *map_in_camera_frame,
                           Eigen::Affine3d(T_camera_map));
  colorizer->SetPointCloud(map_in_camera_frame);
  PointCloudCol::Ptr map_colored_in_camera_frame =
      colorizer->ColorizePointCloud();
  pcl::transformPointCloud(*map_colored_in_camera_frame, *map_colored_,
                           Eigen::Affine3d(T_map_camera));
  BEAM_INFO("Done updating map");
}

void CameraToMapAligner::UpdateViewer() {
  BEAM_INFO("Updating viewer");
  // add point cloud
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      map_colored_);
  viewer_->removePointCloud("Map");
  viewer_->addPointCloud<PointTypeCol>(map_colored_, rgb, "Map");
  viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, "Map");
  viewer_->setBackgroundColor(backgound_rgb_[0], backgound_rgb_[1],
                              backgound_rgb_[2]);

  // Update coordinate frame
  Eigen::Matrix4f T_map_camera =
      (T_map_reference_ * T_reference_camera_).cast<float>();
  viewer_->removeCoordinateSystem("CameraFrameUpdated");
  viewer_->addCoordinateSystem(coordinateFrameScale_,
                               Eigen::Affine3f(T_map_camera),
                               "CameraFrameUpdated");
  BEAM_INFO("Done updating viewer");
}

void CameraToMapAligner::OutputUpdatedTransform() {
  BEAM_INFO("Saving final transform to {}", inputs_.output);
  const Eigen::Matrix4d& T = T_reference_camera_;
  Eigen::Matrix4d TINV = beam::InvertTransform(T);

  nlohmann::json J = {
      {"to_frame", inputs_.reference_frame},
      {"from_frame", image_container_.GetBGRFrameId()},
      {"transform",
       {T(0, 0), T(0, 1), T(0, 2), T(0, 3), T(1, 0), T(1, 1), T(1, 2), T(1, 3),
        T(2, 0), T(2, 1), T(2, 2), T(2, 3), 0, 0, 0, 1}},
      {"transform_inverse",
       {TINV(0, 0), TINV(0, 1), TINV(0, 2), TINV(0, 3), TINV(1, 0), TINV(1, 1),
        TINV(1, 2), TINV(1, 3), TINV(2, 0), TINV(2, 1), TINV(2, 2), TINV(2, 3),
        0, 0, 0, 1}}};

  std::ofstream file(inputs_.output);
  file << std::setw(4) << J << std::endl;
}

} // end namespace inspection
