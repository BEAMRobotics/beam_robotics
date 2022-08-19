#include <inspection/CameraToMapAligner.h>

#include <beam_mapping/Poses.h>

namespace inspection {

CameraToMapAligner::CameraToMapAligner(const Inputs& inputs) : inputs_(inputs) {
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(inputs_.map, *map_) == -1) {
    BEAM_ERROR("Couldn't read file map pcd file: {}", inputs_.map);
  }
  LoadImageContainer();
  FillTfTrees();
  SetupColorizer();
  AddFixedCoordinateSystems();
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
    poses_tree_.AddTransform(Eigen::Affine3d(poses[i]), poses_fixed_frame_,
                             poses_moving_frame_, timestamps[i]);
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
}

void CameraToMapAligner::SetupColorizer() {
  std::shared_ptr<beam_calibration::CameraModel> camera_model =
      beam_calibration::CameraModel::Create(inputs_.intrinsics);

  BEAM_INFO("Creating colorizer");
  colorizer_ = beam_colorize::Colorizer::Create(
      beam_colorize::ColorizerType::PROJECTION);
  colorizer_->SetIntrinsics(camera_model);

  if (image_container_.IsBGRImageSet()) {
    BEAM_INFO("Setting BGR image in colorizer");
    colorizer_->SetDistortion(image_container_.GetBGRIsDistorted());
    colorizer_->SetImage(image_container_.GetBGRImage());
  } else {
    BEAM_ERROR("image container does not have a BGR image, cannot run app.");
    throw std::runtime_error("invalid image container");
  }

  colorizer_->SetDistortion(image_container_.GetBGRIsDistorted());
  colorizer_->SetImage(image_container_.GetBGRImage());
}

void CameraToMapAligner::AddFixedCoordinateSystems() {
  // add map coordinate frame
  BEAM_INFO("Adding fixed coordinate systems to viewer");
  viewer_->addCoordinateSystem(coordinateFrameScale_,
                               poses_fixed_frame_ + " (Map)");

  // add reference coordinate frame
  viewer_->addCoordinateSystem(coordinateFrameScale_,
                               Eigen::Affine3f(T_map_reference_.cast<float>()),
                               inputs_.reference_frame + " (Ref)");

  // add original camera coordinate frame
  Eigen::Matrix4f T_map_camera =
      (T_map_reference_ * T_reference_camera_).cast<float>();
  viewer_->addCoordinateSystem(coordinateFrameScale_,
                               Eigen::Affine3f(T_map_camera),
                               inputs_.reference_frame + "CameraFrameOrig");
}

void CameraToMapAligner::Run() {
  std::function<void(const pcl::visualization::KeyboardEvent&)> keyboard_cb =
      [this](const pcl::visualization::KeyboardEvent& event) {
        keyboardEventOccurred(event);
      };
  viewer_->registerKeyboardCallback(keyboard_cb);
  UpdateExtrinsics(Eigen::Vector3d(), Eigen::Vector3d());
  UpdateMap();
  UpdateViewer();
  PrintIntructions();
  while (!viewer_->wasStopped()) { viewer_->spinOnce(10); }
}

void CameraToMapAligner::keyboardEventOccurred(
    const pcl::visualization::KeyboardEvent& event) {
  bool update_trans = true;
  Eigen::Vector3d trans;
  Eigen::Vector3d rot;
  // pcl::visualization::PCLVisualizer::Ptr viewer =
  //     *static_cast<pcl::visualization::PCLVisualizer::Ptr*>(viewer_void);
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
    PrintIntructions();
    update_trans = false;
  } else if (event.getKeySym() == "Down" && event.keyDown()) {
    sensitivity_t_ -= 0.1;
    std::cout << "decreasing translational sensitivity\n";
    PrintIntructions();
    update_trans = false;
  } else if (event.getKeySym() == "Right" && event.keyDown()) {
    sensitivity_r_ += 0.1;
    std::cout << "increasing rotation sensitivity\n";
    PrintIntructions();
    update_trans = false;
  } else if (event.getKeySym() == "Left" && event.keyDown()) {
    sensitivity_r_ -= 0.1;
    std::cout << "decreasing rotation sensitivity\n";
    PrintIntructions();
    update_trans = false;
  } else if (event.getKeySym() == "End" && event.keyDown()) {
    OutputUpdatedTransform();
    update_trans = false;
    BEAM_INFO("You can now exit using 'ctrl + c'");
  }

  if (update_trans) {
    UpdateExtrinsics(trans, rot);
    UpdateMap();
    UpdateViewer();
  }
}

void CameraToMapAligner::UpdateExtrinsics(const Eigen::Vector3d& trans,
                                          const Eigen::Vector3d& rot) {
  Eigen::Matrix4d T_pert = Eigen::Matrix4d::Identity();
  T_pert.block(0, 0, 3, 3) = beam::LieAlgebraToR(rot);
  T_pert.block(0, 3, 3, 1) = trans;
  T_reference_camera_ = T_reference_camera_ * T_pert;
}

void CameraToMapAligner::PrintIntructions() {
  std::cout
      << "Current sensitivity [trans - mm, rot - deg]: [" << sensitivity_t_
      << ", " << sensitivity_r_ << "]"
      << "\n"
      << "Press up/down arrow keys to increase/decrease translation sensitivity"
      << "\n"
      << "Press right/left arrow keys to increase/decrease rotation sensitivity"
      << "\n"
      << "Press buttons 'a/s/d' to increase translational DOF in x/y/z"
      << "\n"
      << "Press buttons 'z/x/v' to decrease translational DOF in x/y/z"
      << "\n"
      << "Press buttons 'k/l/;' to increase rotational DOF about x/y/z"
      << "\n"
      << "Press buttons 'm/,/.' to decrease rotational DOF about x/y/z"
      << "\n"
      << "Press 'End' button to save final transform."
      << "\n";
}

void CameraToMapAligner::UpdateMap() {
  colorizer_->SetPointCloud(map_);
  Eigen::Matrix4d T_map_camera = T_map_reference_ * T_reference_camera_;
  Eigen::Affine3d TA_camera_map(beam::InvertTransform(T_map_camera));
  colorizer_->SetTransform(TA_camera_map);
  map_colored_ = colorizer_->ColorizePointCloud();
}

void CameraToMapAligner::UpdateViewer() {
  // add point cloud
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      map_colored_);
  viewer_->removeAllPointClouds();
  viewer_->addPointCloud<PointTypeCol>(map_colored_, rgb, "Map");
  viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Map");
  viewer_->setBackgroundColor(1, 1, 1);

  // Update coordinate frame
  Eigen::Matrix4f T_map_camera =
      (T_map_reference_ * T_reference_camera_).cast<float>();
  viewer_->removeCoordinateSystem("CameraFrameUpdated");
  viewer_->addCoordinateSystem(coordinateFrameScale_,
                               Eigen::Affine3f(T_map_camera),
                               "CameraFrameUpdated");
  viewer_->initCameraParameters();
}

void CameraToMapAligner::OutputUpdatedTransform() {
  BEAM_INFO("Saving final transform to {}", inputs_.output);
  const auto& T = T_reference_camera_;
  double T1 = T(0, 0), T2 = T(0, 1), T3 = T(0, 2), T4 = T(0, 3), T5 = T(1, 0),
         T6 = T(1, 1), T7 = T(1, 2), T8 = T(1, 3), T9 = T(2, 0), T10 = T(2, 1),
         T11 = T(2, 2), T12 = T(2, 3), T13 = T(3, 0), T14 = T(3, 1),
         T15 = T(3, 2), T16 = T(3, 3);

  nlohmann::json J = {{"to_frame", inputs_.reference_frame},
                      {"from_frame", image_container_.GetBGRFrameId()},
                      {"transform",
                       {T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12, T13,
                        T14, T15, T16}}};
  std::ofstream file(inputs_.output);
  file << std::setw(4) << J << std::endl;
}

} // end namespace inspection
