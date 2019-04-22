#pragma once

#include <beam_calibration/Intrinsics.h>
#include <beam_calibration/TfTree.h>
#include <beam_colorize/Projection.h>
#include <beam_utils/math.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <string>
#include <nlohmann/json.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudColor;
typedef pcl::visualization::PCLVisualizer Viz;
typedef pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
        ColourHandler;

class LidarCamCalib {
public:
  LidarCamCalib();
  ~LidarCamCalib() = default;

  std::string GetFilePath(std::string filename);
  void PrintIntructions();
  void OutputUpdatedTransform();
  void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                             void *viewer_void);
  void run();

  double sensitivity_r = 3, sensitivity_t = 5;
  bool update_trans, image_distorted, PI = 3.14159265359;
  std::string calibration_to_frame, calibration_from_frame, image_file,
      intrinsics, extrinsics, map_file, map_frame, image_frame, filename,
      output_file_name;
  beam::Vec3 trans, rot;
  Eigen::Affine3d T_C_Map, T_C_Map_updated, T_C_to, T_to_from, T_to_from_updated,
      T_from_Map;
  PointCloud::Ptr cloud_transformed;
  PointCloudColor::Ptr cloud_colored;
  PointCloud::Ptr cloud;
  Viz::Ptr viewer;
  beam_colorize::Projection projector;

};
