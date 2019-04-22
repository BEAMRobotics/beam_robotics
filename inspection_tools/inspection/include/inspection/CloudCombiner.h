#pragma once

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <beam_containers/PointBridge.h>
#include <beam_utils/time.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <boost/make_shared.hpp>



namespace inspection {

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;
using BridgePoint = beam_containers::PointBridge;
using DefectCloud = pcl::PointCloud<beam_containers::PointBridge>;
using PointCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZRGB>;

class CloudCombiner {
public:
  CloudCombiner() = default;
  ~CloudCombiner() = default;

  void CombineClouds(std::vector<std::vector<DefectCloud::Ptr>> clouds);

  DefectCloud::Ptr GetCombinedCloud(){return combined_cloud_;}

private:
  std::vector<std::vector<DefectCloud::Ptr>> defect_clouds_ = {};
  DefectCloud::Ptr combined_cloud_ = boost::make_shared<DefectCloud>();
};

}