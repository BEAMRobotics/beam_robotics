#pragma once

#include <beam_utils/pointclouds.h>

namespace map_quality {

class MapQuality {
public:
  MapQuality(const std::string& map_path, const std::string& output_path);

private:
  void LoadCloud();

  void SaveResults();

  void CalculateVoxelCount();

  std::string map_path_;
  std::string output_path_;
  PointCloud map_;

  // params:
  double voxel_size_m_{0.01};
};

} // namespace map_quality