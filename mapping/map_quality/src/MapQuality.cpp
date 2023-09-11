#include <map_quality/ScanPoseGtGeneration.h>

#include <beam_utils/log.h>

namespace map_quality {

void MapQuality::MapQuality(const std::string& map_path)
    : map_path_(map_path), output_path_(output_path) {
  LoadCloud();
  CalculateVoxelCount();
  SaveResults();
  BEAM_INFO("Map quality analysis finished successfully!");
}

void MapQuality::LoadCloud() {
  pcl::io::loadPCDFile(map_path_, map_);
  if (map_.empty()) {
    BEAM_ERROR("empty input map.");
    throw std::runtime_error{"empty input map"};
  }
}

void MapQuality::SaveResults() {
  BEAM_INFO("Saving results to: {}", output_path_);

  // TODO
}

void CalculateVoxelCount() {
  //
}

} // namespace map_quality