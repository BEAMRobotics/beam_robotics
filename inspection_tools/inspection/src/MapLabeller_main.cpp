#include <thread>

#include "inspection/MapLabeler.h"

using namespace std::literals::chrono_literals;

std::string getJSONFileName(std::string file_name) {
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 24, file_location.end());
  file_location += "config/";
  file_location += file_name;
  return file_location;
}

int main(int argc, char* argv[]) {
  std::string map_labeler_config;
  map_labeler_config = getJSONFileName("MapLabeler.json");

  BEAM_INFO("Loading config from: {}", map_labeler_config);

  inspection::MapLabeler mapper_{map_labeler_config};

  mapper_.PlotFrames("hvlp_link", mapper_.viewer);
  mapper_.PlotFrames("F1_link", mapper_.viewer);
  mapper_.PlotFrames("F2_link", mapper_.viewer);
  mapper_.DrawFinalMap();
  mapper_.SaveLabeledClouds();

  while (!mapper_.viewer->wasStopped()) {
    mapper_.viewer->spinOnce(100);
    std::this_thread::sleep_for(100ms);
  }
  return 0;
}