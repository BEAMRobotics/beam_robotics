#include "inspection/MapLabeler.h"
#include <thread>

std::string getJSONFileName(std::string file_name) {
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 32, file_location.end());
  file_location += "config/";
  file_location += file_name;
  return file_location;
}

int main(int argc, char* argv[]) {
  std::string map_labeler_config;
  std::cout << map_labeler_config << std::endl;
  map_labeler_config = getJSONFileName("MapLabeler.json");
  std::cout << map_labeler_config << std::endl;

  inspection::MapLabeler mapper_{map_labeler_config};
  return 0;
}