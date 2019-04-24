#include "inspection/MapLabeler.h"
#include <thread>

using namespace std::literals::chrono_literals;

std::string getJSONFileName(std::string file_name) {
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 24, file_location.end());
  file_location += "config/";
  file_location += file_name;
  return file_location;
}

int main(int argc, char* argv[]) {
  //std::string map_labeler_config;
  //std::cout << map_labeler_config << std::endl;
  //map_labeler_config = getJSONFileName("MapLabeler.json");
  //std::cout << "Loading MapLabeler.json from: " << map_labeler_config
   //         << std::endl;

  inspection::MapLabeler mapper_{"Test"};


  return 0;
}
