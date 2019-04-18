#include "inspection/MapLabeler.h"

int main(int argc, char* argv[]){

  std::string pose_file = "/home/steve/pose_file_test.txt";
  std::string map_file = "/home/steve/map.pcd";
  std::string image_config = "/home/steve/images.json";
  std::string json_path = "/home/steve/MapLabeler.json";

  inspection::MapLabeler mapper_ = {json_path};

  return 0;
}