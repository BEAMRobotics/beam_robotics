#include "inspection/MapLabeler.h"
#include <thread>

int main(int argc, char* argv[]) {
  std::string json_path = "/home/steve/inspection_labeler/MapLabeler.json";
  inspection::MapLabeler mapper_{json_path};
  return 0;
}