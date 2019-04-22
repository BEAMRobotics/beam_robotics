#include "inspection/MapLabeler.h"
#include <thread>

using namespace std::literals::chrono_literals;

int main(int argc, char* argv[]){

  std::string json_path = "/home/steve/inspection_labeler/MapLabeler.json";
  {

  inspection::MapLabeler mapper_{json_path};
  std::cout << "Test1" << std::endl;

  mapper_.PlotFrames("hvlp_link", mapper_.viewer);
  mapper_.PlotFrames("F1_link", mapper_.viewer);
  mapper_.PlotFrames("F2_link", mapper_.viewer);
//  mapper_.DrawColoredClouds();
  mapper_.DrawFinalMap();
  mapper_.SaveLabeledClouds();

  while (!mapper_.viewer->wasStopped ())
  {
//    std::cout << "Test" << std::endl;
    mapper_.viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }

  }
  return 0;
}