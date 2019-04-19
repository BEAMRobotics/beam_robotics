#include "inspection/ImageExtractor.h"

std::string getJSONFileName(std::string file_name){
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 25, file_location.end());
  file_location += "config/";
  file_location += file_name;
  return file_location;
}

int main() {

  std::string img_extractor_config;
  img_extractor_config = getJSONFileName("ImageExtractorConfig.json");

  inspection::ImageExtractor ImgExtractor(img_extractor_config);
  ImgExtractor.ExtractImages();

  return 0;
 }
