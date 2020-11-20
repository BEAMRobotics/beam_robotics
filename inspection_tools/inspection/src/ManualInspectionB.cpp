#include "inspection/QuantifyDefects.h"

std::string getJSONFileName(std::string file_name) {
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 25, file_location.end());
  file_location += "config/";
  file_location += file_name;
  return file_location;
}

int main() {
  std::string quantify_defects_config;
  quantify_defects_config = getJSONFileName("QuantifyDefectsConfig.json");

  inspection::QuantifyDefects quant_defects(quantify_defects_config);
  quant_defects.OutputCorrosionInfo();
  quant_defects.SaveCorrosionOnlyCloud();

  quant_defects.OutputCrackInfo();
  quant_defects.SaveCrackOnlyCloud();

  quant_defects.OutputSpallInfo();
  quant_defects.SaveSpallOnlyCloud();

  return 0;
}