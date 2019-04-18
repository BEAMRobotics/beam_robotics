#include "inspection/QuantifyDefects.h"

namespace inspection {
QuantifyDefects::QuantifyDefects(const std::string config_file_location) {
  // load member variables
  nlohmann::json J;
  std::ifstream file(config_file_location);
  file >> J;

  cloud_filename_ = J["cloud_filename"];
  crack_threshold_ = J["crack_threshold"];
  spall_threshold_ = J["spall_threshold"];
  delam_threshold_ = J["delam_threshold"];
}

void QuantifyDefects::OutputCrackInfo() {

};

void QuantifyDefects::OutputSpallInfo() {

};

void QuantifyDefects::OutputDelamInfo() {

};

void QuantifyDefects::SaveCrackOnlyCloud() {

};

void QuantifyDefects::SaveSpallOnlyCloud() {

};

void QuantifyDefects::SaveDelamOnlyCloud() {

};

} // end namespace inspection
