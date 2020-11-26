#include <inspection/InspectionUtils.h>

namespace inspection { namespace utils {

std::string GetConfigFilePath(const std::string& file_name) {
  std::string file_location = __FILE__;
  std::string current_file_path_rel = "src/lib/InspectionUtils.cpp";
  file_location.erase(file_location.end() - current_file_path_rel.length(),
                      file_location.end());
  file_location += "config/";
  file_location += file_name;
  return file_location;
}

}} // namespace inspection::utils
