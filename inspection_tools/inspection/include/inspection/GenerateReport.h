#pragma once

#include <string>

namespace inspection {

/**
 * @brief class for generating an inspection report
 */
class GenerateReport {
public:
  GenerateReport(const std::string config_file_location);

  ~GenerateReport() = default;

private:
};

} // namespace inspection
