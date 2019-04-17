#pragma once

#include <string>

namespace inspection {

/**
 * @brief class for loading hand labels images/masks into beam image
 * containers
 */
class LoadHandLabels {
public:
  LoadHandLabels(const std::string config_file_location);

  ~LoadHandLabels() = default;

private:
};

} // namespace inspection
