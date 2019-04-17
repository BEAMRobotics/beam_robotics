#pragma once

#include <string>

namespace inspection {

/**
 * @brief class for labeling/coloring a SLAM map given beam image containers
 */
class MapLabeler {
public:
  MapLabeler(const std::string config_file_location);

  ~MapLabeler() = default;

private:
};

} // namespace inspection
