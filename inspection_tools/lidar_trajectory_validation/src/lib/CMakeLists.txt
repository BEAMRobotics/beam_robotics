# Core inspection library
add_library(${PROJECT_NAME}
  LidarTrajectoryValidation.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    ../../include
)
target_link_libraries(${PROJECT_NAME}
  beam::calibration 
  beam::utils
  beam::mapping
  beam::optimization
  beam::filtering
  beam::matching
  PCL::PCL
  nlohmann_json::nlohmann_json
)
