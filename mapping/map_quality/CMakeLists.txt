cmake_minimum_required(VERSION 3.14)
project(scan_pose_gt_generation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(
  beam REQUIRED
  COMPONENTS
  utils
)

## Declare a C++ library
add_library(
  ${PROJECT_NAME}
  src/MapQuality.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
)
## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
    beam::utils
)

################ Executables ###################
add_executable(${PROJECT_NAME}_run_map_quality_analysis
  src/run_map_quality_analysis.cpp
)

target_include_directories(${PROJECT_NAME}_run_map_quality_analysis
  PUBLIC
)

target_link_libraries(${PROJECT_NAME}_run_map_quality_analysis
  ${PROJECT_NAME}
  beam::utils
)