cmake_minimum_required(VERSION 3.14)
project(map_quality)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(
  beam REQUIRED
  COMPONENTS
  utils
)
find_package(nlohmann_json 3.2.0 REQUIRED)

find_package(OpenMP)

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
    OpenMP::OpenMP_CXX
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

add_executable(${PROJECT_NAME}_tests
  src/tests.cpp
)

target_include_directories(${PROJECT_NAME}_tests
  PUBLIC
)

target_link_libraries(${PROJECT_NAME}_tests
  ${PROJECT_NAME}
  beam::utils
)