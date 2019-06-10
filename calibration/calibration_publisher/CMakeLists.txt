cmake_minimum_required(VERSION 3.14)
project(calibration_publisher)

SET(CMAKE_CXX_STANDARD 17)

find_package(beam REQUIRED utils calibration)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  tf
)

find_package(nlohmann_json 3.2.0 REQUIRED)

# Add the core library
#add_subdirectory(calibration_util)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  std_msgs
  tf
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_main
  src/calibration_publisher_node.cpp
)

target_link_libraries(${PROJECT_NAME}_main
  ${catkin_LIBRARIES}
  beam::utils
  beam::calibration
)
