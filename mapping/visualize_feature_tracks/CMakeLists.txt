cmake_minimum_required(VERSION 3.14)
project(visualize_feature_tracks)

set(CMAKE_CXX_STANDARD 14)


find_package(
  beam REQUIRED
  COMPONENTS
  utils
  cv
)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    roslib
    nodelet
    sensor_msgs
    velodyne_pointcloud
)
find_package(OpenCV 4.5.2 REQUIRED COMPONENTS)

# create library
add_library(${PROJECT_NAME}
    src/feature_tracker.cpp
    src/nodelet.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
    beam::utils
    beam::cv
    ${catkin_LIBRARIES}
)

# executables
add_executable(${PROJECT_NAME}_node
    src/node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    beam::cv
    beam::utils
    ${catkin_LIBRARIES}
)

