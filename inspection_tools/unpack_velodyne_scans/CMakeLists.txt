cmake_minimum_required(VERSION 3.14)
project(unpack_velodyne_scans)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED)
find_package(gflags REQUIRED)
find_package(velodyne_pointcloud REQUIRED)
find_package(beam REQUIRED utils)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rosbag
    std_msgs
    sensor_msgs
    velodyne_pointcloud
)

# Unpack library
add_library(${PROJECT_NAME}
	src/UnpackVelodyneScans.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  beam::utils
  ${catkin_LIBRARIES}
)

# main executable
add_executable(${PROJECT_NAME}_main
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_main
  ${PROJECT_NAME}
	${catkin_LIBRARIES}
)