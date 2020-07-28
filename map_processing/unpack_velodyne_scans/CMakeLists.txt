cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(unpack_velodyne_scans)

find_package(beam REQUIRED utils)
find_package(Boost REQUIRED)
find_package(velodyne_pointcloud REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rosbag
    std_msgs
    sensor_msgs
    velodyne_pointcloud
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rosbag
    std_msgs
    sensor_msgs
  DEPENDS
		BOOST
)

## Build ##
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${velodyne_pointcloud_INCLUDE_DIR}
)

# Unpack library
add_library(${PROJECT_NAME}
  src/UnpackVelodyneScans.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
		${Boost_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}
  beam::beam
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
