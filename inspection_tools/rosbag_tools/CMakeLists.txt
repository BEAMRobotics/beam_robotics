cmake_minimum_required(VERSION 3.14)
project(rosbag_tools)

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

# executables
add_executable(unpack_velodyne_scans_main
  src/UnpackVelodyneScansMain.cpp
)
target_link_libraries(unpack_velodyne_scans_main
  ${PROJECT_NAME}
	${catkin_LIBRARIES}
)
