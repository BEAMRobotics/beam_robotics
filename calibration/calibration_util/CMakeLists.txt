cmake_minimum_required(VERSION 3.14)
project(calibration_util)

set(CMAKE_CXX_STANDARD 14)

## System dependencies are found with CMake's conventions
find_package(OpenCV 4.5.2 REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(beam REQUIRED utils)
find_package(PCL 1.11.1 REQUIRED common registration point_types io)
find_package(Ceres REQUIRED)
find_package(gflags REQUIRED)

find_package(
  catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    roslib
  )
    

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    geometry_msgs
    roscpp
    sensor_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    roslib
  DEPENDS
    Boost
    CERES
    EIGEN3
  )

###########
## Build ##
###########
include_directories(
  include  
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${gflag_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/calibration_util/calibration_util.cpp
  src/calibration_util/lidar_util.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
    ${PCL_LIBRARIES}
    beam::utils
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # test parking_spot_detector: libary-level unit testing
  catkin_add_gtest(test_calibration_util
    test/test_main.cpp
    test/test_calibration_util.cpp
    test/test_lidar_util.cpp
  )

  target_link_libraries(test_calibration_util
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    beam::utils
  )
endif (CATKIN_ENABLE_TESTING)
