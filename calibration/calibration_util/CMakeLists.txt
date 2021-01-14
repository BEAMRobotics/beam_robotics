cmake_minimum_required(VERSION 2.8.3)
project(calibration_util)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
set(catkin_deps
    rosbag
    pcl_ros
    message_filters)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  pcl_ros
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3)

find_package(PCL 1.8 REQUIRED common registration point_types io)
add_definitions(${PCL_DEFINITIONS})

if(NOT EIGEN3_FOUND)
  #Fallback to cmake modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIR ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif(NOT EIGEN3_FOUND)

add_definitions(${EIGEN_DEFINITIONS})
find_package(Ceres REQUIRED)
add_definitions(-DUSE_CERES)
find_package(gflags REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${catkin_deps}
  DEPENDS Eigen OpenCV
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${gflag_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
)

LINK_DIRECTORIES(
  ${PCL_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
  src/calibration_util/calibration_util.cpp
  src/calibration_util/lidar_util.cpp
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
  )
endif (CATKIN_ENABLE_TESTING)
