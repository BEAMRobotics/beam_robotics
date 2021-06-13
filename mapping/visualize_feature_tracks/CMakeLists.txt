cmake_minimum_required(VERSION 2.8.3)
project(visualize_feature_tracks)

## c++11 feature
set(CMAKE_CXX_FLAGS "-std=gnu++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

## Need to dynamically link Boost's log library for loggers shared across multiple targets/libraries
add_definitions(-DBOOST_LOG_DYN_LINK)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(catkin_deps roscpp roslib nodelet)
find_package(catkin REQUIRED ${catkin_deps} message_generation)
find_package(nodelet REQUIRED)
find_package(roscpp REQUIRED)
find_package(beam REQUIRED utils cv containers)
find_package(cv_bridge REQUIRED)

add_message_files(
  DIRECTORY
    msg
  FILES
    LandmarkMeasurementMsg.msg
    CameraMeasurementMsg.msg    
)

generate_messages()

## Find Boost log components used for autonomoose diagnostics
find_package(Boost COMPONENTS REQUIRED log_setup log)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${catkin_deps}
    DEPENDS Boost
    LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR} # NOTE: No 'S' for Boost's include directory
    ${nodelet_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
)

## Declare a C++ library for underlying (non-ROS) implementation (in this case: example_class)
#### Link your ROS nodelet against this library
add_library(${PROJECT_NAME}
    src/feature_tracker.cpp
)

## Link non-ROS C++ library target against required libraries
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    beam::utils
    beam::cv
    beam::containers
)

## Declare a C++ library for the nodelet
add_library(${PROJECT_NAME}_tracker_nodelet
    src/tracker_nodelet.cpp
)

## Set links for the nodelet library
#### Link the nodelet library target against the non-ROS implementation library.
#### Also link against Boost so that the log catcher in the nodelet can be used.
target_link_libraries(${PROJECT_NAME}_tracker_nodelet
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${nodelet_LIBRARIES}
    ${roscpp_LIBRARIES}
)

## Declare a C++ executable
#### ROS Node
add_executable(${PROJECT_NAME}_tracker_node
    src/tracker_node.cpp
)

## Set links for the node executable
#### Link the ROS node executable target against:
########    catkin libraries
########    nodelet library
target_link_libraries(${PROJECT_NAME}_tracker_node
    ${catkin_LIBRARIES}
    ${nodelet_LIBRARIES}
    ${roscpp_LIBRARIES}
)

