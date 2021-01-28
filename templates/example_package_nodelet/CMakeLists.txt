cmake_minimum_required(VERSION 2.8.3)
project(example_package)

## c++11 feature
set(CMAKE_CXX_FLAGS "-std=gnu++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

## Need to dynamically link Boost's log library for loggers shared across multiple targets/libraries
add_definitions(-DBOOST_LOG_DYN_LINK)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(catkin_deps roscpp roslib nodelet)
find_package(catkin REQUIRED ${catkin_deps})
find_package(nodelet REQUIRED)
find_package(roscpp REQUIRED)
find_package(beam REQUIRED utils)

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
    src/example_class/example_class.cpp
)

## Link non-ROS C++ library target against required libraries
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    beam::utils
)

## Declare a C++ library for the nodelet
add_library(${PROJECT_NAME}_nodelet
    src/${PROJECT_NAME}_nodelet.cpp
)

## Set links for the nodelet library
#### Link the nodelet library target against the non-ROS implementation library.
#### Also link against Boost so that the log catcher in the nodelet can be used.
target_link_libraries(${PROJECT_NAME}_nodelet
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${nodelet_LIBRARIES}
    ${roscpp_LIBRARIES}
)

## Declare a C++ executable
#### ROS Node
add_executable(${PROJECT_NAME}_node
    src/example_package_node.cpp
)

## Set links for the node executable
#### Link the ROS node executable target against:
########    catkin libraries
########    nodelet library
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${nodelet_LIBRARIES}
    ${roscpp_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_nodelet ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch scripts config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Need to explicitly list nodelet.xml for installation
install(FILES
    nodelets.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest unit testing
if (CATKIN_ENABLE_TESTING)
    #### Explicitly add test .cpp files with the test executable target called ${PROJECT_NAME}-test
    catkin_add_gtest(${PROJECT_NAME}-test
        test/test_example_package.cpp
        test/ExampleClass_test.cpp
        test/NullTest.cpp
    )
    #### Link test executable against non-ROS implementation library
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${PROJECT_NAME} ${Boost_LIBRARIES})
endif(CATKIN_ENABLE_TESTING)
