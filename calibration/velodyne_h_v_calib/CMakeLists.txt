cmake_minimum_required(VERSION 2.8.3)
project(velodyne_h_v_calib)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED cmake_modules)
find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  roscpp
  std_msgs
  tf_conversions
  roslib
  message_generation
)
find_package(Eigen REQUIRED)

add_message_files(
  FILES
  VelodyneTF.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   INCLUDE_DIRS
   CATKIN_DEPENDS
  	message_runtime
  	std_msgs roscpp
  	tf_conversions
  	interactive_markers
  	roslib
  DEPENDS Eigen
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(velodyne_h_v_calib src/velodyne_h_v_calib.cpp)
add_dependencies(velodyne_h_v_calib velodyne_h_v_calib_generate_messages_cpp)
target_link_libraries(velodyne_h_v_calib ${catkin_LIBRARIES})

add_executable(velodyne_h_v_calib_2 src/velodyne_h_v_calib_2.cpp)
add_dependencies(velodyne_h_v_calib_2 velodyne_h_v_calib_generate_messages_cpp)
target_link_libraries(velodyne_h_v_calib_2 ${catkin_LIBRARIES})
