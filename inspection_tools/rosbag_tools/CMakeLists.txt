cmake_minimum_required(VERSION 3.14)
project(rosbag_tools)

set(CMAKE_CXX_STANDARD 17)

find_package(
  beam REQUIRED
  COMPONENTS
  utils
  cv
  calibration
)

find_package(velodyne_pointcloud REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rosbag
    std_msgs
    sensor_msgs
    velodyne_pointcloud
    cv_bridge
)

# Unpack library
add_library(${PROJECT_NAME}
	src/lib/VelodyneTools.cpp
  src/lib/RosBagIO.cpp
  src/lib/PointcloudXYZIRT.cpp
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

add_executable(unpack_velodyne_scans
  src/UnpackVelodyneScans.cpp
)
target_link_libraries(unpack_velodyne_scans
  ${PROJECT_NAME}
	${catkin_LIBRARIES}
)

add_executable(process_bag
  src/ProcessBag.cpp
)
target_link_libraries(process_bag
  ${PROJECT_NAME}
  beam::cv
  beam::calibration
	${catkin_LIBRARIES}
)

add_executable(pose_stamped_to_odometry
  src/PoseStampedToOdometry.cpp
)
target_link_libraries(pose_stamped_to_odometry
  ${PROJECT_NAME}
	${catkin_LIBRARIES}
)
