cmake_minimum_required(VERSION 3.2)
project(inspection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

find_package(catkin REQUIRED COMPONENTS
  roscpp
	cv_bridge
  geometry_msgs
  nav_msgs
	pcl_ros
	pcl_conversions
	sensor_msgs
  std_msgs
  tf
)

find_package(rosbag REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(beam REQUIRED calibration colorize containers defects utils)

find_package(pcl_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

# Add the core inspection library
add_subdirectory(src/lib)

# Auto inspection binary
add_executable(${PROJECT_NAME}_auto
	src/AutoInspection.cpp
)
target_link_libraries(${PROJECT_NAME}_auto
	${PROJECT_NAME}
	beam::utils
	beam::calibration
)

# ManualA inspection binaries
add_executable(${PROJECT_NAME}_manual_A
	src/ManualInspectionA.cpp
)
target_link_libraries(${PROJECT_NAME}_manual_A
	${PROJECT_NAME}
)

# ManualB inspection binaries
add_executable(${PROJECT_NAME}_manual_B
	src/ManualInspectionB.cpp
)
target_link_libraries(${PROJECT_NAME}_manual_B
	${PROJECT_NAME}
)

# Map Labeller
add_executable(${PROJECT_NAME}_map_labeller
	src/MapLabeller_main.cpp
)
target_link_libraries(${PROJECT_NAME}_map_labeller
	${PROJECT_NAME}
	${pcl_ros_LIBRARIES}
)
