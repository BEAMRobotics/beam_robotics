cmake_minimum_required(VERSION 3.2)
project(inspection)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  sensor_msgs
  std_msgs
  tf
  cv_bridge
)

find_package(rosbag REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.11.1 REQUIRED)
find_package(beam REQUIRED calibration colorize containers defects utils depth)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(OpenCV 4.5.2 REQUIRED COMPONENTS)
find_package(PythonLibs REQUIRED)

# Add the core inspection library
add_subdirectory(src/lib)
include_directories(${PYTHON_INCLUDE_DIRS})

# extract images binary
add_executable(${PROJECT_NAME}_extract_images
	src/ExtractImages.cpp
)
target_link_libraries(${PROJECT_NAME}_extract_images
	${PROJECT_NAME}
	gflags
	beam::utils
)

# view and filter images binary
add_executable(${PROJECT_NAME}_view_and_filter_images
	src/ViewAndFilterImages.cpp
)
target_link_libraries(${PROJECT_NAME}_view_and_filter_images
	${PROJECT_NAME}
	gflags
	beam::utils
)

# Image Labeller
add_executable(${PROJECT_NAME}_label_images
        src/LabelImages.cpp
)
target_link_libraries(${PROJECT_NAME}_label_images
        ${PROJECT_NAME}
        gflags
        beam::utils
        ${PYTHON_LIBRARIES}
)

# ManualB inspection binaries
add_executable(${PROJECT_NAME}_quantify_defects
	src/QuantifyDefects.cpp
)
target_link_libraries(${PROJECT_NAME}_quantify_defects
	${PROJECT_NAME}
	gflags
	beam::utils
)

# Map Labeller
add_executable(${PROJECT_NAME}_label_map
	src/LabelMap.cpp
)
target_link_libraries(${PROJECT_NAME}_label_map
	${PROJECT_NAME}
	${pcl_ros_LIBRARIES}
	gflags
	beam::utils
)

# OdomTopicToPosesFile
add_executable(${PROJECT_NAME}_odom_topic_to_poses_file
	src/OdomTopicToPosesFile.cpp
)
target_link_libraries(${PROJECT_NAME}_odom_topic_to_poses_file
	${PROJECT_NAME}
	gflags
	beam::mapping
	beam::utils
)

# CalibrateCameraToMap
add_executable(${PROJECT_NAME}_calibrate_camera_to_map
	src/CalibrateCameraToMap.cpp
)
target_link_libraries(${PROJECT_NAME}_calibrate_camera_to_map
	${PROJECT_NAME}
	gflags
	beam::mapping
	beam::utils
	beam::colorize
	beam::calibration
)