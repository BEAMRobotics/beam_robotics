cmake_minimum_required(VERSION 3.2)
PROJECT(manual_cam_lidar_calib)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(beam REQUIRED utils colorize calibration)
find_package(nlohmann_json 3.2.0 REQUIRED)


############# Calibrater Main #############
add_executable(${PROJECT_NAME}_main
  src/main.cpp
  src/LC_calibrater.cpp
)
target_include_directories(${PROJECT_NAME}_main
  PUBLIC
    include/manual_cam_lidar_calib
)
target_link_libraries(${PROJECT_NAME}_main
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  beam::utils
  beam::colorize
  beam::calibration
  PCL::PCL
)
##########################################
