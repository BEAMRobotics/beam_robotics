cmake_minimum_required(VERSION 3.2)
project(inspection)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(beam REQUIRED utils containers)
find_package(Catch2 REQUIRED)
find_package(nlohmann_json  3.2.0 REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
  LIBRARIES inspection
)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(src/lib)

add_executable(AutoInspection src/AutoInspection.cpp)
target_link_libraries(AutoInspection ${catkin_LIBRARIES} ${PCL_LIBRARIES}
                      inspection beam::utils)

add_executable(ManualInspectionA src/ManualInspectionA.cpp)
target_link_libraries(ManualInspectionA  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
                       inspection beam::utils)

add_executable(ManualInspectionB src/ManualInspectionB.cpp)
target_link_libraries(ManualInspectionB  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
                       inspection beam::utils)
