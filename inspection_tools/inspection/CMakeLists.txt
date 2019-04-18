cmake_minimum_required(VERSION 3.2)
project(inspection)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(beam REQUIRED utils containers calibration defects)

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
	beam::utils
)

# ManualB inspection binaries
add_executable(${PROJECT_NAME}_manual_B
	src/ManualInspectionB.cpp
)
target_link_libraries(${PROJECT_NAME}_manual_B
	${PROJECT_NAME}
	beam::utils
)
