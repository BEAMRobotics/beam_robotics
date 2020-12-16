cmake_minimum_required(VERSION 3.2)
project(sfm)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

find_package(OpenCV REQUIRED)
find_package(beam REQUIRED utils cv)
find_package(gflags REQUIRED)

SET(Torch_DIR /home/nick/software/pytorch/torch/lib/tmp_install/share/cmake/Torch )
FIND_PACKAGE(Torch REQUIRED)

add_library(${PROJECT_NAME}
  src/ExtractImageFeatures.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}
  beam::cv
  beam::utils
  Boost::boost
  catkin::catkin
  OpenCV::OpenCV
)

add_executable(${PROJECT_NAME}_extract_image_features
	src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_extract_image_features
	${PROJECT_NAME}
	gflags
)