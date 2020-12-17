cmake_minimum_required(VERSION 3.2)
project(relocalization)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

find_package(rosbag REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(DBoW3 REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(beam REQUIRED calibration containers utils cv mapping)

SET(Torch_DIR /home/nick/software/pytorch/torch/lib/tmp_install/share/cmake/Torch )
FIND_PACKAGE(Torch REQUIRED)

add_library(${PROJECT_NAME}
  src/lib/ImageDatabase.cpp
  src/lib/ImageToImage.cpp
  src/lib/ImageSimilaritySearch.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}
  beam::cv
  beam::calibration
  beam::containers
  beam::utils
  beam::mapping
  Boost::boost
  PCL::PCL
  OpenCV::OpenCV
  DBoW3
  nlohmann_json::nlohmann_json
  ${rosbag_LIBRARIES}
  ${TORCH_LIBRARIES}
)


#add_executable(${PROJECT_NAME}_debug
#	src/debug.cpp
#)
#target_link_libraries(${PROJECT_NAME}_debug
#  ${PROJECT_NAME}
#)

add_executable(${PROJECT_NAME}_image_similarity_search
	src/find_similar_images.cpp
)
target_link_libraries(${PROJECT_NAME}_image_similarity_search
  ${PROJECT_NAME}
  gflags
	beam::utils
)

add_executable(${PROJECT_NAME}_build_db
	src/BuildImageDB.cpp
)
target_link_libraries(${PROJECT_NAME}_build_db
  ${PROJECT_NAME}
)

#add_executable(${PROJECT_NAME}_cmu_eval
#	src/CMUDatasetEval.cpp
#)
#target_link_libraries(${PROJECT_NAME}_cmu_eval
#  ${PROJECT_NAME}
#)