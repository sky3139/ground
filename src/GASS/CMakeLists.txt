cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(gaussian_ground_segmentation)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

add_executable(main main GaussianFloorSegmentation.cpp)
target_link_libraries(main ${PCL_LIBRARIES})