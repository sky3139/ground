cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(bare_earth)
# SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_BUILD_TYPE "Debug")

find_package(PCL  REQUIRED)
find_package(Boost REQUIRED)
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_definitions(${PCL_DEFINITIONS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}
${BOOST_LIBRARY_DIRS}
)
# //SHARED
file(GLOB SRC src/GASS/*.cpp)

add_library(amp   src/ground_detection.cpp src/apmf.h 
src/pcd_io.cpp
src/GPF/GPF.cpp
src/FSG/test_floorseg.cpp src/FSG/CloudBin.cpp src/FSG/CloudSegment.cpp src/FSG/FloorSegment.cpp
${SRC}
)
target_link_libraries (amp ${PCL_LIBRARIES} -lpthread )

add_executable(main src/main.cpp)
target_link_libraries (main amp  )

add_executable(evo src/evo.cpp)
target_link_libraries (evo  ${PCL_LIBRARIES}  )

