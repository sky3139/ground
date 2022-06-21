cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(bare_earth)
SET(CMAKE_BUILD_TYPE "Release")
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
add_library(amp   src/ground_detection.cpp src/apmf.h)
target_link_libraries (amp ${PCL_LIBRARIES} -lpthread )

add_executable(main src/main.cpp)
target_link_libraries (main amp  )

add_executable(evo src/evo.cpp)
target_link_libraries (evo  ${PCL_LIBRARIES}  )

add_executable(pp src/pp.cpp)
target_link_libraries (pp  ${PCL_LIBRARIES} boost_thread )
add_executable(gp3 src/gp3.cpp)
target_link_libraries (gp3  ${PCL_LIBRARIES} boost_thread )
add_executable(mc src/mc.cpp)
target_link_libraries (mc  ${PCL_LIBRARIES} boost_thread )
add_executable(meply src/meply.cc)
target_link_libraries (meply  ${PCL_LIBRARIES} boost_thread )
add_executable(mls src/mls.cc)
target_link_libraries (mls  ${PCL_LIBRARIES} boost_thread )
add_executable(render src/render.cc)
target_link_libraries (render  ${PCL_LIBRARIES} boost_thread )