cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(genSurfel)
set(CMAKE_BUILD_TYPE Debug)

FIND_PACKAGE(PCL 1.5 REQUIRED)
add_compile_options(-std=c++11)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# add_executable(genSurfel genSurfel.cpp ioVis.cpp)
# target_link_libraries (genSurfel ${PCL_LIBRARIES})

# add_executable (icp icp.cpp)
# target_link_libraries (icp ${PCL_LIBRARIES})

# add_executable (kd kd.cpp)
# target_link_libraries (kd ${PCL_LIBRARIES})

add_executable (div divideCloud.cpp ioVis.cpp)
target_link_libraries (div ${PCL_LIBRARIES})