cmake_minimum_required(VERSION 3.10)
project(ikd-Tree)

add_library(${PROJECT_NAME} SHARED
    ikd_Tree.cc)

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
