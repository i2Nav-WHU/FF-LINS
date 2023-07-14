cmake_minimum_required(VERSION 3.10)
project(factors)

include_directories(
    ${PROJECT_SOURCE_DIR}/../)
add_library(${PROJECT_NAME} SHARED
    marginalization_factor.cc
    marginalization_info.cc
    pose_manifold.cc
    residual_block_info.cc
    lidar_factor.cc)

# common
target_link_libraries(${PROJECT_NAME} common)

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Ceres
find_package(Ceres REQUIRED)
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})

# TBB
find_package(TBB REQUIRED)
target_link_libraries(${PROJECT_NAME}
    TBB::tbb)
