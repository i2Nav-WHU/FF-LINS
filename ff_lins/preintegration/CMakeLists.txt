cmake_minimum_required(VERSION 3.10)
project(preintegration)

include_directories(
    ${PROJECT_SOURCE_DIR}/../)

add_library(${PROJECT_NAME} SHARED
    preintegration_base.cc
    preintegration_normal.cc
    preintegration_odo.cc
    preintegration_earth.cc
    preintegration_earth_odo.cc)

# common
target_link_libraries(${PROJECT_NAME} common)

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
