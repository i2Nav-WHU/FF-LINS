cmake_minimum_required(VERSION 3.10)
project(common)

include_directories(
    ${PROJECT_SOURCE_DIR}/../)

add_library(${PROJECT_NAME} SHARED
    earth.cc
    logging.cc
    misc.cc
    rotation.cc
    timecost.cc)

## Libraries

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# glog
find_package(Glog REQUIRED)
target_link_libraries(${PROJECT_NAME} ${GLOG_LIBRARIES})

## ThirdParty Libraries

# abseil
target_link_libraries(${PROJECT_NAME}
    absl::strings
    absl::str_format
    absl::time)
