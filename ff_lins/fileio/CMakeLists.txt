cmake_minimum_required(VERSION 3.10)
project(fileio)

# fileio
include_directories(
        ${PROJECT_SOURCE_DIR}/../)
add_library(${PROJECT_NAME} SHARED
        fileloader.cc
        filesaver.cc)

# abseil
target_link_libraries(${PROJECT_NAME}
        absl::strings
        absl::str_format)
