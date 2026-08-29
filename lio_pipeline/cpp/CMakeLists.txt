cmake_minimum_required(VERSION 3.10)
project(lio_pipeline_cpp)

set(CMAKE_CXX_STANDARD 20)

find_package(Eigen3 REQUIRED)

set(SOURCES src/imu.cpp)
set(HEADERS include/lio_pipeline/imu.hpp)

add_library(lio_pipeline_cpp SHARED ${SOURCES} ${HEADERS})
target_link_libraries(lio_pipeline_cpp Eigen3::Eigen)
target_include_directories(lio_pipeline_cpp PUBLIC include)