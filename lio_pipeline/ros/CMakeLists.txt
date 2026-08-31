cmake_minimum_required(VERSION 3.5)
project(lio_pipeline)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}" CACHE PATH "Default install prefix" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

set(LIO_PIPELINE_CPP_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../cpp/build"
  CACHE PATH "Installation prefix of the lio_pipeline C++ library")

if(NOT EXISTS "${LIO_PIPELINE_CPP_PREFIX}/lib/liblio_pipeline_cpp.so")
  message(FATAL_ERROR
  "liblio_pipeline_cpp.so not found in '${LIO_PIPELINE_CPP_PREFIX}/lib'.\n"
  "Run 'make build-cpp' from the workspace root first,\n"
  "or set -DLIO_PIPELINE_CPP_PREFIX=<path> to the library installation prefix.")
endif()

list(PREPEND CMAKE_PREFIX_PATH "${LIO_PIPELINE_CPP_PREFIX}")
find_package(lio_pipeline_cpp CONFIG REQUIRED)

set(HEADERS include/lio_pipeline/lio_pipeline_node.hpp)
set(SOURCES src/lio_pipeline_node.cpp)

add_executable(lio_pipeline_node ${SOURCES} ${HEADERS})
ament_target_dependencies(lio_pipeline_node rclcpp rclcpp_lifecycle sensor_msgs pcl_conversions pcl_ros)
target_link_libraries(lio_pipeline_node 
  lio_pipeline_cpp::lio_pipeline_cpp 
  ${PCL_LIBRARIES}
)
include_directories(${LIO_PIPELINE_CPP_PREFIX}/include)
target_include_directories(lio_pipeline_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  lio_pipeline_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()