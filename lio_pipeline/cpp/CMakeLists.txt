cmake_minimum_required(VERSION 3.10)
project(lio_pipeline_cpp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

set(SOURCES src/imu.cpp)
set(HEADERS include/lio_pipeline/imu.hpp)

add_library(lio_pipeline_cpp SHARED ${SOURCES} ${HEADERS})
target_link_libraries(lio_pipeline_cpp Eigen3::Eigen)
target_include_directories(lio_pipeline_cpp PUBLIC include)

if(BUILD_TESTING)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    find_package(GTest REQUIRED)
    add_executable(imu_test test/imu_test.cpp)
    target_link_libraries(imu_test lio_pipeline_cpp GTest::GTest GTest::Main)
    target_include_directories(imu_test PUBLIC include)
endif()