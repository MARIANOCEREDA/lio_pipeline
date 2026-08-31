cmake_minimum_required(VERSION 3.10)
project(lio_pipeline_cpp)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}" CACHE PATH "Default install prefix" FORCE)
endif()


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(SOURCES 
    src/imu.cpp
    src/synchronizer.cpp)

set(HEADERS 
    include/lio_pipeline_cpp/imu.hpp
    include/lio_pipeline_cpp/synchronizer.hpp
)

add_library(lio_pipeline_cpp SHARED ${SOURCES} ${HEADERS})
target_link_libraries(lio_pipeline_cpp Eigen3::Eigen ${PCL_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Config
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

install(EXPORT ${PROJECT_NAME}Config
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)

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
    add_executable(${PROJECT_NAME}_test test/imu_test.cpp test/test_synchronizer.cpp)
    target_link_libraries(${PROJECT_NAME}_test lio_pipeline_cpp GTest::GTest GTest::Main)
    target_include_directories(${PROJECT_NAME}_test PUBLIC include)
endif()