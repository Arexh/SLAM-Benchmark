cmake_minimum_required(VERSION 2.8.3)
project(slam_benchmark)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -I/usr/include/python2.7")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  matplotlib-cpp/
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_library(SLAMBenchmarkLib SHARED
    src/Utility.cpp
    src/SystemInfoManager.cpp
    src/SystemInfo.cpp
    )

target_link_libraries(SLAMBenchmarkLib)

add_executable(systemInfoSummary test/SystemInfoSummary.cpp)

target_link_libraries(systemInfoSummary 
  SLAMBenchmarkLib
  -lpthread
  -lpython2.7)

add_executable(testSystemInfoManager test/TestSystemInfoManager.cpp)

target_link_libraries(testSystemInfoManager 
  SLAMBenchmarkLib
  -lpthread
  -lpython2.7)