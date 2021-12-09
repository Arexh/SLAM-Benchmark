cmake_minimum_required(VERSION 2.8.3)
project(slam_benchmark)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -I/usr/include/python2.7")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# # find python package - start
# # copy from: https://github.com/lava/matplotlib-cpp/blob/master/CMakeLists.txt
# find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
# target_link_libraries(matplotlib_cpp INTERFACE
#   Python3::Python
#   Python3::Module
# )
# find_package(Python3 COMPONENTS NumPy)
# if(Python3_NumPy_FOUND)
#   target_link_libraries(matplotlib_cpp INTERFACE
#     Python3::NumPy
#   )
# else()
#   target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
# endif()
# install(
#   TARGETS matplotlib_cpp
#   EXPORT install_targets
# )
# # find python package - end

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  matplotlib-cpp/
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_library(SLAMBenchmarkLibs
    src/SystemInfoManager.cpp
    src/SystemInfo.cpp
    )

target_link_libraries(SLAMBenchmarkLibs)

add_executable(systemInfoSummary test/SystemInfoSummary.cpp)

target_link_libraries(systemInfoSummary 
  SLAMBenchmarkLibs
  -lpthread
  -lpython2.7)