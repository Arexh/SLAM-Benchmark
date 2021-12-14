cmake_minimum_required(VERSION 2.8.3)
project(slam_benchmark)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -I/usr/include/python2.7")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# define the path of ORB_SLAM2's setting and vocabulary
ADD_DEFINITIONS(-D_ORB_SLAM2_VOC_PATH=\"${PROJECT_SOURCE_DIR}/third_party/ORB_SLAM2_detailed_comments/Vocabulary/ORBvoc.txt\")
ADD_DEFINITIONS(-D_ORB_SLAM2_TUM_SETTING_PATH=\"${PROJECT_SOURCE_DIR}/third_party/ORB_SLAM2_detailed_comments/Examples/Monocular/TUM1.yaml\")
ADD_DEFINITIONS(-D_ORB_SLAM2_EuRoC_SETTING_PATH=\"${PROJECT_SOURCE_DIR}/third_party/ORB_SLAM2_detailed_comments/Examples/Monocular/EuRoC.yaml\")

# ---------------- include nlohmann_json ----------------
# disable JSON library's tests
set(JSON_BuildTests
    OFF
    CACHE INTERNAL "")
# include as PRIVATE source files, no install
set(JSON_Install
    OFF
    CACHE INTERNAL "")
# specify the directory
add_subdirectory(third_party/nlohmann_json)
# ---------------- include nlohmann_json ----------------

# ---------------- include orb_slam2 ----------------
# disable executable
set(ORB_SLAM2_EXECUTABLE
    OFF
    CACHE INTERNAL "")
# specify the directory
add_subdirectory(third_party/ORB_SLAM2_detailed_comments)
# ---------------- include orb_slam2 ----------------

include_directories(
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/third_party
${PROJECT_SOURCE_DIR}/third_party/matplotlib-cpp/
SYSTEM ${PROJECT_SOURCE_DIR}/third_party/nlohmann_json/include
SYSTEM ${PROJECT_SOURCE_DIR}/third_party/ORB_SLAM2_detailed_comments/include
SYSTEM ${PROJECT_SOURCE_DIR}/third_party/ORB_SLAM2_detailed_comments
SYSTEM ${EIGEN3_INCLUDE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_library(
  SLAMBenchmarkLib
  src/Utility.cpp src/SystemInfoManager.cpp src/SystemInfo.cpp
  src/SystemRecorder.cpp src/ThreadRecorder.cpp
  src/BenchmarkManager.cpp)

target_link_libraries(SLAMBenchmarkLib)

add_executable(systemInfoSummary test/SystemInfoSummary.cpp)

target_link_libraries(systemInfoSummary SLAMBenchmarkLib -lpthread -lpython2.7
                      -lboost_system)

target_link_libraries(systemInfoSummary PRIVATE nlohmann_json::nlohmann_json)

add_executable(testSystemInfoManager test/TestSystemInfoManager.cpp)

target_link_libraries(testSystemInfoManager SLAMBenchmarkLib -lpthread
                      -lpython2.7 -lboost_system)

target_link_libraries(testSystemInfoManager
                      PRIVATE nlohmann_json::nlohmann_json)

add_executable(testJSON test/TestJSON.cpp)

target_link_libraries(testJSON PRIVATE nlohmann_json::nlohmann_json)

add_executable(testSystemRecorder test/TestSystemRecorder.cpp)

target_link_libraries(testSystemRecorder SLAMBenchmarkLib -lpthread -lpython2.7
                      -lboost_system)

target_link_libraries(testSystemRecorder PRIVATE nlohmann_json::nlohmann_json)

add_executable(testORBSLAM2 test/TestORBSLAM2.cpp)

target_link_libraries(testORBSLAM2 SLAMBenchmarkLib ORB_SLAM2)