cmake_minimum_required(VERSION 3.13)
project(slam_benchmark)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -lboost_system -lpthread")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -lboost_system -lpthread")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# define the path of ORB_SLAM2's setting and vocabulary
ADD_DEFINITIONS(-D_ORB_SLAM2_VOC_PATH=\"${PROJECT_SOURCE_DIR}/third_party/ORB_SLAM2_detailed_comments/Vocabulary/ORBvoc.txt\")
ADD_DEFINITIONS(-D_ORB_SLAM2_SETTING_PATH=\"${PROJECT_SOURCE_DIR}/third_party/ORB_SLAM2_detailed_comments/Examples/Monocular\")
# define the path of ORB_SLAM3's setting and vocabulary
ADD_DEFINITIONS(-D_ORB_SLAM3_VOC_PATH=\"${PROJECT_SOURCE_DIR}/third_party/ORB_SLAM3_detailed_comments/Vocabulary/ORBvoc.txt\")
ADD_DEFINITIONS(-D_ORB_SLAM3_SETTING_PATH=\"${PROJECT_SOURCE_DIR}/third_party/ORB_SLAM3_detailed_comments/Examples/Monocular\")
# define EuRoC mh05 timestamps path
ADD_DEFINITIONS(-D_EuRoC_TimeStamps_PATH=\"${PROJECT_SOURCE_DIR}/third_party/ORB_SLAM2_detailed_comments/Examples/Monocular/EuRoC_TimeStamps\")
# define cuda option
option(USE_CUDA "Use Cuda." OFF)

find_package(OpenCV 3 REQUIRED)

message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

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

# ---------------- include orb_slam3 ----------------
# disable executable
set(ORB_SLAM3_COMPILE
    OFF
    CACHE INTERNAL "")
# specify the directory
add_subdirectory(third_party/ORB_SLAM3_detailed_comments)
# ---------------- include orb_slam3 ----------------

# --------------- include VINS-Course ---------------
# specify the directory
add_subdirectory(third_party/VINS-Course)
# --------------- include VINS-Course ---------------

include_directories(
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/third_party
SYSTEM ${PROJECT_SOURCE_DIR}/third_party/nlohmann_json/include
SYSTEM ${PROJECT_SOURCE_DIR}/third_party/ORB_SLAM2_detailed_comments/include
SYSTEM ${PROJECT_SOURCE_DIR}/third_party/ORB_SLAM3_detailed_comments/include
SYSTEM ${PROJECT_SOURCE_DIR}/third_party/ORB_SLAM3_detailed_comments/include/CameraModels
SYSTEM ${PROJECT_SOURCE_DIR}/third_party/ORB_SLAM2_detailed_comments
SYSTEM ${PROJECT_SOURCE_DIR}/third_party/ORB_SLAM3_detailed_comments
SYSTEM ${EIGEN3_INCLUDE_DIR}
SYSTEM ${PROJECT_SOURCE_DIR}/third_party/VINS-Course/include
SYSTEM ${EIGEN3_INCLUDE_DIR}
SYSTEM ${CERES_INCLUDE_DIRS}
SYSTEM ${Pangolin_INCLUDE_DIRS}
SYSTEM ${Boost_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_library(
  SLAMBenchmarkLib
  src/Utility.cpp src/SystemInfoManager.cpp src/SystemInfo.cpp
  src/SystemRecorder.cpp
  src/BenchmarkManager.cpp)

add_library(
  LibThreadRecorder
  src/ThreadRecorder.cpp)

target_link_libraries(ORB_SLAM3
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
-lboost_serialization
-lcrypto
DBoW2
g2o
MyVio
)

target_link_libraries(SLAMBenchmarkLib)

add_executable(systemInfoSummary test/SystemInfoSummary.cpp)

target_link_libraries(systemInfoSummary SLAMBenchmarkLib -lpthread
                      -lboost_system nlohmann_json::nlohmann_json LibThreadRecorder)

add_executable(testSystemInfoManager test/TestSystemInfoManager.cpp)

target_link_libraries(testSystemInfoManager SLAMBenchmarkLib -lpthread
                      -lboost_system nlohmann_json::nlohmann_json LibThreadRecorder)

add_executable(testSystemRecorder test/TestSystemRecorder.cpp)

target_link_libraries(testSystemRecorder SLAMBenchmarkLib -lpthread
                      -lboost_system nlohmann_json::nlohmann_json LibThreadRecorder)

add_executable(SingleBenchmark test/SingleBenchmark.cpp)

target_link_libraries(SingleBenchmark -lboost_system SLAMBenchmarkLib
                      nlohmann_json::nlohmann_json ORB_SLAM2 ORB_SLAM3 LibThreadRecorder)

add_executable(SLAMBenchmark test/SLAMBenchmark.cpp)

target_link_libraries(SLAMBenchmark -lboost_system SLAMBenchmarkLib
                      nlohmann_json::nlohmann_json ORB_SLAM2 ORB_SLAM3 LibThreadRecorder)