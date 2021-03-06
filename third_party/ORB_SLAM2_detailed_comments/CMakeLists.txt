cmake_minimum_required(VERSION 3.13)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# disable warnings
add_definitions(-w)
# add executable option
option(ORB_SLAM2_EXECUTABLE "Build the executable." ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3  ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 ")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

message(WARNING "ORB_SLAM2 OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# ---------------- include g2o ----------------
# specify the directory
add_subdirectory(Thirdparty/g2o)
# ---------------- include g2o ----------------

# --------------- include DBoW2 ---------------
# specify the directory
add_subdirectory(Thirdparty/DBoW2)
# --------------- include DBoW2 ---------------

# ------- define untar vocabulary task --------
add_custom_target(ORB_SLAM2_UntarVocabularyTask ALL)
add_custom_command(TARGET ORB_SLAM2_UntarVocabularyTask PRE_BUILD
   COMMAND ${CMAKE_COMMAND} -E tar xzf ORBvoc.txt.tar.gz
WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/Vocabulary
DEPENDS ${CMAKE_SOURCE_DIR}/Vocabulary/ORBvoc.txt.tar.gz
COMMENT "- Untar Vocabulary/ORBVoc.txt.tar.gz"
VERBATIM)
# ------- define untar vocabulary task --------

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
DBoW2
g2o
)

# Build examples

if (ORB_SLAM2_EXECUTABLE)

   set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

   add_executable(rgbd_tum
   Examples/RGB-D/rgbd_tum.cc)
   target_link_libraries(rgbd_tum ${PROJECT_NAME})

   set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

   add_executable(stereo_kitti
   Examples/Stereo/stereo_kitti.cc)
   target_link_libraries(stereo_kitti ${PROJECT_NAME})

   add_executable(stereo_euroc
   Examples/Stereo/stereo_euroc.cc)
   target_link_libraries(stereo_euroc ${PROJECT_NAME})


   set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

   add_executable(mono_tum
   Examples/Monocular/mono_tum.cc)
   target_link_libraries(mono_tum ${PROJECT_NAME})

   add_executable(mono_kitti
   Examples/Monocular/mono_kitti.cc)
   target_link_libraries(mono_kitti ${PROJECT_NAME})

   add_executable(mono_euroc
   Examples/Monocular/mono_euroc.cc)
   target_link_libraries(mono_euroc ${PROJECT_NAME})

endif()

