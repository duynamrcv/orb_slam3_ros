cmake_minimum_required(VERSION 2.8)
project(orb_slam3_ros)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# set(TORCH_PATH "/home/vietnv/gianglt/libtorch/include/torch/csrc/api/include")
# set(Torch_DIR ${TORCH_PATH})
# set(CMAKE_PREFIX_PATH Torch_DIR)
# set(Torch_LIBRARIES "/home/vietnv/gianglt/libtorch/lib")
# find_library(LIBRARY_TORCH torch PATHS ${Torch_LIBRARIES})
# if(Torch_VERSION GREATER 1.0.1)
#    message(STATUS "Torch version is newer than v1.0.1, will use new api")
#    add_definitions(-DTORCH_NEW_API)
# endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/orb_slam3/cmake_modules)

find_package(OpenCV)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)
find_package(realsense2)

find_package (catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  tf
  tf2_geometry_msgs
  sensor_msgs
  message_generation
  dynamic_reconfigure
)

set (DYNAMIC_RECONFIGURE_PATH ros/config/dynamic_reconfigure.cfg)
execute_process(COMMAND chmod a+x ${DYNAMIC_RECONFIGURE_PATH}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        RESULT_VARIABLE cmd_result
        OUTPUT_VARIABLE cmd_ver)
message(STATUS "Chmod a+x the dynamic_reconfigure file")

generate_dynamic_reconfigure_options(${DYNAMIC_RECONFIGURE_PATH})

catkin_package (
   CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs dynamic_reconfigure message_runtime
   LIBRARIES {PROJECT_NAME}
)


include_directories(
${PROJECT_SOURCE_DIR}/orb_slam3
${PROJECT_SOURCE_DIR}/orb_slam3/include
${PROJECT_SOURCE_DIR}/orb_slam3/include/CameraModels
${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/Sophus
${PROJECT_SOURCE_DIR}/ros/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
# ${TORCH_PATH}
# /home/vietnv/gianglt/libtorch/include
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/orb_slam3/lib)

add_library(${PROJECT_NAME} SHARED
  orb_slam3/src/System.cc
  orb_slam3/src/Tracking.cc
  orb_slam3/src/LocalMapping.cc
  orb_slam3/src/LoopClosing.cc
  orb_slam3/src/ORBextractor.cc
  # orb_slam3/src/GCNextractor.cc
  orb_slam3/src/ORBmatcher.cc
  orb_slam3/src/FrameDrawer.cc
  orb_slam3/src/Converter.cc
  orb_slam3/src/MapPoint.cc
  orb_slam3/src/KeyFrame.cc
  orb_slam3/src/Atlas.cc
  orb_slam3/src/Map.cc
  orb_slam3/src/MapDrawer.cc
  orb_slam3/src/Optimizer.cc
  orb_slam3/src/Frame.cc
  orb_slam3/src/KeyFrameDatabase.cc
  orb_slam3/src/Sim3Solver.cc
  orb_slam3/src/Viewer.cc
  orb_slam3/src/ImuTypes.cc
  orb_slam3/src/G2oTypes.cc
  orb_slam3/src/CameraModels/Pinhole.cpp
  orb_slam3/src/CameraModels/KannalaBrandt8.cpp
  orb_slam3/src/OptimizableTypes.cpp
  orb_slam3/src/MLPnPsolver.cpp
  orb_slam3/src/GeometricTools.cc
  orb_slam3/src/TwoViewReconstruction.cc
  orb_slam3/src/Config.cc
  orb_slam3/src/Settings.cc
)

add_subdirectory(orb_slam3/Thirdparty/g2o)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so
-lboost_serialization
-lcrypto
# ${LIBRARY_TORCH}
# /home/vietnv/gianglt/libtorch/lib/libc10.so
# /home/vietnv/gianglt/libtorch/lib/libtorch.so
)

## Build ROS nodes ##
set(LIBS
  ${PROJECT_SOURCE_DIR}/orb_slam3/lib/lib${PROJECT_NAME}.so
  ${EIGEN3_LIBS}
  ${catkin_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBS}
)

# RGB-D node
add_executable (${PROJECT_NAME}_rgbd
  ros/src/rgbd_node.cc
  ros/src/Utils.cc
)
target_link_libraries(${PROJECT_NAME}_rgbd ${LIBS})

add_executable (${PROJECT_NAME}_gridmap
  ros/src/Gridmap.cc
)
target_link_libraries(${PROJECT_NAME}_gridmap ${LIBS})

add_executable (${PROJECT_NAME}_test
  ros/src/test_point.cc
)
target_link_libraries(${PROJECT_NAME}_test ${LIBS})