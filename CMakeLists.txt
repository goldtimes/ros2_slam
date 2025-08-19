project(open_slam)
cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS true)
# debug模式
if("${ENABLE_DEBUG}")
    message(STATUS " ***  build with debug mode !!!  ****  ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
endif()

set(
    ROS_PACKAGE 
    roscpp 
    rosbag 
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    nav_msgs
    visualization_msgs 
    image_transport 
    cv_bridge
    pcl_conversions
    message_generation
)

# catkin with ros
find_package(catkin REQUIRED COMPONENTS ${ROS_PACKAGE})

catkin_package(
    CATKIN_DEPENDS ${ROS_PACKAGE}
)
# boost
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time timer serialization)

# pcl 
find_package(PCL REQUIRED)

# spdlog 
find_package(spdlog REQUIRED)

# gtsam
find_package(GTSAM REQUIRED)

# ceres
find_package(Ceres REQUIRED)

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# tbb
find_package(TBB REQUIRED)

#eigen
find_package(Eigen3 REQUIRED)
# openmp
FIND_PACKAGE(OpenMP REQUIRED) 
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=4)

include_directories(
  ${PROJECT_SOURCE_DIR}/thirdparty
)

#include 
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
 ${YAML_CPP_INCLUDE_DIRS}
 ${GTSAM_INCLUDE_DIRS}
 ${CERES_INCLUDE_DIRS}
 ${TBB_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
 ${spdlog_INCLUDE_DIRS}
)

list(APPEND thirdparty_libs
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Sophus_LIBRARIES}
    ${CERES_LIBRARIES}
    gtsam
    yaml-cpp
    spdlog::spdlog
)

add_executable(slam_node 
    app/map_node.cc
    src/system.cc
    src/system_config.cc
    src/lidar_process.cc
    src/ros/ros1_manager.cc
    src/front_end.cc
    src/static_imu_init.cc
    src/state.cc
    src/ieskf.cc
    src/propogator.cc
    src/pointcloud_utils.cc
    src/lidar_register/voxelmap_register.cc
    src/lidar_register/voxel_map.cc
)

target_link_libraries(slam_node 
${thirdparty_libs}
)
