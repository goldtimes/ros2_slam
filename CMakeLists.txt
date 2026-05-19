project(lio_slam)
cmake_minimum_required(VERSION 3.5)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# debug模式
# if("${ENABLE_DEBUG}")
#     message(STATUS " ***  build with debug mode !!!  ****  ")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
# else ()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
# endif()

set(
    ROS_PACKAGE 
    roscpp 
    rospy
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
    robot_manager
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

# find_package(glog REQUIRED)

#eigen
find_package(Eigen3 REQUIRED)
# openmp
FIND_PACKAGE(OpenMP REQUIRED) 
if(OpenMP_CXX_FOUND)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=6)

include_directories(
  ${PROJECT_SOURCE_DIR}/thirdparty
)

#include 
include_directories(
 src
 ${CMAKE_CURRENT_SOURCE_DIR}
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

# geo_graphic库
file(GLOB GEO_GRAPHIC_SRC 
    thirdparty/geo_graphic/*.cpp
)
file(GLOB GEO_GRAPHIC_HEAD
    thirdparty/geo_graphic/*.h
    thirdparty/geo_graphic/*.hpp
)
add_library(geo_graphic ${GEO_GRAPHIC_SRC} ${GEO_GRAPHIC_HEAD})


list(APPEND thirdparty_libs
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Sophus_LIBRARIES}
    ${CERES_LIBRARIES}
    glog::glog
    gtsam
    yaml-cpp
    spdlog::spdlog
    geo_graphic
)

add_executable(slam_node 
    src/app/map_node.cc
    src/system/system.cc
    src/system/system_config.cc
    src/lio/lidar_process.cc
    src/ros/ros1_manager.cc
    src/frontend/front_end.cc
    src/lio/static_imu_init.cc
    src/lio/state.cc
    src/lio/ieskf.cc
    src/lio/propogator.cc   
    src/utils/pointcloud_utils.cc
    src/lidar_register/voxelmap_register.cc
    src/lidar_register/voxel_map.cc
    src/lio/fast/ikd_tree.cc
    src/lidar_register/p2plane_register.cc
    src/lidar_register/inc_ndt_register.cc
    src/lidar_register/inc_ndt.cc
    src/lidar_register/fasterlio_register.cc
    src/encoder/encoder_process.cc
    src/gnss/gnss_process.cc
    src/localizer/localizer.cc
    src/loop_closure/loop_closure_detector.cc
)
add_dependencies(slam_node robot_manager_generate_messages_cpp)
target_link_libraries(slam_node 
${thirdparty_libs}
)


catkin_install_python(PROGRAMS
    scripts/pub_map.py
    scripts/pub_slam_pose.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)