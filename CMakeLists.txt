cmake_minimum_required(VERSION 3.5)
project(lio_slam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(-fPIC)

# debug模式
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS " ***  build with debug mode !!!  ****  ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
else ()
    message(STATUS " ***  build with release mode !!!  ****  ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

# ============================================================
#  与 ROS 版本无关的公共第三方库
# ============================================================
# boost
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time timer serialization)
# pcl
find_package(PCL REQUIRED)
# spdlog
find_package(spdlog REQUIRED)
# gtsam(localizer/PGO 使用,必需)
find_package(GTSAM REQUIRED)
# ceres
find_package(Ceres REQUIRED)
# yaml-cpp
find_package(yaml-cpp REQUIRED)
# eigen
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

# geo_graphic库
file(GLOB GEO_GRAPHIC_SRC
    thirdparty/geo_graphic/*.cpp
)
file(GLOB GEO_GRAPHIC_HEAD
    thirdparty/geo_graphic/*.h
    thirdparty/geo_graphic/*.hpp
)
add_library(geo_graphic ${GEO_GRAPHIC_SRC} ${GEO_GRAPHIC_HEAD})

# 公共算法源文件(ROS1/ROS2 共用,通过 ROS_AVAILABLE 宏区分 ROS 接口层)
set(ALGORITHM_SOURCES
    src/system/system.cc
    src/system/system_config.cc
    src/lio/lidar_process.cc
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
    src/lidar_register/superlio_register.cc
    src/encoder/encoder_process.cc
    src/gnss/gnss_process.cc
    src/localizer/localizer.cc
)

# ============================================================
#  选择 ROS1(catkin)/ROS2(ament_cmake) 构建
# ============================================================
find_package(catkin QUIET COMPONENTS roscpp)
find_package(ament_cmake QUIET)
if(catkin_FOUND)
    message(STATUS "  >>>  ROS1(catkin) environment detected, include cmake/ROS1.cmake")
    add_definitions(-DROS_AVAILABLE=1)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
elseif(ament_cmake_FOUND)
    message(STATUS "  >>>  ROS2(ament_cmake) environment detected, include cmake/ROS2.cmake")
    add_definitions(-DROS_AVAILABLE=2)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS2.cmake)
else()
    message(FATAL_ERROR "Neither catkin(ROS1) nor ament_cmake(ROS2) is found !")
endif()
