# ============================================================
#  ROS1(catkin) 专用构建逻辑 —— 由顶层 CMakeLists.txt include
#  等价于 ROS1 时代的 CMakeLists.txt,保证 catkin 构建行为不变
# ============================================================

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

include_directories(
 src
 ${PROJECT_SOURCE_DIR}
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
    geo_graphic
)

# 算法库(公共算法源码 + ROS1 管理器)
set(source_lib_srcs
    ${ALGORITHM_SOURCES}
    src/ros/ros1_manager.cc
)
add_library(source_lib STATIC ${source_lib_srcs})
add_dependencies(source_lib robot_manager_generate_messages_cpp)
target_link_libraries(source_lib ${thirdparty_libs})

# slam 节点
add_executable(slam_node src/app/map_node.cc)
add_dependencies(slam_node robot_manager_generate_messages_cpp)
target_link_libraries(slam_node source_lib)

# pgo 后端节点(gtsam 存在时编译)
if(GTSAM_FOUND)
    add_executable(pgo_node src/app/pgo_node.cc
        src/PGO/PosegraphOptimization.cc
    )
    target_link_libraries(pgo_node source_lib)
endif()

catkin_install_python(PROGRAMS
    scripts/pub_map.py
    scripts/pub_slam_pose.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
