# ============================================================
#  ROS2(ament_cmake / colcon) 专用构建逻辑
#  由顶层 CMakeLists.txt 检测到 ament_cmake 后 include
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
# livox ROS2 驱动(位于 thirdparty/livox_ros_driver,包名 livox_ros_driver2)
find_package(livox_ros_driver2 REQUIRED)
# 自定义 srv 生成(SaveMap, 供 pgo_node 保存地图)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMap.srv"
)

include_directories(
 src
 ${PROJECT_SOURCE_DIR}
 ${EIGEN3_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
 ${YAML_CPP_INCLUDE_DIRS}
 ${CERES_INCLUDE_DIRS}
 ${TBB_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
 ${spdlog_INCLUDE_DIRS}
)

set(ament_libraries
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    pcl_conversions
    ament_index_cpp
    builtin_interfaces
    livox_ros_driver2
)

list(APPEND thirdparty_libs
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
    gtsam
    yaml-cpp
    spdlog::spdlog
    geo_graphic
)

# 算法库(公共算法源码 + ROS2 管理器)
set(source_lib_srcs
    ${ALGORITHM_SOURCES}
    src/ros/ros2_manager.cc
)
add_library(source_lib STATIC ${source_lib_srcs})
ament_target_dependencies(source_lib ${ament_libraries})
target_link_libraries(source_lib ${thirdparty_libs})

# slam 节点
add_executable(slam_node src/app/map_node.cc)
ament_target_dependencies(slam_node ${ament_libraries})
target_link_libraries(slam_node source_lib)

# pgo 后端节点(gtsam 存在时编译)
if(GTSAM_FOUND)
    add_executable(pgo_node src/app/pgo_node.cc
        src/PGO/PosegraphOptimization.cc
    )
    ament_target_dependencies(pgo_node ${ament_libraries})
    target_link_libraries(pgo_node source_lib)
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
    target_link_libraries(pgo_node "${cpp_typesupport_target}")
endif()

# 离线全局地图切块工具(无 ROS 依赖)
add_executable(map_split src/app/map_split.cc)
target_link_libraries(map_split
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    yaml-cpp
)

install(TARGETS
    slam_node
    DESTINATION lib/${PROJECT_NAME}
)
if(GTSAM_FOUND)
    install(TARGETS pgo_node DESTINATION lib/${PROJECT_NAME})
endif()
install(TARGETS map_split DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
    scripts/pub_map.py
    scripts/pub_slam_pose.py
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ROS2 DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
