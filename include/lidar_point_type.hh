#pragma once
#include <pcl/point_types.h>

/**
    因为需要支持各种雷达，所以需要定义多种点云格式
*/
namespace slam {

struct RsPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring = 0;
    double timestamp = 0.0;  // unit: s

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct LsPointXYZIRT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY

    std::uint16_t ring = 0;
    double timestamp = 0.0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    std::uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint32_t t;  // uint: ns
    std::uint16_t reflectivity;
    std::uint8_t ring;
    std::uint16_t noise;
    std::uint32_t range;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct LivoxMid360PointXYZITLT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY

    std::uint8_t tag;
    std::uint8_t line;
    double timestamp;  // 精确到纳秒(ns)的UTC时间戳
} EIGEN_ALIGN16;

struct LivoxPointXYZITLT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY
    std::uint32_t time;  // unit: ns
    std::uint8_t line;
    std::uint8_t tag;
} EIGEN_ALIGN16;

struct PointXYZIRT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    std::uint8_t ring;
    double time;  // offset time relative to the first point. unit: s

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace slam

// 速腾雷达
POINT_CLOUD_REGISTER_POINT_STRUCT(slam::RsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      std::uint16_t, ring, ring)(double, timestamp, timestamp));
// 镭神雷达
POINT_CLOUD_REGISTER_POINT_STRUCT(slam::LsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      std::uint16_t, ring, ring)(double, timestamp, timestamp));
POINT_CLOUD_REGISTER_POINT_STRUCT(slam::VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(std::uint16_t, ring,
                                                                                     ring)(float, time, time));
// mid360雷达
POINT_CLOUD_REGISTER_POINT_STRUCT(slam::LivoxMid360PointXYZITLT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      std::uint8_t, tag, tag)(std::uint8_t, line, line)(double, timestamp, timestamp));

// livox
POINT_CLOUD_REGISTER_POINT_STRUCT(slam::LivoxPointXYZITLT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      std::uint32_t, time, time)(std::uint8_t, line, line)(std::uint8_t, tag, tag));
// 通用的点云格式
POINT_CLOUD_REGISTER_POINT_STRUCT(slam::PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(std::uint8_t, ring,
                                                                                     ring)(double, time, time));
