/*
 * @Author: lihang lihang@kilox.cn
 * @Date: 2025-09-29 11:12:28
 * @LastEditors: lihang lihang@kilox.cn
 * @LastEditTime: 2025-09-29 15:00:06
 * @FilePath: /fast_lvio_ws/src/open_slam/include/localizer/map_align.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE

 */
#pragma once

#include "common/commons.hh"
#include "localizer/point_cloud_registration.hpp"
#include "utils/logger.hh"

namespace slam {
template <typename T>
class SceneAlignment {
   public:
    float m_line_res = 0.2;
    float m_plane_res = 0.2;
    pcl::VoxelGrid<PointXYZI> m_down_sample_filter_line_source, m_down_sample_filter_line_target;
    pcl::VoxelGrid<PointXYZI> m_down_sample_filter_surface_source, m_down_sample_filter_surface_target;
    int pair_idx = 0;
    Point_cloud_registration m_pc_reg;
    std::string m_save_path;
    int m_para_scene_alignments_maximum_residual_block = 5000;
    int m_maximum_icp_iteration = 10;
    float m_accepted_threshold = 0.2;

    SceneAlignment() {
        pair_idx = 0;
        set_downsample_resolution(m_line_res, m_plane_res);
    }

    void set_downsample_resolution(const float &line_res, const float &plane_res) {
        m_line_res = line_res;
        m_plane_res = plane_res;
        m_down_sample_filter_line_source.setLeafSize(m_line_res, m_line_res, m_line_res);
        m_down_sample_filter_surface_source.setLeafSize(m_plane_res, m_plane_res, m_plane_res);
        m_down_sample_filter_line_target.setLeafSize(m_line_res, m_line_res, m_line_res);
        m_down_sample_filter_surface_target.setLeafSize(m_plane_res, m_plane_res, m_plane_res);
    }

    int find_tranfrom_of_two_mappings(PointCloudXYZIPtr source_cloud, PointCloudXYZIPtr target_cloud) {
        auto rT1 = boost::posix_time::microsec_clock::local_time();

        PointCloudXYZIPtr sourece_pt_line = source_cloud;
        PointCloudXYZIPtr sourece_pt_plane = source_cloud;

        PointCloudXYZIPtr target_pt_line = target_cloud;
        PointCloudXYZIPtr target_pt_plane = target_cloud;
        // Point cloud of downsampled
        PointCloudXYZIPtr sourece_pt_line_ds(new PointCloudXYZI), sourece_pt_plane_ds(new PointCloudXYZI);
        PointCloudXYZIPtr target_pt_line_ds(new PointCloudXYZI), target_pt_plane_ds(new PointCloudXYZI);

        m_down_sample_filter_line_source.setInputCloud(sourece_pt_line);
        m_down_sample_filter_surface_source.setInputCloud(sourece_pt_plane);
        m_down_sample_filter_line_target.setInputCloud(target_pt_line);
        m_down_sample_filter_surface_target.setInputCloud(target_pt_plane);

        auto rT2 = boost::posix_time::microsec_clock::local_time();
        double time_use_1 = (rT2 - rT1).total_microseconds() * 1e-6;

        // m_maximum_icp_iteration 10
        m_pc_reg.m_current_frame_index = 10000000;
        m_pc_reg.m_q_w_curr.setIdentity();
        m_pc_reg.m_q_w_last.setIdentity();
        m_pc_reg.m_t_w_last.setZero();
        m_pc_reg.m_para_icp_max_iterations = m_maximum_icp_iteration;
        m_pc_reg.m_para_cere_max_iterations = 50;
        m_pc_reg.m_para_cere_prerun_times = 2;
        m_pc_reg.m_maximum_allow_residual_block = m_para_scene_alignments_maximum_residual_block;

        Eigen::Matrix<double, 3, 1> transform_T;
        Eigen::Quaterniond transform_R = Eigen::Quaterniond::Identity();
        transform_T(0, 0) = 0;
        transform_T(1, 0) = 0;
        transform_T(2, 0) = 0;
        m_pc_reg.m_t_w_incre = transform_T;
        m_pc_reg.m_t_w_curr = transform_T;
        m_pc_reg.gicp_has_conv = false;
        m_line_res = 0.1;
        m_plane_res = 0.1;
        double downsample_rate_1_time = 0;
        double downsample_rate_2_time = 0;
        double downsample_rate_ceres_time = 0;
        double downsample_rate_1_2_time = 0;
        for (int scale = 8; scale >= 0; scale -= 4) {
            // timer.tic("Each omp");
            // m_line_res 0.4 plane_res 0.4
            // 0.8 , 0.4, 0.1 三个分辨率的点云
            float line_res = m_line_res * scale;
            float plane_res = m_plane_res * scale;
            if (line_res < m_line_res) {
                line_res = m_line_res;
            }

            if (plane_res < m_plane_res) {
                plane_res = m_plane_res;
                m_pc_reg.m_para_icp_max_iterations = m_maximum_icp_iteration * 2;
            }

            rT1 = boost::posix_time::microsec_clock::local_time();

            m_down_sample_filter_surface_source.setLeafSize(plane_res, plane_res, plane_res);
            m_down_sample_filter_line_source.setLeafSize(line_res, line_res, line_res);
            m_down_sample_filter_line_target.setLeafSize(line_res, line_res, line_res);
            m_down_sample_filter_surface_target.setLeafSize(plane_res, plane_res, plane_res);
            m_down_sample_filter_surface_source.filter(*sourece_pt_plane_ds);
            m_down_sample_filter_surface_target.filter(*target_pt_plane_ds);
            m_down_sample_filter_line_source.filter(*sourece_pt_line_ds);
            m_down_sample_filter_line_target.filter(*target_pt_line_ds);

            rT2 = boost::posix_time::microsec_clock::local_time();
            time_use_1 = (rT2 - rT1).total_microseconds() * 1e-6;
            downsample_rate_1_2_time += time_use_1;

            m_pc_reg.gicp_has_conv = true;

            rT1 = boost::posix_time::microsec_clock::local_time();
            m_pc_reg.find_out_incremental_transfrom(sourece_pt_line_ds, sourece_pt_plane_ds, target_pt_line_ds,
                                                    target_pt_plane_ds);
            rT2 = boost::posix_time::microsec_clock::local_time();
            time_use_1 = (rT2 - rT1).total_microseconds() * 1e-6;
            downsample_rate_2_time += time_use_1;
            if (m_pc_reg.m_inlier_threshold > m_accepted_threshold * 2) break;
        }

        target_pt_line_ds->clear();
        sourece_pt_plane_ds->clear();
        target_pt_line_ds->clear();
        target_pt_plane_ds->clear();
        return m_pc_reg.m_inlier_threshold;
    };

    ~SceneAlignment(){};
};
}  // namespace slam