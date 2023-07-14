/*
 * FF-LINS: A Consistent Frame-to-Frame Solid-State-LiDAR-Inertial State Estimator 
 *
 * Copyright (C) 2023 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef LINS_H
#define LINS_H

#include "ff_lins/optimizer.h"
#include "fileio/filesaver.h"
#include "lidar/lidar_frame.h"
#include "lidar/lidar_map.h"
#include "lidar/lidar_viewer.h"

#include "preintegration/preintegration.h"

#include <ceres/ceres.h>
#include <tbb/tbb.h>

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <thread>
#include <unordered_map>

using lidar::LidarFrame;

class LINS {

public:
    enum LINSState {
        LINS_ERROR        = -1,
        LINS_INITIALIZING = 0,
        LINS_NORMAL       = 1,
    };

    enum NodeType {
        NODE_IMU   = 0,
        NODE_LIDAR = 1,
    };

    typedef std::shared_ptr<LINS> Ptr;
    typedef std::unique_lock<std::mutex> Lock;

    LINS() = delete;
    explicit LINS(const string &configfile, const string &outputpath, lidar::LidarViewer::Ptr lidar_viewer);

    bool addNewImu(const IMU &imu);
    bool addNewLidarFrame(const LidarFrame::Ptr &frame);
    bool canAddData();
    bool isBufferEmpty();

    void setFinished();

    bool isRunning() const {
        return !is_finished_;
    }

    LINSState linsState() const {
        return linsstate_;
    }

private:
    // 数据处理
    bool linsInitialization(double last_time, double current_time);
    int linsLidarProcessing(LidarFrame::Ptr frame);
    bool linsOptimizationProcessing();
    bool linsLidarVisualization(LidarFrame::Ptr frame, PointCloudPtr map_pointcloud);

    // 时间节点
    IntegrationState addNewTimeNode(double time, NodeType type);
    void addNewLidarFrameTimeNode(LidarFrame::Ptr frame, bool is_add_node);

    // INS数据
    bool waitImuData(double time);
    void doInsMechanization(double time);
    bool waitImuAddToInsWindow(double time);
    bool waitImuDoInsMechanization(double time);

    // 优化参数和因子
    void updateParametersFromOptimizer();
    void parametersStatistic();

    // 线程
    void runFusion();

    void writeNavResult(const IntegrationState &state);

    void accumulateStationaryPointcloud(const IntegrationState &state, double end_time);
    bool accumulateDynamicPointcloud(double end_time);

private:
    // 正常重力
    const double NORMAL_GRAVITY = 9.8; // 9.8 m/s^2

    // INS窗口内的最大数量, 对于200Hz, 保留5秒数据
    const size_t MAXIMUM_INS_NUMBER = 1000;
    const size_t RESERVED_INS_NUM   = 400; // INS窗口内保留的数量

    const double RESERVED_BUFFER_LENGTH = 10.0; // Buffer保存的数据时间长度
    const uint32_t WAITING_DELAY_IN_US  = 100;  // 数据等待微秒时间

    // INS初始化节点的时间间隔
    const double INS_INITIALIZATION_INTERVAL = 1.0; // 1.0 s

    // 时间节点类型
    const std::map<NodeType, string> NODE_NAME = {{NODE_IMU, "IMU"}, {NODE_LIDAR, "lidar"}};

    // 优化参数, 使用deque容器管理, 移除头尾不会造成数据内存移动
    deque<PreBasePtr> preintegrationlist_;
    deque<StateData> statedatalist_;
    deque<double> timelist_;

    // 融合对象
    lidar::LidarMap::Ptr lidar_map_;
    lidar::LidarViewer::Ptr lidar_viewer_;
    Optimizer::Ptr optimizer_;

    // 多线程
    tbb::task_group task_group_;
    std::thread lidar_viewer_thread_;
    std::thread fusion_thread_;

    std::atomic<bool> is_finished_{false};
    std::atomic<double> start_time_{0};

    std::mutex imu_buffer_mutex_;
    std::mutex lidar_point_buffer_mutex_;
    std::mutex lidar_frame_buffer_mutex_;
    std::mutex write_state_mutex_;
    std::mutex visual_tracking_mutex_;

    // 传感器数据
    std::queue<LidarFrame::Ptr> lidar_frame_buffer_;
    std::queue<IMU> imu_buffer_;
    std::deque<std::pair<IMU, IntegrationState>> ins_window_;
    std::deque<PointTypeCustom> lidar_point_buffer_;
    vector<IntegrationState> write_state_buffer_;

    // IMU参数
    std::shared_ptr<IntegrationParameters> integration_parameters_;
    PreintegrationOptions preintegration_options_;
    IntegrationConfiguration integration_config_;

    double imudatarate_{200};
    double imudatadt_{0.005};

    double lidar_frame_rate_;
    double lidar_frame_dt_;
    size_t reserved_buffer_counts_;

    // 外参
    Pose pose_b_l_;
    double td_b_l_;

    bool is_use_visualization_{true};

    // 优化选项
    bool optimize_estimate_lidar_extrinsic_;
    bool optimize_estimate_lidar_td_;

    // 文件IO
    FileSaver::Ptr imu_err_filesaver_;
    FileSaver::Ptr lidar_stat_filesaver_;
    FileSaver::Ptr lidar_ext_filesaver_;
    FileSaver::Ptr traj_filesaver_;

    // 系统状态
    std::atomic<LINSState> linsstate_{LINS_ERROR};
};

#endif // LINS_LINS_H
