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

#include "ff_lins/ff_lins.h"

#include "common/earth.h"
#include "common/gpstime.h"
#include "common/logging.h"
#include "common/misc.h"
#include "common/timecost.h"
#include "lidar/pointcloud.h"

#include <yaml-cpp/yaml.h>

LINS::LINS(const string &configfile, const string &outputpath, lidar::LidarViewer::Ptr lidar_viewer) {
    // 初始状态
    linsstate_ = LINS_ERROR;

    // 加载配置
    YAML::Node config;
    std::vector<double> vecdata;
    config = YAML::LoadFile(configfile);

    // 文件IO
    lidar_stat_filesaver_ = FileSaver::create(outputpath + "/lidar_statistic.txt", 3);
    lidar_ext_filesaver_  = FileSaver::create(outputpath + "/lidar_extrinsic.txt", 7);
    imu_err_filesaver_    = FileSaver::create(outputpath + "/LINS_IMU_ERR.bin", 7, FileSaver::BINARY);
    traj_filesaver_       = FileSaver::create(outputpath + "/trajectory.csv", 8);

    // make a copy of configuration file
    std::ofstream ofconfig(outputpath + "/ff_lins.yaml");
    ofconfig << YAML::Dump(config);
    ofconfig.close();

    // 积分配置参数
    integration_parameters_          = std::make_shared<IntegrationParameters>();
    integration_parameters_->gravity = NORMAL_GRAVITY;

    // IMU噪声参数
    integration_parameters_->gyr_arw      = config["imu"]["arw"].as<double>() * D2R / 60.0;
    integration_parameters_->gyr_bias_std = config["imu"]["gbstd"].as<double>() * D2R / 3600.0;
    integration_parameters_->acc_vrw      = config["imu"]["vrw"].as<double>() / 60.0;
    integration_parameters_->acc_bias_std = config["imu"]["abstd"].as<double>() * 1.0e-5;
    integration_parameters_->corr_time    = config["imu"]["corrtime"].as<double>() * 3600;

    integration_config_.isuseodo    = false;
    integration_config_.iswithearth = false;
    integration_config_.gravity     = {0, 0, NORMAL_GRAVITY};
    preintegration_options_         = Preintegration::getOptions(integration_config_);

    // IMU数据率
    imudatarate_ = config["imu"]["imudatarate"].as<double>();
    imudatadt_   = 1.0 / imudatarate_;

    // 雷达参数
    lidar_frame_rate_ = config["lidar"]["frame_rate"].as<double>();
    lidar_frame_dt_   = 1.0 / lidar_frame_rate_;

    // 保留的缓存帧数量
    reserved_buffer_counts_ = RESERVED_BUFFER_LENGTH / lidar_frame_dt_;

    // 雷达外参
    vecdata           = config["lidar"]["q_b_l"].as<std::vector<double>>();
    Quaterniond q_b_l = Eigen::Quaterniond(vecdata.data());
    vecdata           = config["lidar"]["t_b_l"].as<std::vector<double>>();
    Vector3d t_b_l    = Eigen::Vector3d(vecdata.data());
    td_b_l_           = config["lidar"]["td_b_l"].as<double>();

    pose_b_l_.R = q_b_l.toRotationMatrix();
    pose_b_l_.t = t_b_l;

    // 可视化
    is_use_visualization_ = config["is_use_visualization"].as<bool>();

    optimize_estimate_lidar_extrinsic_ = config["optimizer"]["optimize_estimate_lidar_extrinsic"].as<bool>();
    optimize_estimate_lidar_td_        = config["optimizer"]["optimize_estimate_lidar_td"].as<bool>();

    // 清理容器
    preintegrationlist_.clear();
    statedatalist_.clear();
    timelist_.clear();

    // 激光处理及可视化
    lidar_map_    = std::make_shared<lidar::LidarMap>(configfile, outputpath);
    lidar_viewer_ = std::move(lidar_viewer);
    if (is_use_visualization_) {
        lidar_viewer_thread_ = std::thread(&lidar::LidarViewer::run, lidar_viewer_);
    }

    // 优化对象
    optimizer_ = std::make_shared<Optimizer>(configfile, integration_parameters_, preintegration_options_, lidar_map_);

    // 融合处理三个线程
    fusion_thread_ = std::thread(&LINS::runFusion, this);

    // 更新系统状态, 进入初始化状态
    linsstate_ = LINS_INITIALIZING;
}

bool LINS::addNewImu(const IMU &imu) {
    static IMU last_imu{0};
    if (imu_buffer_mutex_.try_lock()) {
        double dt = imu.time - last_imu.time;
        if ((dt > imudatadt_ * 1.5) && (last_imu.time != 0)) {
            LOGE << absl::StrFormat("Lost IMU data at %0.3lf with dt %0.3lf", imu.time, dt);

            long cnts = lround(dt / imudatadt_) - 1;

            IMU imudata;
            imudata.time = last_imu.time;
            while (cnts--) {
                imudata.time += imudatadt_;
                imudata.dt = imudatadt_;

                // 等效于速率内插
                double scale   = (imudata.time - last_imu.time) / (imu.time - last_imu.time);
                imudata.dtheta = last_imu.dtheta + scale * (imu.dtheta - last_imu.dtheta);
                imudata.dvel   = last_imu.dvel + scale * (imu.dvel - last_imu.dvel);

                imu_buffer_.push(imudata);
                LOGE << absl::StrFormat("Append extra IMU data at %0.3lf", imudata.time);
            }

            // 当前IMU历元
            imudata.dt     = imu.time - imudata.time;
            imudata.time   = imu.time;
            imudata.dtheta = imu.dtheta / imu.dt * imudata.dt;
            imudata.dvel   = imu.dvel / imu.dt * imudata.dt;
            LOGE << absl::StrFormat("Add current IMU data at %0.3lf", imudata.time);
        } else {
            imu_buffer_.push(imu);
        }
        last_imu = imu;

        imu_buffer_mutex_.unlock();
        return true;
    }

    return false;
}

bool LINS::addNewLidarFrame(const LidarFrame::Ptr &frame) {
    // 需要构建初始地图
    Lock lock(lidar_frame_buffer_mutex_);
    lidar_frame_buffer_.push(frame);

    return true;
}

bool LINS::canAddData() {
    // Buffer容量为空则可以继续加入数据, 注意考虑IMU数据需要超前,
    return lidar_frame_buffer_.size() < reserved_buffer_counts_;
}

bool LINS::isBufferEmpty() {
    return lidar_frame_buffer_.empty();
}

void LINS::setFinished() {
    is_finished_ = true;

    if (is_use_visualization_) {
        lidar_viewer_->setFinished();
        lidar_viewer_thread_.join();
    }
    task_group_.wait();
    fusion_thread_.join();

    if (optimize_estimate_lidar_extrinsic_ || optimize_estimate_lidar_td_) {
        Quaterniond q_b_l = Rotation::matrix2quaternion(pose_b_l_.R);
        Vector3d t_b_l    = pose_b_l_.t;
        LOGW << "Estimated lidar extrinsics: "
             << absl::StrFormat("(%0.6lf, %0.6lf, %0.6lf, %0.6lf), (%0.3lf, %0.3lf, %0.3lf), %0.4lf", q_b_l.x(),
                                q_b_l.y(), q_b_l.z(), q_b_l.w(), t_b_l.x(), t_b_l.y(), t_b_l.z(), td_b_l_);
    }
    LOGW << "LINS has finished processing";
}

void LINS::runFusion() {
    LidarFrame::Ptr lidar_frame = nullptr;
    bool is_need_optimization   = false;
    vector<LidarFrame::Ptr> lidar_nonkeyframes_;

    double last_update_time = 0;

    PointCloudPtr map_pointcloud = PointCloudPtr(new PointCloud);

    LOGI << "Fusion thread is started";
    while (!is_finished_) {
        if ((linsstate_ == LINS_INITIALIZING)) {
            while (!is_finished_ && imu_buffer_.empty()) {
                usleep(WAITING_DELAY_IN_US);
            }
        } else {

            // 等待雷达数据
            while (!is_finished_ && lidar_frame_buffer_.empty()) {
                usleep(WAITING_DELAY_IN_US);
            }

            // 取最新的关键帧
            if (!is_finished_) {
                Lock lock(lidar_frame_buffer_mutex_);
                lidar_frame = lidar_frame_buffer_.front();
                lidar_frame_buffer_.pop();
            }
        }
        if (is_finished_) {
            break;
        }

        if (lidar_frame_buffer_.size() > reserved_buffer_counts_) {
            LOGI << "Buffer size: " << lidar_frame_buffer_.size();
        }

        // 融合状态
        if (linsstate_ == LINS_INITIALIZING) {
            // 移除过多的INS状态
            while (ins_window_.size() > MAXIMUM_INS_NUMBER) {
                ins_window_.pop_front();
            }

            // 初始时刻
            if (ins_window_.empty()) {
                Lock lock(imu_buffer_mutex_);
                auto imu = imu_buffer_.front();

                // 向前取整, 取下一个有效时刻
                last_update_time = ceil(imu.time / INS_INITIALIZATION_INTERVAL) * INS_INITIALIZATION_INTERVAL;
            }

            // 等待IMU数据有效并加入INS窗口
            double current_time = last_update_time + INS_INITIALIZATION_INTERVAL;
            if (!waitImuAddToInsWindow(current_time)) {
                break;
            }

            // 初始化参数, 第一个节点
            if (linsInitialization(last_update_time, current_time)) {
                linsstate_ = LINS_NORMAL;
            }
            last_update_time = current_time;
        } else if (linsstate_ == LINS_NORMAL) {
            int result = linsLidarProcessing(lidar_frame);
            if (result == 1) {

                // 合并非关键帧
                lidar_map_->mergeNonKeyframes(lidar_nonkeyframes_, lidar_frame);
                LOGI << "Accumulate " << lidar_nonkeyframes_.size() + 1 << " frames at "
                     << Logging::doubleData(lidar_frame->stamp()) << " with " << lidar_frame->pointCloudMap()->size()
                     << " points";
                lidar_nonkeyframes_.clear();

                // 加入新的雷达帧
                addNewLidarFrameTimeNode(lidar_frame, true);

                // 更新点云特征
                lidar_map_->updateLidarFeaturesInMap();

                is_need_optimization = true;
            } else if (result == 0) {
                // 非关键帧累积
                lidar_nonkeyframes_.push_back(lidar_frame);
            }
        }

        if (is_need_optimization) {
            is_need_optimization = false;

            // 优化求解
            linsOptimizationProcessing();

            // 更新窗口内状态
            auto state = Preintegration::stateFromData(statedatalist_.back(), preintegration_options_);
            MISC::redoInsMechanization(integration_config_, state, RESERVED_INS_NUM, ins_window_);

            // 可视化
            if (is_use_visualization_) {
                task_group_.run([this, &map_pointcloud]() {
                    // 更新点云地图
                    lidar_map_->addNewPointCloudToMap(map_pointcloud);

                    // 可视化
                    auto frame = lidar_map_->keyframes().back();
                    linsLidarVisualization(frame, map_pointcloud);
                });
            } else {
                task_group_.run([this, &map_pointcloud]() {
                    // 更新点云地图
                    lidar_map_->addNewPointCloudToMap(map_pointcloud);
                });
            }
        }

    } // while
    LOGI << "Fusion thread is exited";
}

bool LINS::linsOptimizationProcessing() {

    // 两次非线性优化并进行粗差剔除
    optimizer_->optimization(timelist_, statedatalist_, preintegrationlist_);

    if (lidar_map_->isWindowFull()) {
        optimizer_->marginalization(timelist_, statedatalist_, preintegrationlist_);
    }

    // 更新外参参数
    updateParametersFromOptimizer();

    // 统计并输出相关的参数
    parametersStatistic();

    return true;
}

bool LINS::linsLidarVisualization(LidarFrame::Ptr frame, PointCloudPtr map_pointcloud) {
    // IMU的位姿
    auto state = Preintegration::stateFromData(statedatalist_.back(), preintegration_options_);
    Pose pose;
    pose.R = state.q.toRotationMatrix();
    pose.t = state.p;
    lidar_viewer_->updateCurrentPose(pose);

    // 新的地图点云
    if (!map_pointcloud->empty()) {
        lidar_viewer_->updateMapPointCloud(map_pointcloud);
    }

    // 当前帧点云
    lidar_viewer_->updateCurrentPointCloud(frame->pointCloudWorld());

    // 当前帧平面点
    PointCloudPtr plane_pointcloud = PointCloudPtr(new PointCloud);
    const auto &features           = frame->features();
    for (size_t k = 0; k < features.size(); k++) {
        if ((features[k]->featureType() == lidar::LidarFeature::FEATURE_PLANE) && !features[k]->isOutlier()) {
            plane_pointcloud->push_back(frame->pointCloudWorld()->points[k]);
        }
    }
    lidar_viewer_->updatePlanePointCloud(plane_pointcloud);

    return true;
}

int LINS::linsLidarProcessing(LidarFrame::Ptr frame) {

    // 更新时标
    frame->setTimeDelay(td_b_l_);
    frame->setStamp(frame->stamp() + td_b_l_);

    // 进行机械编排处理
    if (!waitImuDoInsMechanization(frame->endTime())) {
        return -1;
    }

    // 去畸变降采样
    lidar_map_->lidarFrameProcessing(ins_window_, pose_b_l_, frame);

    // 关键帧选择
    bool is_keyframe = lidar_map_->keyframeSelection(frame);

    return is_keyframe ? 1 : 0;
}

bool LINS::waitImuAddToInsWindow(double time) {
    double end_time = time + imudatadt_ * 2;

    // 等待IMU数据
    waitImuData(end_time);

    // IMU数据加入窗口
    Lock lock(imu_buffer_mutex_);
    while (true) {
        if (imu_buffer_.empty() || (imu_buffer_.front().time > end_time)) {
            break;
        }
        auto imu = imu_buffer_.front();
        imu_buffer_.pop();

        ins_window_.emplace_back(imu, IntegrationState());
    }

    return !is_finished_;
}

bool LINS::waitImuDoInsMechanization(double time) {
    double end_time = time + imudatadt_ * 2;

    // 等待IMU数据
    waitImuData(end_time);

    // INS机械编排
    doInsMechanization(end_time);

    return !is_finished_;
}

bool LINS::waitImuData(double time) {
    while (true) {
        imu_buffer_mutex_.lock();
        if ((!imu_buffer_.empty() && (imu_buffer_.back().time < time)) ||
            (imu_buffer_.empty() && !ins_window_.empty() && ins_window_.back().first.time < time)) {
            imu_buffer_mutex_.unlock();

            usleep(WAITING_DELAY_IN_US);
        } else {
            imu_buffer_mutex_.unlock();
            break;
        }

        if (is_finished_) {
            break;
        }
    }

    return true;
}

void LINS::doInsMechanization(double time) {
    // INS机械编排到最新IMU数据
    auto imu_pre = ins_window_.back().first;
    auto state   = ins_window_.back().second;

    write_state_mutex_.lock();
    write_state_buffer_.clear();
    Lock lock(imu_buffer_mutex_);
    while (true) {
        if (imu_buffer_.empty() || (imu_buffer_.front().time > time)) {
            break;
        }

        auto imu_cur = imu_buffer_.front();
        imu_buffer_.pop();

        MISC::insMechanization(integration_config_, imu_pre, imu_cur, state);
        ins_window_.push_back({imu_cur, state});

        // 保存定位结果
        write_state_buffer_.push_back(state);

        imu_pre = imu_cur;
    }
    write_state_mutex_.unlock();

    // 写入文件
    task_group_.run([this]() {
        Lock lock(write_state_mutex_);
        for (const auto &state : write_state_buffer_) {
            writeNavResult(state);
        }
    });
}

void LINS::addNewLidarFrameTimeNode(LidarFrame::Ptr frame, bool is_add_node) {
    // 添加时间节点
    IntegrationState state;
    if (is_add_node) {
        state = addNewTimeNode(frame->stamp(), NODE_LIDAR);
    } else {
        state = Preintegration::stateFromData(statedatalist_.back(), preintegration_options_);
        LOGI << "Append lidar frame to node " << Logging::doubleData(frame->stamp());
    }

    // 加入关键帧队列
    lidar_map_->addNewKeyFrame(frame);

    // 时间偏移估计参数

    // 速度
    frame->setVelocity(state.v);

    // 角速度, 半个雷达帧间隔内的平均角速度
    Vector3d omega(0, 0, 0);
    auto preintegration    = preintegrationlist_.back();
    const auto &imu_buffer = preintegration->imuBuffer();
    size_t start_index     = static_cast<size_t>((preintegration->deltaTime() - lidar_frame_dt_ * 0.5) / imudatadt_);
    double dt              = 0;
    for (size_t k = start_index; k < imu_buffer.size(); k++) {
        const auto &imu = imu_buffer[k];
        omega += imu.dtheta;
        dt += imu.dt;
    }
    // 补偿零偏
    omega = omega / dt - state.bg;
    frame->setAngularVelocity(omega);
}

IntegrationState LINS::addNewTimeNode(double time, NodeType type) {

    vector<IMU> series;
    IntegrationState state;

    // 获取时段内用于预积分的IMU数据
    double start = timelist_.back();
    double end   = time;
    MISC::getImuSeriesFromTo(ins_window_, start, end, series);

    state = Preintegration::stateFromData(statedatalist_.back(), preintegration_options_);

    // 新建立新的预积分
    {
        preintegrationlist_.emplace_back(
            Preintegration::createPreintegration(integration_parameters_, series[0], state, preintegration_options_));
    }

    // 预积分, 从第二个历元开始
    for (size_t k = 1; k < series.size(); k++) {
        preintegrationlist_.back()->addNewImu(series[k]);
    }

    // 当前状态加入到滑窗中
    state      = preintegrationlist_.back()->currentState();
    state.time = time;
    statedatalist_.emplace_back(Preintegration::stateToData(state, preintegration_options_));

    // 新的时间节点
    LOGI << "Add new " << NODE_NAME.at(type) << " time node at "
         << absl::StrFormat("%0.6lf with dt %0.3lf", time, time - timelist_.back());
    timelist_.push_back(time);

    return state;
}

void LINS::parametersStatistic() {

    // 待输出的参数
    vector<double> parameters;

    // 地图参数
    const auto &statistic = lidar_map_->statisticParameters();
    parameters.insert(parameters.end(), statistic.begin(), statistic.end());

    // 计算耗时
    const auto &timecost = optimizer_->timecosts();
    parameters.insert(parameters.end(), timecost.begin(), timecost.end());

    // 迭代次数
    const auto &iteration = optimizer_->iterations();
    parameters.insert(parameters.end(), iteration.begin(), iteration.end());

    // 粗差雷达因子
    const auto &outliers = optimizer_->outliers();
    parameters.push_back(outliers[2]);

    lidar_stat_filesaver_->dump(parameters);
    lidar_stat_filesaver_->flush();
}

void LINS::updateParametersFromOptimizer() {
    if (optimize_estimate_lidar_td_ || optimize_estimate_lidar_extrinsic_) {
        optimizer_->updateLidarExtrinsic(pose_b_l_, td_b_l_);

        vector<double> extrinsic;
        Vector3d euler = Rotation::matrix2euler(pose_b_l_.R) * R2D;

        extrinsic.push_back(timelist_.back());
        extrinsic.push_back(pose_b_l_.t[0]);
        extrinsic.push_back(pose_b_l_.t[1]);
        extrinsic.push_back(pose_b_l_.t[2]);
        extrinsic.push_back(euler[0]);
        extrinsic.push_back(euler[1]);
        extrinsic.push_back(euler[2]);
        extrinsic.push_back(td_b_l_);

        lidar_ext_filesaver_->dump(extrinsic);
        lidar_ext_filesaver_->flush();
    }
}

bool LINS::linsInitialization(double last_time, double current_time) {

    if (ins_window_.size() < imudatarate_) {
        return false;
    }

    // 缓存数据用于零速检测
    vector<IMU> imu_buff;
    for (const auto &ins : ins_window_) {
        auto &imu = ins.first;
        if ((imu.time > last_time) && (imu.time < current_time)) {
            imu_buff.push_back(imu);
        }
    }

    // 零速检测估计陀螺零偏和横滚俯仰角
    vector<double> average;
    static Vector3d bg{0, 0, 0};
    static Vector3d attitude{0, 0, 0};
    static bool is_has_zero_velocity = false;

    bool is_zero_velocity = MISC::detectZeroVelocity(imu_buff, imudatarate_, average);
    if (is_zero_velocity) {
        // 陀螺零偏
        bg = Vector3d(average[0], average[1], average[2]);
        bg *= imudatarate_;

        // 重力调平获取横滚俯仰角
        Vector3d fb(average[3], average[4], average[5]);
        fb *= imudatarate_;

        attitude[0] = -asin(fb[1] / integration_parameters_->gravity);
        attitude[1] = asin(fb[0] / integration_parameters_->gravity);

        LOGI << "Zero velocity get gyroscope bias " << bg.transpose() * 3600 * R2D << ", roll " << attitude[0] * R2D
             << ", pitch " << attitude[1] * R2D;
        is_has_zero_velocity = true;
    }

    // 里程计速度大于MINMUM_ALIGN_VELOCITY, 或者非零速状态
    Vector3d position{0, 0, 0};
    Vector3d velocity{0, 0, 0};
    double initial_time = last_time;

    // 推算当前时刻速度, 避免错误初始化
    double current_velocity;
    {
        auto state = IntegrationState{
            .time = initial_time,
            .p    = position,
            .q    = Rotation::euler2quaternion(attitude),
            .v    = velocity,
            .bg   = bg,
            .ba   = {0, 0, 0},
            .sodo = 0.0,
            .avb  = {-integration_parameters_->abv[1], integration_parameters_->abv[2]},
        };
        MISC::redoInsMechanization(integration_config_, state, RESERVED_INS_NUM * 2, ins_window_);
        current_velocity = ins_window_.back().second.v.norm();
    }

    if (!is_zero_velocity && (current_velocity > 0.2)) {
        if (!is_has_zero_velocity) {
            // 直接重力调平获取横滚俯仰角
            Vector3d fb(average[3], average[4], average[5]);
            fb *= imudatarate_;

            attitude[0] = -asin(fb[1] / integration_parameters_->gravity);
            attitude[1] = asin(fb[0] / integration_parameters_->gravity);

            LOGW << "Get roll " << attitude[0] * R2D << ", pitch " << attitude[1] * R2D << " without zero velocity";
        }
        attitude[2] = 0;

        // 无轮速直接初始化
        position = Vector3d::Zero();
        velocity = Vector3d::Zero();

    } else {
        // 初始位姿, 横滚俯仰来自重力调平, 航向给定
        attitude[2] = 0;
        auto state  = IntegrationState{
             .p = position,
             .q = Rotation::euler2quaternion(attitude),
        };

        // 累积静态点云到当前时刻
        accumulateStationaryPointcloud(state, current_time);

        // 零速状态返回
        return false;
    }

    // 初始状态, 从上一秒开始
    auto state = IntegrationState{
        .time = initial_time,
        .p    = position,
        .q    = Rotation::euler2quaternion(attitude),
        .v    = velocity,
        .bg   = bg,
        .ba   = {0, 0, 0},
    };

    // 初始时间节点
    statedatalist_.emplace_back(Preintegration::stateToData(state, preintegration_options_));
    timelist_.push_back(initial_time);

    // 初始先验
    optimizer_->constructPrior(statedatalist_.front(), is_has_zero_velocity);

    // 计算第一秒的INS结果
    state = Preintegration::stateFromData(statedatalist_.back(), preintegration_options_);
    MISC::redoInsMechanization(integration_config_, state, RESERVED_INS_NUM, ins_window_);

    start_time_ = current_time;
    LOGI << "Initialization at " << Logging::doubleData(last_time);

    // 当前时间节点
    addNewTimeNode(current_time, NODE_IMU);

    // 初始点云帧

    // 全部投影到当前时刻
    accumulateDynamicPointcloud(current_time);

    // 可视化地图点云
    lidar_viewer_->updateMapPointCloud(lidar_map_->keyframes().back()->pointCloudWorld());

    return true;
}

void LINS::accumulateStationaryPointcloud(const IntegrationState &state, double end_time) {
    vector<LidarFrame::Ptr> lidar_frames;
    {
        Lock lock(lidar_frame_buffer_mutex_);
        while (!lidar_frame_buffer_.empty()) {
            auto frame = lidar_frame_buffer_.front();

            if ((frame->endTime() + td_b_l_) < end_time) {
                // 取出雷达帧
                lidar_frame_buffer_.pop();

                lidar_frames.push_back(frame);
            } else {
                break;
            }
        }
    }

    if (lidar_frames.empty()) {
        return;
    }

    for (auto &frame : lidar_frames) {
        // 更新时标
        frame->setTimeDelay(td_b_l_);
        frame->setStamp(frame->stamp() + td_b_l_);

        // 降采样投影
        lidar_map_->lidarFrameProcessing(state, pose_b_l_, frame);
    }

    // 可视化最新一帧的点云
    lidar_viewer_->updateCurrentPointCloud(lidar_frames.back()->pointCloudWorld());
}

bool LINS::accumulateDynamicPointcloud(double end_time) {
    // 确保雷达数据有效
    LidarFrame::Ptr latest_frame;
    while (!is_finished_) {
        lidar_frame_buffer_mutex_.lock();
        latest_frame = lidar_frame_buffer_.back();
        lidar_frame_buffer_mutex_.unlock();
        if (latest_frame->stamp() > end_time) {
            break;
        }

        usleep(WAITING_DELAY_IN_US);
    }

    vector<LidarFrame::Ptr> lidar_frames;
    {
        Lock lock(lidar_frame_buffer_mutex_);
        while (!lidar_frame_buffer_.empty()) {
            auto frame = lidar_frame_buffer_.front();

            // 所有时间小于当前时刻的帧
            if ((frame->endTime() + td_b_l_) < end_time) {
                // 取出雷达帧
                lidar_frame_buffer_.pop();

                lidar_frames.push_back(frame);
            } else {
                break;
            }
        }
    }

    // 无有效雷达帧
    if (lidar_frames.empty()) {
        return false;
    }

    // 取最新的雷达帧投影作为关键帧, 其他帧合并
    auto keyframe = lidar_frames.back();
    // 关键帧预处理, 投影到当前时刻
    keyframe->setTimeDelay(td_b_l_);
    keyframe->setStamp(end_time);
    lidar_map_->lidarFrameProcessing(ins_window_, pose_b_l_, keyframe);

    vector<LidarFrame::Ptr> non_keyframes;
    if (lidar_frames.size() > 1) {
        for (size_t k = 0; k < lidar_frames.size() - 1; k++) {
            auto frame = lidar_frames[k];

            // 更新时标
            frame->setTimeDelay(td_b_l_);
            frame->setStamp(frame->stamp() + td_b_l_);
            lidar_map_->lidarFrameProcessing(ins_window_, pose_b_l_, frame);

            non_keyframes.push_back(frame);
        }

        // 合并非关键帧
        lidar_map_->mergeNonKeyframes(non_keyframes, keyframe);
    }

    // 加入地图
    addNewLidarFrameTimeNode(keyframe, false);

    // 更新点云特征
    if (lidar_map_->keyframes().size() > 1) {
        lidar_map_->updateLidarFeaturesInMap();
    }

    LOGI << "Accumulate " << non_keyframes.size() << " frames at " << Logging::doubleData(keyframe->stamp()) << " with "
         << keyframe->pointCloudMap()->size() << " points";

    return true;
}

void LINS::writeNavResult(const IntegrationState &state) {

    // 保存结果
    vector<double> result;

    double time = state.time;
    Vector3d bg = state.bg * R2D * 3600;
    Vector3d ba = state.ba * 1e5;

    // imu error file
    result.clear();

    result.push_back(time);
    result.push_back(bg[0]);
    result.push_back(bg[1]);
    result.push_back(bg[2]);
    result.push_back(ba[0]);
    result.push_back(ba[1]);
    result.push_back(ba[2]);

    result.push_back(state.sodo);
    imu_err_filesaver_->dump(result);
    imu_err_filesaver_->flush();

    // trajectory
    result.clear();

    result.push_back(time);
    result.push_back(state.p[0]);
    result.push_back(state.p[1]);
    result.push_back(state.p[2]);
    result.push_back(state.q.x());
    result.push_back(state.q.y());
    result.push_back(state.q.z());
    result.push_back(state.q.w());
    traj_filesaver_->dump(result);
    traj_filesaver_->flush();
}