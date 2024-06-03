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

#include "ff_lins/optimizer.h"

#include "common/logging.h"
#include "common/misc.h"
#include "common/timecost.h"

#include "lidar/pointcloud.h"

#include "factors/lidar_factor.h"
#include "factors/marginalization_factor.h"
#include "factors/marginalization_info.h"
#include "factors/pose_manifold.h"
#include "factors/residual_block_info.h"

#include "preintegration/imu_error_factor.h"
#include "preintegration/imu_mix_prior_factor.h"
#include "preintegration/imu_pose_prior_factor.h"
#include "preintegration/preintegration.h"
#include "preintegration/preintegration_factor.h"

#include <tbb/tbb.h>
#include <yaml-cpp/yaml.h>

Optimizer::Optimizer(const string &configfile, std::shared_ptr<IntegrationParameters> parameters,
                     PreintegrationOptions options, lidar::LidarMap::Ptr lidar_map)
    : integration_parameters_(std::move(parameters))
    , preintegration_options_(options)
    , lidar_map_(std::move(lidar_map)) {

    // 配置参数
    YAML::Node config;
    vector<double> vecdata;
    config = YAML::LoadFile(configfile);

    // 优化参数
    optimize_windows_size_             = config["optimizer"]["optimize_window_size"].as<size_t>();
    optimize_estimate_lidar_extrinsic_ = config["optimizer"]["optimize_estimate_lidar_extrinsic"].as<bool>();
    optimize_estimate_lidar_td_        = config["optimizer"]["optimize_estimate_lidar_td"].as<bool>();
    optimize_lidar_extrinsic_accurate_ = config["optimizer"]["optimize_lidar_extrinsic_accurate"].as<bool>();

    gb_prior_std_ = config["imu"]["gb_prior_std"].as<double>();
    ab_prior_std_ = config["imu"]["ab_prior_std"].as<double>();

    // 激光参数
    vecdata           = config["lidar"]["q_b_l"].as<std::vector<double>>();
    Quaterniond q_b_l = Eigen::Quaterniond(vecdata.data());
    vecdata           = config["lidar"]["t_b_l"].as<std::vector<double>>();
    Vector3d t_b_l    = Eigen::Vector3d(vecdata.data());
    double td_b_l     = config["lidar"]["td_b_l"].as<double>();

    memcpy(extrinsic_b_l_, t_b_l.data(), sizeof(double) * 3);
    memcpy(extrinsic_b_l_ + 3, q_b_l.coeffs().data(), sizeof(double) * 4); // x, y, z, w
    extrinsic_b_l_[7] = td_b_l;

    // 统计参数
    iterations_.resize(2);
    timecosts_.resize(4);
    outliers_.resize(3);

    LOGI << "Optimizer is constructed";
}

bool Optimizer::optimization(const deque<double> &timelist, deque<StateData> &statedatalist,
                             deque<PreBasePtr> &preintegrationlist) {
    static int first_num_iterations  = 5;
    static int second_num_iterations = 10;

    TimeCost timecost;

    // 方便后续移除粗差因子
    ceres::Problem::Options problem_options;
    problem_options.enable_fast_removal = true;

    ceres::Problem problem(problem_options);
    ceres::Solver solver;
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type         = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations         = first_num_iterations;
    options.num_threads                = 8;

    // 状态参数
    addStateParameters(problem, statedatalist);

    addLidarParameters(problem);

    // 边缘化残差
    if (last_marginalization_info_ && last_marginalization_info_->isValid()) {
        auto factor = new MarginalizationFactor(last_marginalization_info_);
        problem.AddResidualBlock(factor, nullptr, last_marginalization_parameter_blocks_);
    }

    // 预积分残差
    addImuFactors(problem, statedatalist, preintegrationlist);

    // 激光点到面残差
    LidarResiduals lidar_residuals;
    lidar_residuals = addLidarFactors(problem, true, timelist, statedatalist);

    string msg = absl::StrFormat("Add %llu preintegration", preintegrationlist.size());
    if (!lidar_residuals.empty()) {
        absl::StrAppendFormat(&msg, ", %llu lidar", lidar_residuals.size());
    }
    LOGI << msg << " factors";

    // 第一次优化
    {
        timecost.restart();

        // 求解最小二乘
        solver.Solve(options, &problem, &summary);
        // LOGI << summary.BriefReport();

        iterations_[0] = summary.num_successful_steps;
        timecosts_[0]  = timecost.costInMillisecond();
    }

    // 粗差检测和剔除
    {
        // Detect outlier lidar factors
        ResidualOutliers lidar_outliers;
        lidar_outliers = lidarOutlierCullingByChi2(problem, lidar_residuals);

        int num_outliers = 0;
        // Remove factors only
        for (const auto &outlier : lidar_outliers) {
            if (outlier.second) {
                problem.RemoveResidualBlock(outlier.first);
                num_outliers++;
            }
        }
        outliers_[2] = num_outliers;
        LOGI << "Remove " << num_outliers << " lidar outliers factors";
    }

    // 第二次优化
    {
        // 第二次迭代次数
        options.max_num_iterations = second_num_iterations;

        timecost.restart();

        // 求解最小二乘
        solver.Solve(options, &problem, &summary);
        // LOGI << summary.BriefReport();

        iterations_[1] = summary.num_successful_steps;
        timecosts_[1]  = timecost.costInMillisecond();

        if (!lidar_map_->isWindowFull()) {
            // 进行必要的重积分
            doReintegration(preintegrationlist, statedatalist);
        }
    }

    // 更新参数, 必须的
    updateParametersFromOptimizer(timelist, statedatalist);

    LOGI << "Optimization costs " << timecosts_[0] << " ms and " << timecosts_[1] << ", with iteration "
         << iterations_[0] << " and " << iterations_[1];
    return true;
}

void Optimizer::updateParametersFromOptimizer(const deque<double> &timelist, const deque<StateData> &statedatalist) {
    // 先更新外参
    Pose pose_b_l;
    pose_b_l.t[0] = extrinsic_b_l_[0];
    pose_b_l.t[1] = extrinsic_b_l_[1];
    pose_b_l.t[2] = extrinsic_b_l_[2];

    Quaterniond q_b_l = Quaterniond(extrinsic_b_l_[6], extrinsic_b_l_[3], extrinsic_b_l_[4], extrinsic_b_l_[5]);
    pose_b_l.R        = Rotation::quaternion2matrix(q_b_l.normalized());

    // 保证系统稳定
    if (optimize_estimate_lidar_td_ && (fabs(extrinsic_b_l_[7]) > 0.2)) {
        LOGW << "Estimate large td_b_l " << Logging::doubleData(extrinsic_b_l_[7]) << ", and td_b_l is reset";
        extrinsic_b_l_[7] = 0;
    }

    for (const auto &frame : lidar_map_->keyframes()) {
        // 位姿状态索引
        int index = getStateDataIndex(timelist, frame->stamp());
        if (index < 0) {
            LOGE << "Wrong matched lidar frame at " << Logging::doubleData(frame->stamp());
            continue;
        }

        // 更新位姿
        IntegrationState state = Preintegration::stateFromData(statedatalist[index], preintegration_options_);
        frame->setPose(MISC::stateToPoseWithExtrinsic(state, pose_b_l));

        // 更新世界系点云
        lidar::PointCloudCommon::pointCloudProjection(frame->pose(), frame->pointCloudLidar(),
                                                      frame->pointCloudWorld());
    }
}

bool Optimizer::marginalization(deque<double> &timelist, deque<StateData> &statedatalist,
                                deque<PreBasePtr> &preintegrationlist) {
    TimeCost timecost;

    size_t num_marg = 0;
    vector<ulong> keyframeids;
    auto frame = lidar_map_->keyframes()[1];
    num_marg   = static_cast<size_t>(getStateDataIndex(timelist, frame->stamp()));

    // 保留的时间节点
    double last_time = timelist[num_marg];

    // 边缘化信息
    std::shared_ptr<MarginalizationInfo> marginalization_info = std::make_shared<MarginalizationInfo>();

    // 指定每个参数块独立的ID, 用于索引参数
    std::unordered_map<long, long> parameters_ids;
    long parameters_id = 0;
    {
        // 边缘化参数
        for (auto &last_marginalization_parameter_block : last_marginalization_parameter_blocks_) {
            parameters_ids[reinterpret_cast<long>(last_marginalization_parameter_block)] = parameters_id++;
        }

        // 外参参数

        parameters_ids[reinterpret_cast<long>(extrinsic_b_l_)]     = parameters_id++;
        parameters_ids[reinterpret_cast<long>(extrinsic_b_l_ + 7)] = parameters_id++;

        // 位姿参数
        for (const auto &statedata : statedatalist) {
            parameters_ids[reinterpret_cast<long>(statedata.pose)] = parameters_id++;
            parameters_ids[reinterpret_cast<long>(statedata.mix)]  = parameters_id++;
        }

        // 更新参数块的特定ID, 必要的
        marginalization_info->updateParamtersIds(parameters_ids);
    }

    // 边缘化因子
    if (last_marginalization_info_ && last_marginalization_info_->isValid()) {

        vector<int> marginalized_index;
        for (size_t i = 0; i < num_marg; i++) {
            for (size_t k = 0; k < last_marginalization_parameter_blocks_.size(); k++) {
                if (last_marginalization_parameter_blocks_[k] == statedatalist[i].pose ||
                    last_marginalization_parameter_blocks_[k] == statedatalist[i].mix) {
                    marginalized_index.push_back((int) k);
                }
            }
        }

        auto factor   = std::make_shared<MarginalizationFactor>(last_marginalization_info_);
        auto residual = std::make_shared<ResidualBlockInfo>(factor, nullptr, last_marginalization_parameter_blocks_,
                                                            marginalized_index);
        marginalization_info->addResidualBlockInfo(residual);
    }

    // 预积分因子
    for (size_t k = 0; k < num_marg; k++) {
        // 由于会移除多个预积分, 会导致出现保留和移除同时出现, 判断索引以区分
        vector<int> marg_index;
        if (k == (num_marg - 1)) {
            marg_index = {0, 1};
        } else {
            marg_index = {0, 1, 2, 3};
        }

        auto factor = std::make_shared<PreintegrationFactor>(preintegrationlist[k]);
        auto residual =
            std::make_shared<ResidualBlockInfo>(factor, nullptr,
                                                vector<double *>{statedatalist[k].pose, statedatalist[k].mix,
                                                                 statedatalist[k + 1].pose, statedatalist[k + 1].mix},
                                                marg_index);
        marginalization_info->addResidualBlockInfo(residual);
    }

    // 先验约束因子
    if (is_use_prior_) {
        auto pose_factor   = std::make_shared<ImuPosePriorFactor>(pose_prior_, pose_prior_std_);
        auto pose_residual = std::make_shared<ResidualBlockInfo>(
            pose_factor, nullptr, vector<double *>{statedatalist[0].pose}, vector<int>{0});
        marginalization_info->addResidualBlockInfo(pose_residual);

        auto mix_factor   = std::make_shared<ImuMixPriorFactor>(preintegration_options_, mix_prior_, mix_prior_std_);
        auto mix_residual = std::make_shared<ResidualBlockInfo>(mix_factor, nullptr,
                                                                vector<double *>{statedatalist[0].mix}, vector<int>{0});
        marginalization_info->addResidualBlockInfo(mix_residual);

        // 雷达IMU外参先验
        if (optimize_estimate_lidar_extrinsic_) {
            auto extrinsic_lidar_factor =
                std::make_shared<ImuPosePriorFactor>(extrinsic_lidar_prior_, extrinsic_lidar_prior_std_);
            auto extrinsic_lidar_residual = std::make_shared<ResidualBlockInfo>(
                extrinsic_lidar_factor, nullptr, vector<double *>{extrinsic_b_l_}, vector<int>{});
            marginalization_info->addResidualBlockInfo(extrinsic_lidar_residual);
        }

        is_use_prior_ = false;
    }

    // 可能移除多个雷达节点
    auto cur_frame = lidar_map_->keyframes().back();
    int cur_index  = getStateDataIndex(timelist, cur_frame->stamp());

    auto loss_function = std::make_shared<ceres::HuberLoss>(1.0);
    for (size_t i = 0; i < lidar_map_->keyframes().size() - 1; i++) {
        auto ref_frame = lidar_map_->keyframes()[i];
        if (MISC::isTheSameTimeNode(ref_frame->stamp(), last_time, MISC::MINIMUM_TIME_INTERVAL) ||
            (ref_frame->stamp() > last_time)) {
            break;
        }

        // 位姿状态索引
        int ref_index = getStateDataIndex(timelist, ref_frame->stamp());

        if ((ref_index >= 0) && (cur_index >= 0)) {
            const auto &features     = ref_frame->features();
            const auto &points_lidar = cur_frame->pointCloudLidar()->points;
            for (size_t k = 0; k < features.size(); k++) {
                const auto &feature = features[k];
                // 无效特征
                if (!feature || (feature->featureType() != lidar::LidarFeature::FEATURE_PLANE) ||
                    feature->isOutlier()) {
                    continue;
                }

                auto factor = std::make_shared<LidarFactor>(
                    points_lidar[k], feature->unitNormalVector(), feature->normInverse(), feature->pointStd(),
                    ref_frame->velocity(), ref_frame->angularVelocity(), ref_frame->timeDelay(), cur_frame->velocity(),
                    cur_frame->angularVelocity(), cur_frame->timeDelay());
                auto residual = std::make_shared<ResidualBlockInfo>(
                    factor, loss_function,
                    std::vector<double *>{statedatalist[ref_index].pose, statedatalist[cur_index].pose, extrinsic_b_l_,
                                          &extrinsic_b_l_[7]},
                    std::vector<int>{0});
                marginalization_info->addResidualBlockInfo(residual);
            }
        }
    }

    // 边缘化处理
    marginalization_info->marginalization();

    // 保留的数据, 使用独立ID
    std::unordered_map<long, double *> address;
    for (size_t k = num_marg; k < statedatalist.size(); k++) {
        address[parameters_ids[reinterpret_cast<long>(statedatalist[k].pose)]] = statedatalist[k].pose;
        address[parameters_ids[reinterpret_cast<long>(statedatalist[k].mix)]]  = statedatalist[k].mix;
    }
    address[parameters_ids[reinterpret_cast<long>(extrinsic_b_l_)]]     = extrinsic_b_l_;
    address[parameters_ids[reinterpret_cast<long>(extrinsic_b_l_ + 7)]] = &extrinsic_b_l_[7];

    last_marginalization_parameter_blocks_ = marginalization_info->getParamterBlocks(address);
    last_marginalization_info_             = std::move(marginalization_info);

    // 移除边缘化的数据

    for (auto frame : lidar_map_->keyframes()) {
        if (MISC::isTheSameTimeNode(frame->stamp(), last_time, MISC::MINIMUM_TIME_INTERVAL) ||
            (frame->stamp() > last_time)) {
            break;
        }

        int index = getStateDataIndex(timelist, frame->stamp());
        if (index >= 0) {
            lidar_map_->removeOldestLidarFrame();
        }
    }

    // 预积分观测及时间状态, 放最后, 需要关键帧索引需要timelist
    for (size_t k = 0; k < num_marg; k++) {
        timelist.pop_front();
        statedatalist.pop_front();
        preintegrationlist.pop_front();
    }

    timecosts_[2] = timecost.costInMillisecond();

    LOGI << "Marginalize " << num_marg << " states, last time " << Logging::doubleData(last_time) << ", costs "
         << timecosts_[2];

    return true;
}

void Optimizer::addStateParameters(ceres::Problem &problem, deque<StateData> &statedatalist) {
    LOGI << "Total " << statedatalist.size() << " pose states from " << Logging::doubleData(statedatalist.begin()->time)
         << " to " << Logging::doubleData(statedatalist.back().time);

    for (auto &statedata : statedatalist) {
        // 位姿
        ceres::Manifold *manifold = new PoseManifold();
        problem.AddParameterBlock(statedata.pose, Preintegration::numPoseParameter(), manifold);
        problem.AddParameterBlock(statedata.mix, Preintegration::numMixParameter(preintegration_options_));
    }
}

void Optimizer::addLidarParameters(ceres::Problem &problem) {
    // 外参参数化
    ceres::Manifold *manifold = new PoseManifold();
    problem.AddParameterBlock(extrinsic_b_l_, 7, manifold);
    if (!optimize_estimate_lidar_extrinsic_ || !lidar_map_->isWindowFull()) {
        problem.SetParameterBlockConstant(extrinsic_b_l_);
    }

    // 时间偏移
    problem.AddParameterBlock(&extrinsic_b_l_[7], 1);
    if (!optimize_estimate_lidar_td_ || !lidar_map_->isWindowFull()) {
        problem.SetParameterBlockConstant(&extrinsic_b_l_[7]);
    } else {
        if (lidar_map_->keyframes().back()->velocity().norm() < MINIMUM_SPEED_ESTIMATE_LIDAR_TD) {
            problem.SetParameterBlockConstant(&extrinsic_b_l_[7]);
        }
    }
}

LidarResiduals Optimizer::addLidarFactors(ceres::Problem &problem, bool isusekernel, const deque<double> &timelist,
                                          deque<StateData> &statedatalist) {
    LidarResiduals residual_block;
    residual_block.clear();

    ceres::LossFunction *loss_function = nullptr;
    if (isusekernel) {
        loss_function = new ceres::HuberLoss(1.0);
    }

    // 关键帧选择步进
    auto cur_frame = lidar_map_->keyframes().back();
    for (size_t i = 0; i < lidar_map_->keyframes().size() - 1; i++) {
        auto ref_frame = lidar_map_->keyframes()[i];

        // Lidar系的点云
        const auto &points_lidar = cur_frame->pointCloudLidar()->points;

        // 位姿状态索引
        int ref_index = getStateDataIndex(timelist, ref_frame->stamp());
        int cur_index = getStateDataIndex(timelist, cur_frame->stamp());
        if ((ref_index < 0) || (cur_index < 0)) {
            LOGE << "Wrong matched lidar frame at " << Logging::doubleData(cur_frame->stamp());
            continue;
        }

        // 所有特征
        const auto &features = ref_frame->features();
        for (size_t k = 0; k < features.size(); k++) {
            const auto &feature = features[k];
            // 无效特征
            if (!feature || (feature->featureType() != lidar::LidarFeature::FEATURE_PLANE) || feature->isOutlier()) {
                continue;
            }

            auto factor = new LidarFactor(points_lidar[k], feature->unitNormalVector(), feature->normInverse(),
                                          feature->pointStd(), ref_frame->velocity(), ref_frame->angularVelocity(),
                                          ref_frame->timeDelay(), cur_frame->velocity(), cur_frame->angularVelocity(),
                                          cur_frame->timeDelay());
            auto residual_block_id =
                problem.AddResidualBlock(factor, loss_function, statedatalist[ref_index].pose,
                                         statedatalist[cur_index].pose, extrinsic_b_l_, &extrinsic_b_l_[7]);
            residual_block.emplace_back(residual_block_id, feature);
        }
    }

    if (is_use_prior_ && optimize_estimate_lidar_extrinsic_) {
        // 对外参添加先验
        auto pose_factor = new ImuPosePriorFactor(extrinsic_lidar_prior_, extrinsic_lidar_prior_std_);
        problem.AddResidualBlock(pose_factor, nullptr, extrinsic_b_l_);
    }

    return residual_block;
}

void Optimizer::addImuFactors(ceres::Problem &problem, deque<StateData> &statedatalist,
                              deque<PreBasePtr> &preintegrationlist) {
    for (size_t k = 0; k < preintegrationlist.size(); k++) {
        // 预积分因子
        auto factor = new PreintegrationFactor(preintegrationlist[k]);
        problem.AddResidualBlock(factor, nullptr, statedatalist[k].pose, statedatalist[k].mix,
                                 statedatalist[k + 1].pose, statedatalist[k + 1].mix);
    }

    // 添加IMU误差约束, 限制过大的误差估计
    auto factor = new ImuErrorFactor(preintegration_options_);
    problem.AddResidualBlock(factor, nullptr, statedatalist[preintegrationlist.size()].mix);

    // IMU初始先验因子, 仅限于初始化
    if (is_use_prior_) {
        auto pose_factor = new ImuPosePriorFactor(pose_prior_, pose_prior_std_);
        problem.AddResidualBlock(pose_factor, nullptr, statedatalist[0].pose);

        auto mix_factor = new ImuMixPriorFactor(preintegration_options_, mix_prior_, mix_prior_std_);
        problem.AddResidualBlock(mix_factor, nullptr, statedatalist[0].mix);
    }
}

ResidualOutliers Optimizer::lidarOutlierCullingByChi2(ceres::Problem &problem, LidarResiduals &residual_ids) {
    static double chi2_threshold = 3.841; // 0.05

    size_t residual_size = residual_ids.size();
    ResidualOutliers outlier_ids(residual_size);

    auto culling_function = [&](const tbb::blocked_range<size_t> &range) {
        for (size_t k = range.begin(); k != range.end(); k++) {
            const auto &residual_id = residual_ids[k].first;
            const auto &feature     = residual_ids[k].second;

            // 获取参数块和CostFunction
            vector<double *> parameter_blocks;
            problem.GetParameterBlocksForResidualBlock(residual_id, &parameter_blocks);
            const ceres::CostFunction *cost_function = problem.GetCostFunctionForResidualBlock(residual_id);

            // 计算残差
            double residual;
            cost_function->Evaluate(parameter_blocks.data(), &residual, nullptr);

            // 判断粗差
            bool is_outlier = false;
            double cost     = residual * residual;
            if (cost > chi2_threshold) {
                feature->setOutlier();
                is_outlier = true;
            }
            outlier_ids[k] = std::make_pair(residual_id, is_outlier);
        }
    };
    tbb::parallel_for(tbb::blocked_range<size_t>(0, residual_size), culling_function);

    return outlier_ids;
}

void Optimizer::doReintegration(deque<PreBasePtr> &preintegrationlist, deque<StateData> &statedatalist) {
    int cnt = 0;
    for (size_t k = 0; k < preintegrationlist.size(); k++) {
        IntegrationState state = Preintegration::stateFromData(statedatalist[k], preintegration_options_);
        Vector3d dbg           = preintegrationlist[k]->deltaState().bg - state.bg;
        Vector3d dba           = preintegrationlist[k]->deltaState().ba - state.ba;
        if ((dbg.norm() > 6 * integration_parameters_->gyr_bias_std) ||
            (dba.norm() > 6 * integration_parameters_->acc_bias_std)) {
            preintegrationlist[k]->reintegration(state);
            cnt++;
        }
    }
    if (cnt) {
        LOGW << "Reintegration " << cnt << " preintegration";
    }
}

int Optimizer::getStateDataIndex(const deque<double> &timelist, double time) {

    size_t index = MISC::getStateDataIndex(timelist, time, MISC::MINIMUM_TIME_INTERVAL);
    if (!MISC::isTheSameTimeNode(timelist[index], time, MISC::MINIMUM_TIME_INTERVAL)) {
        LOGW << "Wrong matching time node " << Logging::doubleData(timelist[index]) << " to "
             << Logging::doubleData(time);
        return -1;
    }
    return static_cast<int>(index);
}

void Optimizer::constructPrior(const StateData &statedata, bool is_zero_velocity) {
    // 初始先验
    double pos_prior_std  = 0.1;                                       // 0.1 m
    double att_prior_std  = 0.1 * D2R;                                 // 0.1 deg
    double vel_prior_std  = 0.1;                                       // 0.1 m/s
    double bg_prior_std   = integration_parameters_->gyr_bias_std * 3; // Bias std * 3
    double ba_prior_std   = ab_prior_std_;
    double sodo_prior_std = 0.005;     // 5000 PPM
    double avb_prior_std  = 0.5 * D2R; // 0.5 deg

    if (!is_zero_velocity) {
        att_prior_std = 0.5 * D2R; // 0.5 deg
        bg_prior_std  = gb_prior_std_;
        vel_prior_std = 0.5;
    }

    memcpy(pose_prior_, statedata.pose, sizeof(double) * 7);
    memcpy(mix_prior_, statedata.mix, sizeof(double) * 18);
    for (size_t k = 0; k < 3; k++) {
        pose_prior_std_[k + 0] = pos_prior_std;
        pose_prior_std_[k + 3] = att_prior_std;

        mix_prior_std_[k + 0] = vel_prior_std;
        mix_prior_std_[k + 3] = bg_prior_std;
        mix_prior_std_[k + 6] = ba_prior_std;
    }

    mix_prior_std_[9]  = sodo_prior_std;
    mix_prior_std_[10] = avb_prior_std;
    mix_prior_std_[11] = avb_prior_std;

    if (optimize_estimate_lidar_extrinsic_) {
        // 外参先验
        if (optimize_lidar_extrinsic_accurate_) {
            pos_prior_std = 0.05;      // 0.05 m
            att_prior_std = 0.1 * D2R; // 0.1 deg
        } else {
            pos_prior_std = 0.1;       // 0.1 m
            att_prior_std = 1.0 * D2R; // 1.0 deg
        }

        memcpy(extrinsic_lidar_prior_, extrinsic_b_l_, sizeof(double) * 7);
        for (size_t k = 0; k < 3; k++) {
            extrinsic_lidar_prior_std_[k + 0] = pos_prior_std;
            extrinsic_lidar_prior_std_[k + 3] = att_prior_std;
        }
    }

    is_use_prior_ = true;
}

void Optimizer::updateLidarExtrinsic(Pose &pose_b_l, double &td_b_l) const {
    pose_b_l.t[0] = extrinsic_b_l_[0];
    pose_b_l.t[1] = extrinsic_b_l_[1];
    pose_b_l.t[2] = extrinsic_b_l_[2];

    Quaterniond q_b_l = Quaterniond(extrinsic_b_l_[6], extrinsic_b_l_[3], extrinsic_b_l_[4], extrinsic_b_l_[5]);
    pose_b_l.R        = Rotation::quaternion2matrix(q_b_l.normalized());

    td_b_l = extrinsic_b_l_[7];
}
