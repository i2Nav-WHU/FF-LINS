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

#ifndef MARGINILAZATION_INFO_H
#define MARGINILAZATION_INFO_H

#include "factors/residual_block_info.h"

#include <memory>
#include <unordered_map>

class MarginalizationInfo {

public:
    MarginalizationInfo() = default;

    ~MarginalizationInfo() {
        for (auto &block : parameter_block_data_)
            delete[] block.second;
    }

    bool isValid() const {
        return isvalid_;
    }

    static int localSize(int size) {
        return size == POSE_GLOBAL_SIZE ? POSE_LOCAL_SIZE : size;
    }

    static int globalSize(int size) {
        return size == POSE_LOCAL_SIZE ? POSE_GLOBAL_SIZE : size;
    }

    void addResidualBlockInfo(const std::shared_ptr<ResidualBlockInfo> &blockinfo);

    void updateParamtersIds(const std::unordered_map<long, long> &parameters_ids) {
        parameters_ids_ = parameters_ids;
    }

    bool marginalization();

    std::vector<double *> getParamterBlocks(std::unordered_map<long, double *> &address);

    const Eigen::MatrixXd &linearizedJacobians() {
        return linearized_jacobians_;
    }

    const Eigen::VectorXd &linearizedResiduals() {
        return linearized_residuals_;
    }

    int marginalizedSize() const {
        return marginalized_size_;
    }

    int remainedSize() const {
        return remained_size_;
    }

    const std::vector<int> &remainedBlockSize() {
        return remained_block_size_;
    }

    const std::vector<int> &remainedBlockIndex() {
        return remained_block_index_;
    }

    const std::vector<double *> &remainedBlockData() {
        return remained_block_data_;
    }

private:
    // 线性化
    void linearization();

    // Schur消元, 求解 Hp * dx_r = bp
    void schurElimination();

    // 构造增量方程 H * dx = b, 计算 H 和 b
    void constructEquation();
    void constructSingleEquation(const std::shared_ptr<ResidualBlockInfo> &factor, Eigen::MatrixXd &H0,
                                 Eigen::VectorXd &b0);

    bool updateParameterBlocksIndex();

    // 边缘化预处理, 评估每个残差块, 拷贝参数
    void preMarginalization();

    void releaseMemory() {
        // 释放因子所占有的内存, 尤其是边缘化因子及其占有的边缘化信息数据结构
        factors_.clear();
    }

private:
    // 增量线性方程参数
    Eigen::MatrixXd H0_, Hp_;
    Eigen::VectorXd b0_, bp_;

    // 以内存地址为key的无序表, 其值为参数块的键
    std::unordered_map<long, long> parameters_ids_;

    // 存放参数块的global size
    std::unordered_map<long, int> parameter_block_size_;
    // 存放参数块索引, 待边缘化参数索引在前, 保留参数索引在后, 用于构造边缘化 H * dx = b
    std::unordered_map<long, int> parameter_block_index_;
    // 存放参数块数据指针
    std::unordered_map<long, double *> parameter_block_data_;

    // 保留的参数
    std::vector<int> remained_block_size_;  // global size
    std::vector<int> remained_block_index_; // local size
    std::vector<double *> remained_block_data_;

    // local size in total
    int marginalized_size_{0};
    int remained_size_{0};
    int local_size_{0};

    // 边缘化参数相关的残差块
    std::vector<std::shared_ptr<ResidualBlockInfo>> factors_;

    const double EPS = 1e-8;

    // 边缘化求解的残差和雅克比
    Eigen::MatrixXd linearized_jacobians_;
    Eigen::VectorXd linearized_residuals_;

    // 若无待边缘化参数, 则无效
    bool isvalid_{true};
};

#endif // MARGINILAZATION_INFO_H
