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

#include "factors/marginalization_info.h"

#include <tbb/tbb.h>

void MarginalizationInfo::addResidualBlockInfo(const std::shared_ptr<ResidualBlockInfo> &blockinfo) {
    factors_.push_back(blockinfo);

    const auto &parameter_blocks = blockinfo->parameterBlocks();
    const auto &block_sizes      = blockinfo->parameterBlockSizes();

    for (size_t k = 0; k < parameter_blocks.size(); k++) {
        parameter_block_size_[parameters_ids_[reinterpret_cast<long>(parameter_blocks[k])]] = block_sizes[k];
    }

    // 被边缘化的参数, 先加入表中以进行后续的排序
    for (int index : blockinfo->marginalizationParametersIndex()) {
        parameter_block_index_[parameters_ids_[reinterpret_cast<long>(parameter_blocks[index])]] = 0;
    }
}

bool MarginalizationInfo::marginalization() {

    // 对边缘化的参数和保留的参数按照local size分配索引, 边缘化参数位于前端
    if (!updateParameterBlocksIndex()) {
        isvalid_ = false;

        // 释放内存
        releaseMemory();

        return false;
    }

    // 计算每个残差块参数, 进行参数内存拷贝
    preMarginalization();

    // 构造增量线性方程
    constructEquation();

    // Schur消元
    schurElimination();

    // 求解线性化雅克比和残差
    linearization();

    // 释放内存
    releaseMemory();

    return true;
}

std::vector<double *> MarginalizationInfo::getParamterBlocks(std::unordered_map<long, double *> &address) {
    std::vector<double *> remained_block_addr;

    remained_block_data_.clear();
    remained_block_index_.clear();
    remained_block_size_.clear();

    for (const auto &block : parameter_block_index_) {
        // 保留的参数
        if (block.second >= marginalized_size_) {
            remained_block_data_.push_back(parameter_block_data_[block.first]);
            remained_block_size_.push_back(parameter_block_size_[block.first]);
            remained_block_index_.push_back(parameter_block_index_[block.first]);
            remained_block_addr.push_back(address[block.first]);
        }
    }

    return remained_block_addr;
}

void MarginalizationInfo::linearization() {
    // SVD分解求解雅克比, Hp = J^T * J = V * S^{1/2} * S^{1/2} * V^T
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(Hp_);
    Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > EPS).select(saes2.eigenvalues().array(), 0));
    Eigen::VectorXd S_inv =
        Eigen::VectorXd((saes2.eigenvalues().array() > EPS).select(saes2.eigenvalues().array().inverse(), 0));

    Eigen::VectorXd S_sqrt     = S.cwiseSqrt();
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

    // J0 = S^{1/2} * V^T
    linearized_jacobians_ = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
    // e0 = -{J0^T}^{-1} * bp = - S^{-1/2} * V^T * bp
    linearized_residuals_ = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * -bp_;
}

void MarginalizationInfo::schurElimination() {
    // H0 * dx = b0
    Eigen::MatrixXd Hmm = 0.5 * (H0_.block(0, 0, marginalized_size_, marginalized_size_) +
                                 H0_.block(0, 0, marginalized_size_, marginalized_size_).transpose());
    Eigen::MatrixXd Hmr = H0_.block(0, marginalized_size_, marginalized_size_, remained_size_);
    Eigen::MatrixXd Hrm = H0_.block(marginalized_size_, 0, remained_size_, marginalized_size_);
    Eigen::MatrixXd Hrr = H0_.block(marginalized_size_, marginalized_size_, remained_size_, remained_size_);
    Eigen::VectorXd bmm = b0_.segment(0, marginalized_size_);
    Eigen::VectorXd brr = b0_.segment(marginalized_size_, remained_size_);

    // SVD分解Amm求逆
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Hmm);
    Eigen::MatrixXd Hmm_inv =
        saes.eigenvectors() *
        Eigen::VectorXd((saes.eigenvalues().array() > EPS).select(saes.eigenvalues().array().inverse(), 0))
            .asDiagonal() *
        saes.eigenvectors().transpose();

    // Hp = Hrr - Hrm * Hmm^-1 * Hmr
    Hp_ = Hrr - Hrm * Hmm_inv * Hmr;
    // bp = br - Hrm * Hmm^-1 * bm
    bp_ = brr - Hrm * Hmm_inv * bmm;
}

void MarginalizationInfo::constructEquation() {
    H0_ = Eigen::MatrixXd::Zero(local_size_, local_size_);
    b0_ = Eigen::VectorXd::Zero(local_size_);

    int num_threads = tbb::info::default_concurrency();
    if (factors_.size() < num_threads) {
        for (const auto &factor : factors_) {
            constructSingleEquation(factor, H0_, b0_);
        }
    } else {
        // 分配内存空间
        tbb::concurrent_vector<Eigen::MatrixXd> H0_vec(num_threads);
        tbb::concurrent_vector<Eigen::VectorXd> b0_vec(num_threads);
        tbb::concurrent_vector<std::pair<size_t, size_t>> factors_index;

        // 初始化
        size_t thread_size     = static_cast<size_t>(num_threads);
        size_t avg_num_factors = factors_.size() / thread_size;
        for (size_t k = 0; k < thread_size; k++) {
            H0_vec[k].setZero(local_size_, local_size_);
            b0_vec[k].setZero(local_size_);

            factors_index.emplace_back(avg_num_factors * k, avg_num_factors * (k + 1));
        }
        factors_index.back().second = factors_.size();

        // 多线程处理
        auto construction_function = [&](const tbb::blocked_range<size_t> &range) {
            for (size_t k = range.begin(); k != range.end(); k++) {
                size_t start_index = factors_index[k].first;
                size_t end_idnex   = factors_index[k].second;

                for (size_t f_in = start_index; f_in < end_idnex; f_in++) {
                    const auto &factor = factors_[f_in];
                    constructSingleEquation(factor, H0_vec[k], b0_vec[k]);
                }
            }
        };
        tbb::parallel_for(tbb::blocked_range<size_t>(0, thread_size), construction_function);

        // 合并处理结果
        for (size_t k = 0; k < thread_size; k++) {
            H0_ += H0_vec[k];
            b0_ += b0_vec[k];
        }
    }
}

void MarginalizationInfo::constructSingleEquation(const std::shared_ptr<ResidualBlockInfo> &factor, Eigen::MatrixXd &H0,
                                                  Eigen::VectorXd &b0) {
    for (size_t i = 0; i < factor->parameterBlocks().size(); i++) {
        int row0 = parameter_block_index_[parameters_ids_[reinterpret_cast<long>(factor->parameterBlocks()[i])]];
        int rows = parameter_block_size_[parameters_ids_[reinterpret_cast<long>(factor->parameterBlocks()[i])]];
        rows     = localSize(rows);

        Eigen::MatrixXd jacobian_i = factor->jacobians()[i].leftCols(rows);
        for (size_t j = i; j < factor->parameterBlocks().size(); ++j) {
            int col0 = parameter_block_index_[parameters_ids_[reinterpret_cast<long>(factor->parameterBlocks()[j])]];
            int cols = parameter_block_size_[parameters_ids_[reinterpret_cast<long>(factor->parameterBlocks()[j])]];
            cols     = localSize(cols);

            Eigen::MatrixXd jacobian_j = factor->jacobians()[j].leftCols(cols);

            // H = J^T * J
            Eigen::MatrixXd JTJ = jacobian_i.transpose() * jacobian_j;
            if (i == j) {
                // Hmm, Hrr
                H0.block(row0, col0, rows, cols) += JTJ;
            } else {
                // Hmr, Hrm = Hmr^T
                H0.block(row0, col0, rows, cols) += JTJ;
                H0.block(col0, row0, cols, rows) = H0.block(row0, col0, rows, cols).transpose();
            }
        }
        // b = - J^T * e
        b0.segment(row0, rows) -= jacobian_i.transpose() * factor->residuals();
    }
}

bool MarginalizationInfo::updateParameterBlocksIndex() {
    int index = 0;
    // 只有被边缘化的参数预先加入了表
    for (auto &block : parameter_block_index_) {
        block.second = index;
        index += localSize(parameter_block_size_[block.first]);
    }
    marginalized_size_ = index;

    // 加入保留的参数, 分配索引
    for (const auto &block : parameter_block_size_) {
        if (parameter_block_index_.find(block.first) == parameter_block_index_.end()) {
            parameter_block_index_[block.first] = index;
            index += localSize(block.second);
        }
    }
    remained_size_ = index - marginalized_size_;

    local_size_ = index;

    return marginalized_size_ > 0;
}

void MarginalizationInfo::preMarginalization() {
    // 并行计算
    auto preparation_function = [&](const tbb::blocked_range<size_t> &range) {
        for (size_t k = range.begin(); k != range.end(); k++) {
            const auto &factor = factors_[k];
            factor->Evaluate();
        }
    };
    tbb::parallel_for(tbb::blocked_range<size_t>(0, factors_.size()), preparation_function);

    // 涉及修改 parameter_block_data_
    for (const auto &factor : factors_) {
        std::vector<int> block_sizes = factor->parameterBlockSizes();
        for (size_t k = 0; k < block_sizes.size(); k++) {
            long id  = parameters_ids_[reinterpret_cast<long>(factor->parameterBlocks()[k])];
            int size = block_sizes[k];

            // 拷贝参数块数据
            if (parameter_block_data_.find(id) == parameter_block_data_.end()) {
                auto *data = new double[size];
                memcpy(data, factor->parameterBlocks()[k], sizeof(double) * size);
                parameter_block_data_[id] = data;
            }
        }
    }
}
