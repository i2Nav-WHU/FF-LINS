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

#ifndef RESIDUAL_BLOCK_INFO_H
#define RESIDUAL_BLOCK_INFO_H

#include <ceres/ceres.h>
#include <memory>

#define POSE_LOCAL_SIZE 6
#define POSE_GLOBAL_SIZE 7

class ResidualBlockInfo {

public:
    ResidualBlockInfo(std::shared_ptr<ceres::CostFunction> cost_function,
                      std::shared_ptr<ceres::LossFunction> loss_function, std::vector<double *> parameter_blocks,
                      std::vector<int> marg_para_index);

    void Evaluate();

    const std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> &jacobians() {
        return jacobians_;
    }

    const std::vector<int> &parameterBlockSizes() {
        return cost_function_->parameter_block_sizes();
    }

    const std::vector<double *> &parameterBlocks() {
        return parameter_blocks_;
    }

    const Eigen::VectorXd &residuals() {
        return residuals_;
    }

    const std::vector<int> &marginalizationParametersIndex() {
        return marg_para_index_;
    }

private:
    std::shared_ptr<ceres::CostFunction> cost_function_;
    std::shared_ptr<ceres::LossFunction> loss_function_;

    std::vector<double *> parameter_blocks_;

    std::vector<int> marg_para_index_;

    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians_;
    Eigen::VectorXd residuals_;
};

#endif // RESIDUAL_BLOCK_INFO_H
