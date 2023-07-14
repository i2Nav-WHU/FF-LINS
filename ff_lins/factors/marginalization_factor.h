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

#ifndef MARGINALIZATION_FACTOR_H
#define MARGINALIZATION_FACTOR_H

#include "factors/marginalization_info.h"

#include <ceres/ceres.h>
#include <memory>

class MarginalizationFactor : public ceres::CostFunction {

public:
    MarginalizationFactor() = delete;
    explicit MarginalizationFactor(std::shared_ptr<MarginalizationInfo> marg_info);

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override;

private:
    std::shared_ptr<MarginalizationInfo> marg_info_;
};

#endif // MARGINALIZATION_FACTOR_H
