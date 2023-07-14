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

#ifndef LIDAR_FACTOR_H
#define LIDAR_FACTOR_H

#include "common/types.h"
#include "lidar/lidar.h"

#include <ceres/ceres.h>

class LidarFactor : public ceres::SizedCostFunction<1, 7, 7, 7, 1> {

public:
    LidarFactor(const PointType &point_lidar, Vector3d unit_normal_vector, double norm_inverse, double std,
                Vector3d vel0, Vector3d omega0, double td0, Vector3d vel1, Vector3d omega1, double td1);

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override;

private:
    Vector3d unit_normal_vector_;
    double norm_inverse_;

    Vector3d point_lidar_;
    double sqrt_info_;

    Vector3d vel0_, vel1_;
    Vector3d omega0_, omega1_;
    double td0_, td1_;
};

#endif // LIDAR_FACTOR_H
