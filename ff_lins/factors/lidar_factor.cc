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

#include "factors/lidar_factor.h"

#include "common/rotation.h"

LidarFactor::LidarFactor(const PointType &point_lidar, Vector3d unit_normal_vector, double norm_inverse, double std,
                         Vector3d vel0, Vector3d omega0, double td0, Vector3d vel1, Vector3d omega1, double td1)
    : unit_normal_vector_(std::move(unit_normal_vector))
    , norm_inverse_(norm_inverse)
    , vel0_(std::move(vel0))
    , vel1_(std::move(vel1))
    , omega0_(std::move(omega0))
    , omega1_(std::move(omega1))
    , td0_(td0)
    , td1_(td1) {
    point_lidar_ = point_lidar.getVector3fMap().cast<double>();
    sqrt_info_   = 1.0 / std;
}

bool LidarFactor::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const {
    // Pose_world_imu0
    Vector3d t_w_b0(parameters[0][0], parameters[0][1], parameters[0][2]);
    Quaterniond q_w_b0(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    // Pose_world_imu1
    Vector3d t_w_b1(parameters[1][0], parameters[1][1], parameters[1][2]);
    Quaterniond q_w_b1(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    // Pose_imu_lidar
    Vector3d t_b_l(parameters[2][0], parameters[2][1], parameters[2][2]);
    Quaterniond q_b_l(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
    // td_imu_lidar
    double td = parameters[3][0];

    // 残差

    // Pose_world_imu0
    double dt0        = td - td0_;
    Matrix3d R_w_b0   = q_w_b0.toRotationMatrix();
    Matrix3d R_b0_bb0 = Matrix3d::Identity() + Rotation::skewSymmetric(omega0_ * dt0);
    Matrix3d R_w_bb0  = R_w_b0 * R_b0_bb0;
    Vector3d t_w_bb0  = t_w_b0 + vel0_ * dt0;
    // Pose_world_imu1
    double dt1        = td - td1_;
    Matrix3d R_w_b1   = q_w_b1.toRotationMatrix();
    Matrix3d R_b1_bb1 = Matrix3d::Identity() + Rotation::skewSymmetric(omega1_ * dt1);
    Matrix3d R_w_bb1  = R_w_b1 * R_b1_bb1;
    Vector3d t_w_bb1  = t_w_b1 + vel1_ * dt1;

    Matrix3d R_b_l = q_b_l.toRotationMatrix();
    Vector3d pb1   = R_b_l * point_lidar_ + t_b_l;
    Vector3d pw    = R_w_bb1 * pb1 + t_w_bb1;
    Vector3d pb0   = R_w_bb0.transpose() * (pw - t_w_bb0);
    Vector3d pl0   = R_b_l.transpose() * (pb0 - t_b_l);

    residuals[0] = sqrt_info_ * (unit_normal_vector_.transpose() * pl0 + norm_inverse_);

    if (jacobians) {
        Eigen::Matrix<double, 1, 3> sqrt_common = sqrt_info_ * unit_normal_vector_.transpose() * R_b_l.transpose();

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jaco(jacobians[0]);
            jaco.setZero();

            jaco.block<1, 3>(0, 0) = -sqrt_common * R_w_bb0.transpose();
            jaco.block<1, 3>(0, 3) =
                sqrt_common * R_b0_bb0.transpose() * Rotation::skewSymmetric(R_w_b0.transpose() * (pw - t_w_bb0));
        }

        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jaco(jacobians[1]);
            jaco.setZero();

            jaco.block<1, 3>(0, 0) = sqrt_common * R_w_bb0.transpose();
            jaco.block<1, 3>(0, 3) =
                -sqrt_common * R_w_bb0.transpose() * R_w_b1 * Rotation::skewSymmetric(R_b1_bb1 * pb1);
        }

        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jaco(jacobians[2]);
            jaco.setZero();

            Matrix3d common                              = R_b_l.transpose() * R_w_bb0.transpose() * R_w_bb1;
            Eigen::Matrix<double, 1, 3> sqrt_unit_common = sqrt_info_ * unit_normal_vector_.transpose();

            jaco.block<1, 3>(0, 0) = sqrt_unit_common * (common - R_b_l.transpose());
            jaco.block<1, 3>(0, 3) = sqrt_unit_common * (Rotation::skewSymmetric(R_b_l.transpose() * (pb0 - t_b_l)) -
                                                         common * R_b_l * Rotation::skewSymmetric(point_lidar_));
        }

        if (jacobians[3]) {
            jacobians[3][0] =
                sqrt_common * ((R_w_b0 * Rotation::skewSymmetric(omega0_)).transpose() * (pw - t_w_bb0) +
                               R_w_bb0.transpose() * (R_w_b1 * Rotation::skewSymmetric(omega1_) * pb1 + vel1_ - vel0_));
        }
    }

    return true;
}
