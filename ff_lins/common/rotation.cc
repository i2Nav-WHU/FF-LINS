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

#include "common/rotation.h"

Quaterniond Rotation::matrix2quaternion(const Matrix3d &matrix) {
    return Quaterniond(matrix);
}

Matrix3d Rotation::quaternion2matrix(const Quaterniond &quaternion) {
    return quaternion.toRotationMatrix();
}

Vector3d Rotation::matrix2euler(const Eigen::Matrix3d &dcm) {
    Vector3d euler;

    euler[1] = atan(-dcm(2, 0) / sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));

    if (dcm(2, 0) <= -0.999) {
        euler[0] = atan2(dcm(2, 1), dcm(2, 2));
        euler[2] = atan2((dcm(1, 2) - dcm(0, 1)), (dcm(0, 2) + dcm(1, 1)));
    } else if (dcm(2, 0) >= 0.999) {
        euler[0] = atan2(dcm(2, 1), dcm(2, 2));
        euler[2] = M_PI + atan2((dcm(1, 2) + dcm(0, 1)), (dcm(0, 2) - dcm(1, 1)));
    } else {
        euler[0] = atan2(dcm(2, 1), dcm(2, 2));
        euler[2] = atan2(dcm(1, 0), dcm(0, 0));
    }

    // heading 0~2PI
    if (euler[2] < 0) {
        euler[2] = M_PI * 2 + euler[2];
    }

    return euler;
}

Vector3d Rotation::quaternion2euler(const Quaterniond &quaternion) {
    return matrix2euler(quaternion.toRotationMatrix());
}

Quaterniond Rotation::rotvec2quaternion(const Vector3d &rotvec) {
    double angle = rotvec.norm();
    Vector3d vec = rotvec.normalized();
    return Quaterniond(Eigen::AngleAxisd(angle, vec));
}

Vector3d Rotation::quaternion2vector(const Quaterniond &quaternion) {
    Eigen::AngleAxisd axisd(quaternion);
    return axisd.angle() * axisd.axis();
}

Matrix3d Rotation::euler2matrix(const Vector3d &euler) {
    return Matrix3d(Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) * Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                    Eigen::AngleAxisd(euler[0], Vector3d::UnitX()));
}

Quaterniond Rotation::euler2quaternion(const Vector3d &euler) {
    return Quaterniond(Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) * Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                       Eigen::AngleAxisd(euler[0], Vector3d::UnitX()));
}

Matrix3d Rotation::skewSymmetric(const Vector3d &vector) {
    Matrix3d mat;
    mat << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1), vector(0), 0;
    return mat;
}

Eigen::Matrix4d Rotation::quaternionleft(const Quaterniond &q) {
    Eigen::Matrix4d ans;
    ans(0, 0)             = q.w();
    ans.block<1, 3>(0, 1) = -q.vec().transpose();
    ans.block<3, 1>(1, 0) = q.vec();
    ans.block<3, 3>(1, 1) = q.w() * Eigen::Matrix3d::Identity() + skewSymmetric(q.vec());
    return ans;
}

Eigen::Matrix4d Rotation::quaternionright(const Quaterniond &p) {
    Eigen::Matrix4d ans;
    ans(0, 0)             = p.w();
    ans.block<1, 3>(0, 1) = -p.vec().transpose();
    ans.block<3, 1>(1, 0) = p.vec();
    ans.block<3, 3>(1, 1) = p.w() * Eigen::Matrix3d::Identity() - skewSymmetric(p.vec());
    return ans;
}