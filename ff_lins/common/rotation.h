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

#ifndef ROTATION_H
#define ROTATION_H

#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

class Rotation {

public:
    static Quaterniond matrix2quaternion(const Matrix3d &matrix);
    static Matrix3d quaternion2matrix(const Quaterniond &quaternion);

    // ZYX旋转顺序, 前右下的IMU, 输出RPY
    static Vector3d matrix2euler(const Eigen::Matrix3d &dcm);
    static Vector3d quaternion2euler(const Quaterniond &quaternion);

    static Quaterniond rotvec2quaternion(const Vector3d &rotvec);
    static Vector3d quaternion2vector(const Quaterniond &quaternion);

    // RPY --> C_b^n, 旋转不可交换, ZYX顺序
    static Matrix3d euler2matrix(const Vector3d &euler);
    static Quaterniond euler2quaternion(const Vector3d &euler);

    // 反对称矩阵
    static Matrix3d skewSymmetric(const Vector3d &vector);

    static Eigen::Matrix4d quaternionleft(const Quaterniond &q);
    static Eigen::Matrix4d quaternionright(const Quaterniond &p);
};

#endif // ROTATION_H
