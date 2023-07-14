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

#ifndef LIDAR_POINTCLOUD_H
#define LIDAR_POINTCLOUD_H

#include "common/types.h"
#include "lidar/lidar.h"

namespace lidar {

class PointCloudCommon {

public:
    // 点云相对投影, 原始点云在l1系
    static PointType relativeProjection(const Pose &pose0, const Pose &pose1, const PointType &point_l1);

    static void pointCloudProjection(const Pose &pose, PointCloudPtr src, PointCloudPtr dst);

    static bool planeEstimation(const PointVector &points, double threshold, Vector3d &unit_normal_vector,
                                double &norm_inverse, std::vector<double> &distance);

    // 点云投影
    static PointType pointProjection(const Pose &pose, const PointType &point);

    static double pointSquareDistance(const PointType &point0, const PointType &point1);

public:
    // 平面拟合的点数
    static constexpr int PLANE_ESTIMATION_POINT_SZIE = 5;
};

} // namespace lidar

#endif // LIDAR_POINTCLOUD_H
