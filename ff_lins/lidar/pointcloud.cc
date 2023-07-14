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

#include "lidar/pointcloud.h"

#include <Eigen/Eigenvalues>
#include <tbb/tbb.h>

namespace lidar {

PointType PointCloudCommon::relativeProjection(const Pose &pose0, const Pose &pose1, const PointType &point_l1) {
    Pose pose_l0_l1;
    pose_l0_l1.R = pose0.R.transpose() * pose1.R;
    pose_l0_l1.t = pose0.R.transpose() * (pose1.t - pose0.t);

    return pointProjection(pose_l0_l1, point_l1);
}

void PointCloudCommon::pointCloudProjection(const Pose &pose, PointCloudPtr src, PointCloudPtr dst) {
    size_t points_size = src->size();
    dst->resize(points_size);

    auto projection_function = [&](const tbb::blocked_range<size_t> &range) {
        for (size_t k = range.begin(); k != range.end(); k++) {
            auto &point = src->points[k];

            dst->points[k] = pointProjection(pose, point);
        }
    };

    tbb::parallel_for(tbb::blocked_range<size_t>(0, points_size), projection_function);
}

PointType PointCloudCommon::pointProjection(const Pose &pose, const PointType &point) {
    PointType projected = point;

    projected.getVector3fMap() = (pose.R * point.getVector3fMap().cast<double>() + pose.t).cast<float>();

    return projected;
}

bool PointCloudCommon::planeEstimation(const PointVector &points, double threshold, Vector3d &unit_normal_vector,
                                       double &norm_inverse, std::vector<double> &distance) {
    if (points.size() < PLANE_ESTIMATION_POINT_SZIE) {
        return false;
    }

    // 构建超定方程求解法向量 A * n = b
    Eigen::Matrix<double, PLANE_ESTIMATION_POINT_SZIE, 3> A;
    Eigen::Matrix<double, PLANE_ESTIMATION_POINT_SZIE, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0;

    for (int k = 0; k < PLANE_ESTIMATION_POINT_SZIE; k++) {
        A.row(k) = points[k].getVector3fMap().cast<double>();
    }
    Vector3d normal_vector = A.colPivHouseholderQr().solve(b);

    // 法向量
    norm_inverse       = 1.0 / normal_vector.norm();
    unit_normal_vector = normal_vector * norm_inverse;

    // 校验点到平面距离
    distance.clear();
    for (int k = 0; k < PLANE_ESTIMATION_POINT_SZIE; k++) {
        auto dis = unit_normal_vector.dot(points[k].getVector3fMap().cast<double>()) + norm_inverse;
        distance.push_back(dis);
        if (fabs(dis) > threshold) {
            return false;
        }
    }
    return true;
}

double PointCloudCommon::pointSquareDistance(const PointType &point0, const PointType &point1) {
    return (point0.getVector3fMap() - point1.getVector3fMap()).squaredNorm();
}

} // namespace lidar
