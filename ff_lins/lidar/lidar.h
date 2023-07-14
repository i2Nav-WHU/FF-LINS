/*
 * FF-LINS: A Frame-to-Frame Solid-State-LiDAR-Inertial State Estimator
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

#ifndef LIDAR_LIDAR_H
#define LIDAR_LIDAR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Custom point type for undistortion
struct PointXYZIT {
    PCL_ADD_POINT4D;                // quad-word XYZ
    float intensity;                // point intensity
    double time;                    // point time
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT, (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                                      intensity)(double, time, time))

enum LidarType {
    Livox = 1,
};

typedef PointXYZIT PointTypeCustom;
typedef pcl::PointCloud<PointTypeCustom> PointCloudCustom;
typedef pcl::PointCloud<PointTypeCustom>::Ptr PointCloudCustomPtr;

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;

#endif // LIDAR_LIDAR_H