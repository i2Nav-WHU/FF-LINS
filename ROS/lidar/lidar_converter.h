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

#ifndef LIDAR_CONVERTER_H
#define LIDAR_CONVERTER_H

#include "lidar/lidar.h"

#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>
#include <sensor_msgs/PointCloud2.h>

using std::vector;

class LidarConverter {

public:
    typedef std::shared_ptr<LidarConverter> Ptr;

    LidarConverter() = delete;
    LidarConverter(int scan_line, double nearest_distance, double farthest_distance);

    size_t livoxPointCloudConvertion(const livox_ros_driver::CustomMsgConstPtr &msg, PointCloudCustomPtr &pointcloud,
                                     double &start, double &end, bool to_gps_time);
    size_t velodynePointCloudConvertion(const sensor_msgs::PointCloud2ConstPtr &msg, PointCloudCustomPtr &pointcloud,
                                        double &start, double &end, bool to_gps_time);

    size_t ousterPointCloudConvertion(const sensor_msgs::PointCloud2ConstPtr &msg, PointCloudCustomPtr &pointcloud,
                                      double &start, double &end, bool to_gps_time);

    size_t hesaiPointCloudConvertion(const sensor_msgs::PointCloud2ConstPtr &msg, PointCloudCustomPtr &pointcloud,
                                     double &start, double &end, bool to_gps_time);

private:
    static PointTypeCustom livoxPointConvertion(const livox_ros_driver::CustomPoint &point, uint64_t timebase,
                                                bool to_gps_time);

private:
    double nearest_distance_{1.0};
    double nearest_square_distance_{1.0}, farthest_square_distance_{10000};
    int scan_line_{6};
};

#endif // LIDAR_CONVERTER_H
