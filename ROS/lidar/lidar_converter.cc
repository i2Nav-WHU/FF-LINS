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

#include "lidar_converter.h"

#include "common/gpstime.h"

#include <pcl_conversions/pcl_conversions.h>

LidarConverter::LidarConverter(int scan_line, double nearest_distance, double farthest_distance)
    : scan_line_(scan_line) {
    nearest_distance_         = nearest_distance;
    nearest_square_distance_  = nearest_distance * nearest_distance;
    farthest_square_distance_ = farthest_distance * farthest_distance;
}

size_t LidarConverter::livoxPointCloudConvertion(const livox_ros_driver::CustomMsgConstPtr &msg,
                                                 PointCloudCustomPtr &pointcloud, double &start, double &end,
                                                 bool to_gps_time) {
    pointcloud->clear();
    pointcloud->reserve(msg->points.size());

    // 使用统一的时间
    uint64_t timebase = msg->header.stamp.toNSec();

    start = DBL_MAX;
    end   = 0;
    for (size_t k = 0; k < msg->points.size(); k++) {
        const auto &raw = msg->points[k];
        // 第0回波或第1回波
        if ((raw.line < scan_line_) && ((raw.tag & 0x30) == 0x00 || (raw.tag & 0x30) == 0x10)) {
            PointTypeCustom point = livoxPointConvertion(raw, timebase, to_gps_time);

            if (point.time < start) {
                start = point.time;
            }
            if (point.time > end) {
                end = point.time;
            }

            // 有效测量距离范围, 无效点数值为0
            double square_dist = point.getVector3fMap().squaredNorm();
            if ((square_dist > nearest_square_distance_) && (square_dist < farthest_square_distance_)) {
                // 原始点
                pointcloud->push_back(point);
            }
        }
    }

    return pointcloud->size();
}

size_t LidarConverter::velodynePointCloudConvertion(const sensor_msgs::PointCloud2ConstPtr &msg,
                                                    PointCloudCustomPtr &pointcloud, double &start, double &end,
                                                    bool to_gps_time) {
    pointcloud->clear();

    pcl::PointCloud<VelodynePoint> pointcloud_raw;
    pcl::fromROSMsg(*msg, pointcloud_raw);

    // 数据包时间
    double stamp = msg->header.stamp.toSec();
    if (to_gps_time) {
        int week;
        double sow;
        GpsTime::unix2gps(stamp, week, sow);
        stamp = sow;
    }

    PointTypeCustom point;

    start = DBL_MAX;
    end   = 0;
    for (size_t k = 0; k < pointcloud_raw.size(); k++) {
        auto &raw = pointcloud_raw[k];

        point.getVector3fMap() = raw.getVector3fMap();
        point.intensity        = raw.intensity;

        point.time = stamp + raw.time;
        if (point.time < start) {
            start = point.time;
        }
        if (point.time > end) {
            end = point.time;
        }

        // 不在距离范围内
        double square_dist = point.getVector3fMap().squaredNorm();
        if ((square_dist > nearest_square_distance_) && (square_dist < farthest_square_distance_)) {
            pointcloud->push_back(point);
        }
    }

    return pointcloud->size();
}

size_t LidarConverter::ousterPointCloudConvertion(const sensor_msgs::PointCloud2ConstPtr &msg,
                                                  PointCloudCustomPtr &pointcloud, double &start, double &end,
                                                  bool to_gps_time) {
    pointcloud->clear();

    pcl::PointCloud<OusterPoint> pointcloud_raw;
    pcl::fromROSMsg(*msg, pointcloud_raw);

    // 数据包时间
    double stamp = msg->header.stamp.toSec();
    if (to_gps_time) {
        int week;
        double sow;
        GpsTime::unix2gps(stamp, week, sow);
        stamp = sow;
    }

    PointTypeCustom point;

    start = DBL_MAX;
    end   = 0;
    for (size_t k = 0; k < pointcloud_raw.size(); k++) {
        auto &raw = pointcloud_raw[k];

        point.getVector3fMap() = raw.getVector3fMap();
        point.intensity        = raw.intensity;
        point.time             = stamp + raw.t * 1.0e-9;

        if (point.time < start) {
            start = point.time;
        }
        if (point.time > end) {
            end = point.time;
        }

        // 不在距离范围内
        double square_dist = point.getVector3fMap().squaredNorm();
        if ((square_dist > nearest_square_distance_) && (square_dist < farthest_square_distance_)) {
            pointcloud->push_back(point);
        }
    }

    return pointcloud->size();
}

size_t LidarConverter::hesaiPointCloudConvertion(const sensor_msgs::PointCloud2ConstPtr &msg,
                                                 PointCloudCustomPtr &pointcloud, double &start, double &end,
                                                 bool to_gps_time) {
    pointcloud->clear();

    pcl::PointCloud<HesaiPoint> pointcloud_raw;
    pcl::fromROSMsg(*msg, pointcloud_raw);

    // 数据包时间
    double stamp = msg->header.stamp.toSec();
    if (to_gps_time) {
        int week;
        double sow;
        GpsTime::unix2gps(stamp, week, sow);
        stamp = sow;
    }

    PointTypeCustom point;

    start = DBL_MAX;
    end   = 0;
    for (size_t k = 0; k < pointcloud_raw.size(); k++) {
        auto &raw              = pointcloud_raw[k];
        point.getVector3fMap() = raw.getVector3fMap();
        point.intensity        = raw.intensity;

        // 时间转换
        if (to_gps_time) {
            int week;
            double sow;
            GpsTime::unix2gps(raw.timestamp, week, sow);
            point.time = sow;
        } else {
            point.time = raw.timestamp;
        }

        if (point.time < start) {
            start = point.time;
        }
        if (point.time > end) {
            end = point.time;
        }

        // 不在距离范围内
        double square_dist = point.getVector3fMap().squaredNorm();
        if ((square_dist > nearest_square_distance_) && (square_dist < farthest_square_distance_)) {
            pointcloud->push_back(point);
        }
    }

    return pointcloud->size();
}

PointTypeCustom LidarConverter::livoxPointConvertion(const livox_ros_driver::CustomPoint &raw, uint64_t timebase,
                                                     bool to_gps_time) {
    PointTypeCustom point;

    point.x         = raw.x;
    point.y         = raw.y;
    point.z         = raw.z;
    point.intensity = raw.reflectivity;
    point.time      = static_cast<double>(timebase + raw.offset_time) * 1.0e-9;
    if (to_gps_time) {
        int week;
        double sow;
        GpsTime::unix2gps(point.time, week, sow);
        point.time = sow;
    }

    return point;
}
