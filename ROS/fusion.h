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

#ifndef FUSION_H
#define FUSION_H

#include "lidar/lidar_converter.h"

#include "common/types.h"
#include "ff_lins/ff_lins.h"

#include <livox_ros_driver/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include <memory>

extern std::atomic<bool> global_finished;

class Fusion {

public:
    Fusion() = default;

    void run();

    void setFinished();

private:
    void imuCallback(const sensor_msgs::ImuConstPtr &imumsg);
    void livoxCallback(const livox_ros_driver::CustomMsgConstPtr &lidarmsg);
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &lidarmsg);

    void addImuData(const IMU &imu);

    void processSubscribe(const string &imu_topic, const string &lidar_topic, ros::NodeHandle &nh);
    void processRead(const string &imu_topic, const string &lidar_topic, const string &bagfile);

private:
    std::shared_ptr<LINS> lins_;
    bool is_read_bag_{false};

    IMU imu_{.time = 0}, imu_pre_{.time = 0};
    double imu_data_dt_{0.005};
    std::queue<IMU> imu_buffer_;

    LidarConverter::Ptr lidar_converter_;
    int lidar_type_ = Livox;
};

#endif // FUSION_ROS_H
