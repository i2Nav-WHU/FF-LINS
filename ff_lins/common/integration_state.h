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

#ifndef INTEGRATION_DEFINE_H
#define INTEGRATION_DEFINE_H

#include <Eigen/Geometry>
#include <vector>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using std::vector;

typedef struct IntegrationState {
    double time;

    Vector3d p{0, 0, 0};
    Quaterniond q{0, 0, 0, 0};
    Vector3d v{0, 0, 0};

    Vector3d bg{0, 0, 0};
    Vector3d ba{0, 0, 0};

    Vector3d s{0, 0, 0};
    double sodo{0};
    Vector2d avb{0, 0};

    Vector3d sg{0, 0, 0};
    Vector3d sa{0, 0, 0};
} IntegrationState;

typedef struct IntegrationStateData {
    double time;

    double pose[7]; // pose : 3 + 4 = 7

    // mix parameters
    // vel + bias : 3 + 6 = 9
    // vel + bias + sodo : 3 + 6 + 1 = 10
    // vel + bias + sodo + abv : 3 + 6 + 1 + 2 = 12
    double mix[18];
} IntegrationStateData;

typedef struct IntegrationParameters {
    // IMU噪声为白噪声, 积分为随机游走, 即VRW和ARW
    // 零偏和比例因子建模为一阶高斯马尔卡夫过程, 参数为标准差及相关时间

    double acc_vrw;      // 速度随机游走, m / s^1.5
    double gyr_arw;      // 角度随机游走, rad / s^0.5
    double gyr_bias_std; // 陀螺零偏标准差, rad / s
    double acc_bias_std; // 加表零偏标准差, m / s^2
    double corr_time;    // 相关时间, s

    double gravity; // 当地重力, m / s^2

    Vector3d odo_std; // 里程计白噪声, m/s
    double odo_srw;   // 里程计比例因子随机游走, PPM / sqrt(s)
    double odo_arw;   // 安装角随机游走, deg / sqrt(s)

    Vector3d abv;  // b系与v系的安装角, rad
    Vector3d lodo; // b系下的里程计杆臂, m

    Vector3d station; // 站心坐标系原点
    Vector3d lant; // b系下的天线杆臂, m
} IntegrationParameters;

typedef struct IntegrationConfiguration {
    bool isuseodo;
    bool iswithearth;
    bool isodoext;

    Vector3d origin; // 站心原点
    Vector3d gravity;
    Vector3d iewn;
} IntegrationConfiguration;

#endif // INTEGRATION_DEFINE_H
