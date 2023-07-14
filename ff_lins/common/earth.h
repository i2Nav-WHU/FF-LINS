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

#ifndef EARTH_H
#define EARTH_H

#include "common/types.h"

#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

class Earth {

public:
    static double gravity(const Vector3d &blh);

    static Eigen::Vector2d meridianPrimeVerticalRadius(double lat);
    static double RN(double lat);

    static Matrix3d cne(const Vector3d &blh);
    static Quaterniond qne(const Vector3d &blh);
    static Vector3d blh(const Quaterniond &qne, double height);

    static Vector3d blh2ecef(const Vector3d &blh);
    static Vector3d ecef2blh(const Vector3d &ecef);

    static Matrix3d DRi(const Vector3d &blh);
    static Matrix3d DR(const Vector3d &blh);

    static Vector3d local2global(const Vector3d &origin, const Vector3d &local);
    static Vector3d global2local(const Vector3d &origin, const Vector3d &global);
    static Pose local2global(const Vector3d &origin, const Pose &local);
    static Pose global2local(const Vector3d &origin, const Pose &global);

    static Vector3d iewe();
    static Vector3d iewn(double lat);
    static Vector3d iewn(const Vector3d &origin, const Vector3d &local);
    static Vector3d enwn(const Eigen::Vector2d &rmn, const Vector3d &blh, const Vector3d &vel);
    static Vector3d enwn(const Vector3d &origin, const Vector3d &local, const Vector3d &vel);

private:
    static constexpr double WGS84_WIE = 7.2921151467E-5;       //地球自转角速度
    static constexpr double WGS84_F   = 0.0033528106647474805; //扁率
    static constexpr double WGS84_RA  = 6378137.0000000000;    //长半轴a
    static constexpr double WGS84_RB  = 6356752.3142451793;    //短半轴b
    static constexpr double WGS84_GM0 = 398600441800000.00;    //地球引力常数
    static constexpr double WGS84_E1  = 0.0066943799901413156; //第一偏心率平方
    static constexpr double WGS84_E2  = 0.0067394967422764341; //第二偏心率平方
};

#endif // EARTH_H
