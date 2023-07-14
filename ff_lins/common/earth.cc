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

#include "common/earth.h"

double Earth::gravity(const Vector3d &blh) {

    double sin2 = sin(blh[0]);
    sin2 *= sin2;

    return 9.7803267715 * (1 + 0.0052790414 * sin2 + 0.0000232718 * sin2 * sin2) +
           blh[2] * (0.0000000043977311 * sin2 - 0.0000030876910891) + 0.0000000000007211 * blh[2] * blh[2];
}

Eigen::Vector2d Earth::meridianPrimeVerticalRadius(double lat) {
    double tmp, sqrttmp;

    tmp = sin(lat);
    tmp *= tmp;
    tmp     = 1 - WGS84_E1 * tmp;
    sqrttmp = sqrt(tmp);

    return {WGS84_RA * (1 - WGS84_E1) / (sqrttmp * tmp), WGS84_RA / sqrttmp};
}

double Earth::RN(double lat) {
    double sinlat = sin(lat);
    return WGS84_RA / sqrt(1.0 - WGS84_E1 * sinlat * sinlat);
}

Matrix3d Earth::cne(const Vector3d &blh) {
    double coslon, sinlon, coslat, sinlat;

    sinlat = sin(blh[0]);
    sinlon = sin(blh[1]);
    coslat = cos(blh[0]);
    coslon = cos(blh[1]);

    Matrix3d dcm;
    dcm(0, 0) = -sinlat * coslon;
    dcm(0, 1) = -sinlon;
    dcm(0, 2) = -coslat * coslon;

    dcm(1, 0) = -sinlat * sinlon;
    dcm(1, 1) = coslon;
    dcm(1, 2) = -coslat * sinlon;

    dcm(2, 0) = coslat;
    dcm(2, 1) = 0;
    dcm(2, 2) = -sinlat;

    return dcm;
}

Quaterniond Earth::qne(const Vector3d &blh) {
    Quaterniond quat;

    double coslon, sinlon, coslat, sinlat;

    coslon = cos(blh[1] * 0.5);
    sinlon = sin(blh[1] * 0.5);
    coslat = cos(-M_PI * 0.25 - blh[0] * 0.5);
    sinlat = sin(-M_PI * 0.25 - blh[0] * 0.5);

    quat.w() = coslat * coslon;
    quat.x() = -sinlat * sinlon;
    quat.y() = sinlat * coslon;
    quat.z() = coslat * sinlon;

    return quat;
}

Vector3d Earth::blh(const Quaterniond &qne, double height) {
    return {-2 * atan(qne.y() / qne.w()) - M_PI * 0.5, 2 * atan2(qne.z(), qne.w()), height};
}

Vector3d Earth::blh2ecef(const Vector3d &blh) {
    double coslat, sinlat, coslon, sinlon;
    double rnh, rn;

    coslat = cos(blh[0]);
    sinlat = sin(blh[0]);
    coslon = cos(blh[1]);
    sinlon = sin(blh[1]);

    rn  = RN(blh[0]);
    rnh = rn + blh[2];

    return {rnh * coslat * coslon, rnh * coslat * sinlon, (rnh - rn * WGS84_E1) * sinlat};
}

Vector3d Earth::ecef2blh(const Vector3d &ecef) {
    double p = sqrt(ecef[0] * ecef[0] + ecef[1] * ecef[1]);
    double rn;
    double lat, lon;
    double h = 0, h2;

    // 初始状态
    lat = atan(ecef[2] / (p * (1.0 - WGS84_E1)));
    lon = 2.0 * atan2(ecef[1], ecef[0] + p);

    do {
        h2  = h;
        rn  = RN(lat);
        h   = p / cos(lat) - rn;
        lat = atan(ecef[2] / (p * (1.0 - WGS84_E1 * rn / (rn + h))));
    } while (fabs(h - h2) > 1.0e-4);

    return {lat, lon, h};
}

Matrix3d Earth::DRi(const Vector3d &blh) {
    Matrix3d dri = Matrix3d::Zero();

    Eigen::Vector2d rmn = meridianPrimeVerticalRadius(blh[0]);

    dri(0, 0) = 1.0 / (rmn[0] + blh[2]);
    dri(1, 1) = 1.0 / ((rmn[1] + blh[2]) * cos(blh[0]));
    dri(2, 2) = -1;
    return dri;
}

Matrix3d Earth::DR(const Vector3d &blh) {
    Matrix3d dr = Matrix3d::Zero();

    Eigen::Vector2d rmn = meridianPrimeVerticalRadius(blh[0]);

    dr(0, 0) = rmn[0] + blh[2];
    dr(1, 1) = (rmn[1] + blh[2]) * cos(blh[0]);
    dr(2, 2) = -1;
    return dr;
}

Vector3d Earth::local2global(const Vector3d &origin, const Vector3d &local) {

    Vector3d ecef0 = blh2ecef(origin);
    Matrix3d cn0e  = cne(origin);

    Vector3d ecef1 = ecef0 + cn0e * local;
    Vector3d blh1  = ecef2blh(ecef1);

    return blh1;
}

Vector3d Earth::global2local(const Vector3d &origin, const Vector3d &global) {
    Vector3d ecef0 = blh2ecef(origin);
    Matrix3d cn0e  = cne(origin);

    Vector3d ecef1 = blh2ecef(global);

    return cn0e.transpose() * (ecef1 - ecef0);
}

Pose Earth::local2global(const Vector3d &origin, const Pose &local) {
    Pose global;

    Vector3d ecef0 = blh2ecef(origin);
    Matrix3d cn0e  = cne(origin);

    Vector3d ecef1 = ecef0 + cn0e * local.t;
    Vector3d blh1  = ecef2blh(ecef1);
    Matrix3d cn1e  = cne(blh1);

    global.t = blh1;
    global.R = cn1e.transpose() * cn0e * local.R;

    return global;
}

Pose Earth::global2local(const Vector3d &origin, const Pose &global) {
    Pose local;

    Vector3d ecef0 = blh2ecef(origin);
    Matrix3d cn0e  = cne(origin);

    Vector3d ecef1 = blh2ecef(global.t);
    Matrix3d cn1e  = cne(global.t);

    local.t = cn0e.transpose() * (ecef1 - ecef0);
    local.R = cn0e.transpose() * cn1e * global.R;

    return local;
}

Vector3d Earth::iewe() {
    return {0, 0, WGS84_WIE};
}

Vector3d Earth::iewn(double lat) {
    return {WGS84_WIE * cos(lat), 0, -WGS84_WIE * sin(lat)};
}

Vector3d Earth::iewn(const Vector3d &origin, const Vector3d &local) {
    Vector3d global = local2global(origin, local);

    return iewn(global[0]);
}

Vector3d Earth::enwn(const Eigen::Vector2d &rmn, const Vector3d &blh, const Vector3d &vel) {
    return {vel[1] / (rmn[1] + blh[2]), -vel[0] / (rmn[0] + blh[2]), -vel[1] * tan(blh[0]) / (rmn[1] + blh[2])};
}

Vector3d Earth::enwn(const Vector3d &origin, const Vector3d &local, const Vector3d &vel) {
    Vector3d global     = local2global(origin, local);
    Eigen::Vector2d rmn = meridianPrimeVerticalRadius(global[0]);

    return enwn(rmn, global, vel);
}