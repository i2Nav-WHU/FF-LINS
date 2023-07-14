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

#ifndef LIDAR_FEATURE_H
#define LIDAR_FEATURE_H

#include "lidar/pointcloud.h"

#include <Eigen/Geometry>
#include <memory>

namespace lidar {

using Eigen::Vector3d;

class LidarFeature {

public:
    typedef std::shared_ptr<LidarFeature> Ptr;

    enum FeatureType {
        FEATURE_NONE   = -1,
        FEATURE_PLANE  = 0,
        FEATURE_CORNER = 1,
    };

    LidarFeature() = default;
    LidarFeature(PointVector nearest_points, Vector3d unit_normal_vector, double norm_inverse, FeatureType type)
        : nearest_points_(std::move(nearest_points))
        , unit_normal_vector_(std::move(unit_normal_vector))
        , norm_inverse_(norm_inverse)
        , type_(type) {
    }

    static std::shared_ptr<LidarFeature> createFeature(const PointVector &nearest_points, const Vector3d &unit_normal_vector,
                                                  double norm_inverse, FeatureType type) {
        return std::make_shared<LidarFeature>(nearest_points, unit_normal_vector, norm_inverse, type);
    }

    FeatureType featureType() {
        return type_;
    }

    void updateFeature(PointVector nearest_points, Vector3d unit_normal_vector, double norm_inverse, FeatureType type,
                       bool is_outlier) {
        nearest_points_     = std::move(nearest_points);
        unit_normal_vector_ = std::move(unit_normal_vector);
        norm_inverse_       = norm_inverse;
        type_               = type;
        is_outlier_         = is_outlier;
    }

    const PointVector &nearestPoints() {
        return nearest_points_;
    }

    const Vector3d &unitNormalVector() {
        return unit_normal_vector_;
    }

    double normInverse() const {
        return norm_inverse_;
    }

    void setOutlier() {
        is_outlier_ = true;
        outlier_counts_++;
    }

    bool isOutlier() const {
        return is_outlier_;
    }

    int outlierCounts() const {
        return outlier_counts_;
    }

    void setPointStd(double std) {
        point_std_ = std;
    }

    double pointStd() {
        return point_std_;
    }

private:
    PointVector nearest_points_;
    Vector3d unit_normal_vector_;
    double norm_inverse_;
    double point_std_;

    bool is_outlier_{false};
    int outlier_counts_{0};
    FeatureType type_{FEATURE_NONE};
};

} // namespace lidar

#endif // LIDAR_FEATURE_H