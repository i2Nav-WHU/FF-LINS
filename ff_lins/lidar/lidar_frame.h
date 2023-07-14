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

#ifndef LIDAR_FRAME_H
#define LIDAR_FRAME_H

#include "common/types.h"

#include "lidar/lidar_feature.h"
#include "lidar/pointcloud.h"

#include <memory>
#include <mutex>

namespace lidar {

class LidarFrame {

public:
    typedef std::shared_ptr<LidarFrame> Ptr;

    LidarFrame() = delete;
    LidarFrame(ulong id, double start_time, double end_time, PointCloudCustomPtr pointcloud) {
        id_             = id;
        start_time_     = start_time;
        end_time_       = end_time;
        stamp_          = end_time;
        raw_pointcloud_ = std::move(pointcloud);
        num_features_   = 0;
        num_covered_    = 0;
    }

    static LidarFrame::Ptr createFrame(double start_time, double end_time, PointCloudCustomPtr pointcloud) {
        static ulong factory_id = 0;
        return std::make_shared<LidarFrame>(factory_id++, start_time, end_time, pointcloud);
    }

    ulong id() {
        return id_;
    }

    const Pose &pose() {
        return pose_;
    }

    void setPose(Pose pose) {
        pose_ = std::move(pose);
    }

    const Vector3d &velocity() {
        return velocity_;
    }

    void setVelocity(Vector3d velocity) {
        velocity_ = std::move(velocity);
    }

    const Vector3d &angularVelocity() {
        return omega_;
    }

    void setAngularVelocity(Vector3d omega) {
        omega_ = std::move(omega);
    }

    double timeDelay() const {
        return td_;
    }

    void setTimeDelay(double td) {
        td_ = td;
    }

    double stamp() const {
        return stamp_;
    }

    void setStamp(double stamp) {
        stamp_ = stamp;
    }

    double startTime() const {
        return start_time_ + td_;
    }

    double endTime() const {
        return end_time_ + td_;
    }

    void setUndistortedPointCloud(PointCloudPtr lidar_full, PointCloudPtr lidar, PointCloudPtr world,
                                  PointCloudPtr map_full, PointCloudPtr map) {
        pointcloud_lidar_full_ = std::move(lidar_full);
        pointcloud_lidar_      = std::move(lidar);
        pointcloud_world_      = std::move(world);
        pointcloud_map_full_   = std::move(map_full);
        pointcloud_map_        = std::move(map);
    }

    PointCloudCustomPtr rawPointCloud() const {
        return raw_pointcloud_;
    }

    PointCloudPtr pointCloudLidar() const {
        return pointcloud_lidar_;
    }

    PointCloudPtr pointCloudWorld() const {
        return pointcloud_world_;
    }

    PointCloudPtr pointCloudLidarFull() const {
        return pointcloud_lidar_full_;
    }

    PointCloudPtr pointCloudMapFull() const {
        return pointcloud_map_full_;
    }

    PointCloudPtr pointCloudMap() const {
        return pointcloud_map_;
    }

    std::vector<LidarFeature::Ptr> &features() {
        return features_;
    }

    void setFrameInMap() {
        is_in_map_ = true;
    }

    bool isFrameInMap() {
        return is_in_map_;
    }

    void setKeyFrame() {
        is_keyframe_ = true;
    }

    bool isKeyFrame() const {
        return is_keyframe_;
    }

    int numFeatures() const {
        return num_features_;
    }

    void setNumFeatures(int num_features) {
        num_features_ = num_features;
    }

    int numCovered() const {
        return num_covered_;
    }

    void setNumCovered(int num_covered) {
        num_covered_ = num_covered;
    }

private:
    PointCloudCustomPtr raw_pointcloud_; // 未补偿畸变的原始点云

    // 点云及其对应的特
    PointCloudPtr pointcloud_lidar_full_;     // 去畸变的点云, Lidar系
    PointCloudPtr pointcloud_lidar_;          // 去畸变降采样的点云, Lidar系
    PointCloudPtr pointcloud_world_;          // 去畸变降采样的点云, world系
    PointCloudPtr pointcloud_map_full_;       // 去畸变的累积点云, lidar系
    PointCloudPtr pointcloud_map_;            // 去畸变降采样的累积点云, lidar系
    std::vector<LidarFeature::Ptr> features_; // 点云特征参数

    int num_features_;
    int num_covered_;

    Pose pose_;         // Lidar位姿
    Vector3d velocity_; // IMU速度
    Vector3d omega_;    // IMU角速度

    double stamp_; // 对应于结束时间
    double start_time_, end_time_;
    double td_{0};
    bool is_in_map_{false};
    bool is_keyframe_{false};

    ulong id_;
};

} // namespace lidar

#endif // LIDAR_FRAME_H