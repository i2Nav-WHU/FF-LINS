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

#ifndef LIDAR_VIEWER_H
#define LIDAR_VIEWER_H

#include "common/types.h"
#include "lidar/lidar.h"

#include <memory>

namespace lidar {

class LidarViewer {

public:
    typedef std::shared_ptr<LidarViewer> Ptr;

    virtual ~LidarViewer() = default;

    virtual void run()         = 0;
    virtual void setFinished() = 0;

    virtual void updateMapPointCloud(PointCloudPtr pointcloud)      = 0;
    virtual void updateCurrentPointCloud(PointCloudPtr pointcloud)  = 0;
    virtual void updatePlanePointCloud(PointCloudPtr pointcloud)    = 0;
    virtual void updateCurrentPose(const Pose &pose)                = 0;

protected:
    PointCloudPtr local_map_pointcloud_;
    PointCloudPtr map_pointcloud_;
    PointCloudPtr current_pointcloud_;
    PointCloudPtr plane_pointcloud_;
    Pose pose_;
};

} // namespace lidar

#endif // LIDAR_VIEWER_H