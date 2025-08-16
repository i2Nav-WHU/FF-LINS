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

#include "lidar_viewer_rviz.h"

#include "common/logging.h"
#include "common/rotation.h"

#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/random_sample.h>
#include <pcl_conversions/pcl_conversions.h>

LidarViewerRviz::LidarViewerRviz(ros::NodeHandle &nh)
    : is_finished_(false) {
    frame_id_ = "world";

    pose_pub_    = nh.advertise<nav_msgs::Odometry>("/lidar/pose", 10);
    path_pub_    = nh.advertise<nav_msgs::Path>("/lidar/path", 10);
    current_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/lidar/current", 10);
    plane_pub_   = nh.advertise<sensor_msgs::PointCloud2>("/lidar/plane", 10);
    map_pub_     = nh.advertise<sensor_msgs::PointCloud2>("/lidar/map", 10);

    map_pointcloud_ = PointCloudPtr(new PointCloud);
}

void LidarViewerRviz::setFinished() {
    is_finished_ = true;
    update_sem_.notify_one();
}

void LidarViewerRviz::run() {

    LOGI << "Lidar viewer thread is started";
    while (!is_finished_) {
        std::unique_lock<std::mutex> lock(update_mutex_);
        update_sem_.wait(lock);

        if (is_current_pose_) {
            publishOdometry();
            is_current_pose_ = false;
        }

        if (is_current_pointcloud_) {
            publishCurrentPointCloud();
            is_current_pointcloud_ = false;
        }

        if (is_plane_pointcloud_) {
            publishPlanePointCloud();
            is_plane_pointcloud_ = false;
        }

        if (is_map_pointcloud_) {
            publishMapPointCloud();
            is_map_pointcloud_ = false;
        }
    }

    LOGI << "Lidar viewer thread is exited";
}

void LidarViewerRviz::updateMapPointCloud(PointCloudPtr pointcloud) {
    std::unique_lock<std::mutex> lock(map_mutex_);

    // 随机降采样 1/10
    pcl::RandomSample<PointType> random_sample;
    random_sample.setSample(pointcloud->size() / 10);
    random_sample.setInputCloud(pointcloud);
    random_sample.filter(*map_pointcloud_);
    is_map_pointcloud_ = true;

    update_sem_.notify_one();
}

void LidarViewerRviz::updateCurrentPointCloud(PointCloudPtr pointcloud) {
    std::unique_lock<std::mutex> lock(map_mutex_);
    current_pointcloud_    = std::move(pointcloud);
    is_current_pointcloud_ = true;

    update_sem_.notify_one();
}

void LidarViewerRviz::updatePlanePointCloud(PointCloudPtr pointcloud) {
    std::unique_lock<std::mutex> lock(map_mutex_);
    plane_pointcloud_    = std::move(pointcloud);
    is_plane_pointcloud_ = true;

    update_sem_.notify_one();
}

void LidarViewerRviz::updateCurrentPose(const Pose &pose) {
    std::unique_lock<std::mutex> lock(map_mutex_);
    pose_            = pose;
    is_current_pose_ = true;

    update_sem_.notify_one();
}

void LidarViewerRviz::publishOdometry() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    nav_msgs::Odometry odometry;

    auto quaternion = Rotation::matrix2quaternion(pose_.R);
    auto stamp      = ros::Time::now();

    // Odometry
    odometry.header.stamp            = stamp;
    odometry.header.frame_id         = frame_id_;
    odometry.pose.pose.position.x    = pose_.t.x();
    odometry.pose.pose.position.y    = pose_.t.y();
    odometry.pose.pose.position.z    = pose_.t.z();
    odometry.pose.pose.orientation.x = quaternion.x();
    odometry.pose.pose.orientation.y = quaternion.y();
    odometry.pose.pose.orientation.z = quaternion.z();
    odometry.pose.pose.orientation.w = quaternion.w();
    pose_pub_.publish(odometry);

    // Path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp    = stamp;
    pose_stamped.header.frame_id = frame_id_;
    pose_stamped.pose            = odometry.pose.pose;

    path_.header.stamp    = stamp;
    path_.header.frame_id = frame_id_;
    path_.poses.push_back(pose_stamped);
    path_pub_.publish(path_);
}

void LidarViewerRviz::publishCurrentPointCloud() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    auto stamp = ros::Time::now();

    sensor_msgs::PointCloud2 current_msg;
    pcl::toROSMsg(*current_pointcloud_, current_msg);

    // toROSMsg() will overwrite these parameters
    current_msg.header.stamp    = stamp;
    current_msg.header.frame_id = frame_id_;

    current_pub_.publish(current_msg);
}

void LidarViewerRviz::publishPlanePointCloud() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    auto stamp = ros::Time::now();

    sensor_msgs::PointCloud2 plane_msg;
    pcl::toROSMsg(*plane_pointcloud_, plane_msg);

    // toROSMsg() will overwrite these parameters
    plane_msg.header.stamp    = stamp;
    plane_msg.header.frame_id = frame_id_;

    plane_pub_.publish(plane_msg);
}

void LidarViewerRviz::publishMapPointCloud() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    auto stamp = ros::Time::now();

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_pointcloud_, map_msg);

    // toROSMsg() will overwrite these parameters
    map_msg.header.stamp    = stamp;
    map_msg.header.frame_id = frame_id_;

    map_pub_.publish(map_msg);
}
