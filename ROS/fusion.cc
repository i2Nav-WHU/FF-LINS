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

#include "fusion.h"
#include "lidar/lidar_viewer_rviz.h"

#include "common/gpstime.h"
#include "common/logging.h"
#include "common/misc.h"
#include "lidar/lidar_frame.h"

#include <absl/time/clock.h>
#include <absl/time/time.h>
#include <filesystem>
#include <sensor_msgs/image_encodings.h>
#include <yaml-cpp/yaml.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

std::atomic<bool> global_finished = false;

void Fusion::setFinished() {
    if (lins_) {
        lins_->setFinished();
    }
}

void Fusion::run() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // message topic
    string imu_topic, lidar_topic;
    pnh.param<string>("imu_topic", imu_topic, "/livox/imu");
    pnh.param<string>("lidar_topic", lidar_topic, "/livox/lidar");

    // 读取ROS包
    string bagfile;
    pnh.param<bool>("is_read_bag", is_read_bag_, false);
    pnh.param<string>("bagfile", bagfile, "");

    //  parameter
    string configfile;
    pnh.param<string>("configfile", configfile, "");

    // 加载配置
    YAML::Node config;
    try {
        config = YAML::LoadFile(configfile);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file " << configfile << std::endl;
        return;
    }
    auto outputpath        = config["outputpath"].as<string>();
    auto is_make_outputdir = config["is_make_outputdir"].as<bool>();

    // 如果文件夹不存在, 尝试创建
    if (!std::filesystem::is_directory(outputpath)) {
        std::filesystem::create_directory(outputpath);
    }
    if (!std::filesystem::is_directory(outputpath)) {
        std::cout << "Failed to open outputpath" << std::endl;
        return;
    }

    if (is_make_outputdir) {
        absl::CivilSecond cs = absl::ToCivilSecond(absl::Now(), absl::LocalTimeZone());
        absl::StrAppendFormat(&outputpath, "/T%04d%02d%02d%02d%02d%02d", cs.year(), cs.month(), cs.day(), cs.hour(),
                              cs.minute(), cs.second());
        std::filesystem::create_directory(outputpath);
    }
    // 设置Log输出路径
    FLAGS_log_dir = outputpath;

    double imu_data_rate = config["imu"]["imudatarate"].as<double>();
    imu_data_dt_         = 1.0 / imu_data_rate;

    // LiDAR参数
    int scan_line            = config["lidar"]["scan_line"].as<int>();
    double nearest_distance  = config["lidar"]["nearest_distance"].as<double>();
    double farthest_distance = config["lidar"]["farthest_distance"].as<double>();

    // lidar数据转换对象
    lidar_converter_ = std::make_shared<LidarConverter>(scan_line, nearest_distance, farthest_distance);

    // 创建LINS
    auto lidar_viewer = std::make_shared<LidarViewerRviz>(nh);
    lins_             = std::make_shared<LINS>(configfile, outputpath, lidar_viewer);

    // check is initialized
    if (!lins_->isRunning()) {
        LOGE << "Fusion ROS terminate";
        return;
    }

    // 处理ROS数据
    if (is_read_bag_) {
        LOGI << "Start to read ROS bag file";
        processRead(imu_topic, lidar_topic, bagfile);
        LOGI << "Finish to read ROS bag file";

        // 结束处理
        global_finished = true;
    } else {
        processSubscribe(imu_topic, lidar_topic, nh);
    }
}

void Fusion::processSubscribe(const string &imu_topic, const string &lidar_topic, ros::NodeHandle &nh) {
    // GNSS and IMU
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 200, &Fusion::imuCallback, this);

    // Livox lidar
    ros::Subscriber lidar_sub =
        nh.subscribe<livox_ros_driver::CustomMsg>(lidar_topic, 10, &Fusion::livoxCallback, this);

    LOGI << "Waiting ROS message...";

    // Enter message loopback
    ros::spin();
}

void Fusion::processRead(const string &imu_topic, const string &lidar_topic, const string &bagfile) {
    // 消息列表
    vector<string> topics;
    topics.emplace_back(imu_topic);
    topics.emplace_back(lidar_topic);

    // 遍历ROS包
    rosbag::Bag bag(bagfile);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for (rosbag::MessageInstance const &msg : view) {
        // 等待数据处理完毕
        while (!global_finished && !lins_->canAddData()) {
            usleep(100);
        }

        // 强制退出信号
        if (global_finished) {
            return;
        }

        sensor_msgs::ImuConstPtr imu_ptr = msg.instantiate<sensor_msgs::Imu>();
        if (imu_ptr) {
            imuCallback(imu_ptr);
        }

        livox_ros_driver::CustomMsgConstPtr livox_ptr = msg.instantiate<livox_ros_driver::CustomMsg>();
        if (livox_ptr) {
            livoxCallback(livox_ptr);
        }
    }

    // 等待数据处理结束
    while (!lins_->isBufferEmpty()) {
        usleep(100);
    }
}

void Fusion::imuCallback(const sensor_msgs::ImuConstPtr &imumsg) {
    imu_pre_ = imu_;

    // Time convertion
    double unixsecond = imumsg->header.stamp.toSec();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    imu_.time = weeksec;
    // delta time
    imu_.dt = imu_.time - imu_pre_.time;

    // IMU measurements, Front-Right-Down
    imu_.dtheta[0] = imumsg->angular_velocity.x * imu_.dt;
    imu_.dtheta[1] = imumsg->angular_velocity.y * imu_.dt;
    imu_.dtheta[2] = imumsg->angular_velocity.z * imu_.dt;
    imu_.dvel[0]   = imumsg->linear_acceleration.x * imu_.dt;
    imu_.dvel[1]   = imumsg->linear_acceleration.y * imu_.dt;
    imu_.dvel[2]   = imumsg->linear_acceleration.z * imu_.dt;

    // 数据未准备好
    if (imu_pre_.time == 0) {
        return;
    }

    addImuData(imu_);
}

void Fusion::addImuData(const IMU &imu) {
    imu_buffer_.push(imu);
    while (!imu_buffer_.empty()) {
        auto temp = imu_buffer_.front();

        // add new IMU
        if (lins_->addNewImu(temp)) {
            imu_buffer_.pop();
        } else {
            // thread lock failed, try next time
            break;
        }
    }
}

void Fusion::livoxCallback(const livox_ros_driver::CustomMsgConstPtr &lidarmsg) {
    PointCloudCustomPtr pointcloud = PointCloudCustomPtr(new PointCloudCustom);
    double start, end;

    lidar_converter_->livoxPointCloudConvertion(lidarmsg, pointcloud, start, end, true);

    // Create lidar frame
    auto frame = LidarFrame::createFrame(pointcloud->points.front().time, pointcloud->points.back().time, pointcloud);
    lins_->addNewLidarFrame(frame);
}