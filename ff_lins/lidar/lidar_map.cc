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

#include "lidar/lidar_map.h"

#include "common/logging.h"
#include "common/misc.h"
#include "common/timecost.h"

#include <tbb/tbb.h>
#include <yaml-cpp/yaml.h>

#include <pcl/filters/random_sample.h>
#include <pcl/io/pcd_io.h>

namespace lidar {

LidarMap::LidarMap(const string &configfile, const string &outputpath) {
    // 加载配置
    YAML::Node config;
    std::vector<double> vecdata;
    config = YAML::LoadFile(configfile);

    point_to_plane_std_ = config["optimizer"]["optimize_point_to_plane_std"].as<double>();
    window_size_        = config["optimizer"]["optimize_window_size"].as<size_t>();

    // 点云处理参数

    downsample_size_              = config["lidar"]["downsample_size"].as<double>();
    lidar_frame_rate_             = config["lidar"]["frame_rate"].as<double>();
    plane_estimation_threshold_   = config["lidar"]["plane_estimation_threshold"].as<double>();
    minimum_keyframe_translation_ = config["lidar"]["minimum_keyframe_translation"].as<double>();
    point_filter_num_             = config["lidar"]["point_filter_num"].as<int>();
    is_save_pointcloud_           = config["lidar"]["is_save_pointcloud"].as<bool>();
    if (is_save_pointcloud_) {
        pcd_file_ = outputpath + "/lins_map.pcd";

        saved_pointcloud_ = PointCloudPtr(new PointCloud);
    }

    // 体素滤波降采样
    downsample_filter_.setLeafSize(downsample_size_, downsample_size_, downsample_size_);
    lidar_frame_dt_ = 1.0 / lidar_frame_rate_;

    LOGI << "Lidar mapping is constructed";
}

bool LidarMap::keyframeSelection(LidarFrame::Ptr frame) {
    auto last_keyframe = keyframes_.back();

    // 关键帧选择: 时间, 平移, 旋转, 覆盖率
    stat_keyframe_interval_    = frame->stamp() - last_keyframe->stamp();
    stat_keyframe_translation_ = (frame->pose().t - last_keyframe->pose().t).norm();
    Eigen::AngleAxisd angleaxis(frame->pose().R.transpose() * last_keyframe->pose().R);
    stat_keyframe_rotation_ = fabs(angleaxis.angle());

    if ((stat_keyframe_interval_ > MAXIMUM_KEYFRAME_INTERVAL) ||
        (stat_keyframe_translation_ > minimum_keyframe_translation_) ||
        (stat_keyframe_rotation_ > MINIMUM_KEYFRAME_ROTATION)) {

        // 设置为关键帧, 附加帧清零
        frame->setKeyFrame();
        LOGI << "New lidar keyframe at " << Logging::doubleData(frame->stamp()) << " with interval "
             << stat_keyframe_interval_ << " s, translation " << stat_keyframe_translation_ << ", rotation "
             << stat_keyframe_rotation_;
    }

    return frame->isKeyFrame();
}

void LidarMap::addNewKeyFrame(LidarFrame::Ptr frame) {
    if (!keyframes_.empty()) {
        auto last_keyframe         = keyframes_.back();
        stat_keyframe_interval_    = frame->stamp() - last_keyframe->stamp();
        stat_keyframe_translation_ = (frame->pose().t - last_keyframe->pose().t).norm();
        Eigen::AngleAxisd angleaxis(frame->pose().R.transpose() * last_keyframe->pose().R);
        stat_keyframe_rotation_ = fabs(angleaxis.angle());
    }
    keyframes_.push_back(frame);

    // 构建新的ikdtree, 加入雷达系的点云
    auto ikdtree = std::make_shared<ikd_Tree::KD_TREE<PointType>>();
    ikdtree->setDownsampleParam(downsample_size_);
    ikdtree->build(frame->pointCloudMap()->points);
    lidar_frame_map_[frame->id()] = ikdtree;
}

void LidarMap::removeOldestLidarFrame() {
    marginalized_frame_ = keyframes_.front();
    keyframes_.pop_front();

    if (lidar_frame_map_.find(marginalized_frame_->id()) != lidar_frame_map_.end()) {
        auto ikd_tree = lidar_frame_map_[marginalized_frame_->id()];
        lidar_frame_map_.erase(marginalized_frame_->id());

        // Release the ikd-Tree
        task_group_.run([ikd_tree]() {
            ikd_tree->release();
        });
    }

    LOGI << "Remove lidar keyframe at " << Logging::doubleData(marginalized_frame_->stamp());
}

void LidarMap::removeNewestLidarFrame() {
    auto newest_frame = keyframes_.back();
    keyframes_.pop_back();

    if (lidar_frame_map_.find(newest_frame->id()) != lidar_frame_map_.end()) {
        auto ikd_tree = lidar_frame_map_[newest_frame->id()];
        lidar_frame_map_.erase(newest_frame->id());

        // Release the ikd-Tree
        task_group_.run([ikd_tree]() {
            ikd_tree->release();
        });
    }

    LOGI << "Remove lidar frame at " << Logging::doubleData(newest_frame->stamp());
}

void LidarMap::mergeNonKeyframes(const vector<LidarFrame::Ptr> &nonkeyframes, LidarFrame::Ptr keyframe) {
    Pose keyframe_pose = keyframe->pose();
    Pose nonkeyframe_pose, relative_pose;

    for (auto nonkeyframe : nonkeyframes) {
        nonkeyframe_pose = nonkeyframe->pose();

        // From nonkeyframe to keyframe
        relative_pose.R = keyframe_pose.R.transpose() * nonkeyframe_pose.R;
        relative_pose.t = keyframe_pose.R.transpose() * (nonkeyframe_pose.t - keyframe_pose.t);

        PointCloudCommon::pointCloudProjection(relative_pose, nonkeyframe->pointCloudMapFull(),
                                               nonkeyframe->pointCloudMapFull());

        // 点云累积
        *keyframe->pointCloudMapFull() += *nonkeyframe->pointCloudMapFull();
    }

    // 降采样
    downsample_filter_.setInputCloud(keyframe->pointCloudMapFull());
    downsample_filter_.filter(*keyframe->pointCloudMap());
}

bool LidarMap::lidarFrameProcessing(const std::deque<std::pair<IMU, IntegrationState>> &ins_window,
                                    const Pose &pose_b_l, LidarFrame::Ptr frame) {

    // 用于建立雷达帧的地图
    size_t points_size                = frame->rawPointCloud()->size();
    PointCloudPtr map_pointcloud_full = PointCloudPtr(new PointCloud);
    PointCloudPtr map_pointcloud      = PointCloudPtr(new PointCloud);
    map_pointcloud_full->resize(points_size);

    Pose lidar_pose;
    MISC::getPoseFromInsWindow(ins_window, pose_b_l, frame->stamp(), lidar_pose);
    double td = frame->timeDelay();

    auto undistortion_function = [&](const tbb::blocked_range<size_t> &range) {
        for (size_t k = range.begin(); k != range.end(); k++) {
            auto &raw = frame->rawPointCloud()->points[k];

            // 数据转换
            PointType point;
            point.intensity        = raw.intensity;
            point.getVector3fMap() = raw.getVector3fMap();

            // 当前点的位姿, 考虑每个点的时间偏移
            Pose curr_pose;
            MISC::getPoseFromInsWindow(ins_window, pose_b_l, raw.time + td, curr_pose);

            // 投影到第一个点的位姿上
            map_pointcloud_full->points[k] = PointCloudCommon::relativeProjection(lidar_pose, curr_pose, point);
        }
    };
    tbb::parallel_for(tbb::blocked_range<size_t>(0, points_size), undistortion_function);

    // 运动畸变补偿降采样的点云
    PointCloudPtr undis_pointcloud = PointCloudPtr(new PointCloud);
    undis_pointcloud->reserve(points_size);

    // 抽取降采样
    int num_points = 0;
    for (size_t k = 0; k < points_size; k++) {
        const auto &point = map_pointcloud_full->points[k];
        if (num_points++ % point_filter_num_ == 0) {
            undis_pointcloud->points.push_back(point);
        }
    }

    // 降采样
    PointCloudPtr down_pointcloud = PointCloudPtr(new PointCloud);
    downsample_filter_.setInputCloud(undis_pointcloud);
    downsample_filter_.filter(*down_pointcloud);

    // 投影到世界系
    frame->setPose(lidar_pose);
    PointCloudPtr world_pointcloud = PointCloudPtr(new PointCloud);
    PointCloudCommon::pointCloudProjection(lidar_pose, down_pointcloud, world_pointcloud);

    // 去畸变且降采样的点云
    frame->setUndistortedPointCloud(undis_pointcloud, down_pointcloud, world_pointcloud, map_pointcloud_full,
                                    map_pointcloud);

    return true;
}

bool LidarMap::lidarFrameProcessing(const IntegrationState &state, const Pose &pose_b_l, LidarFrame::Ptr frame) {
    // 直接累积静态点云, 无需去畸变

    // 用于建立雷达帧的地图
    size_t points_size                = frame->rawPointCloud()->size();
    PointCloudPtr map_pointcloud_full = PointCloudPtr(new PointCloud);
    PointCloudPtr map_pointcloud      = PointCloudPtr(new PointCloud);
    map_pointcloud_full->resize(points_size);

    Pose lidar_pose = MISC::stateToPoseWithExtrinsic(state, pose_b_l);
    for (size_t k = 0; k < points_size; k++) {
        auto &raw = frame->rawPointCloud()->points[k];

        // 数据转换
        PointType point;
        point.intensity        = raw.intensity;
        point.getVector3fMap() = raw.getVector3fMap();

        map_pointcloud_full->points[k] = point;
    }

    // 运动畸变补偿降采样的点云
    PointCloudPtr undis_pointcloud = PointCloudPtr(new PointCloud);
    undis_pointcloud->reserve(points_size);

    // 抽取降采样
    int num_points = 0;
    for (size_t k = 0; k < points_size; k++) {
        const auto &point = map_pointcloud_full->points[k];
        if (num_points++ % point_filter_num_ == 0) {
            undis_pointcloud->points.push_back(point);
        }
    }

    // 降采样
    PointCloudPtr down_pointcloud = PointCloudPtr(new PointCloud);
    downsample_filter_.setInputCloud(undis_pointcloud);
    downsample_filter_.filter(*down_pointcloud);

    // 投影到世界系
    frame->setPose(lidar_pose);
    PointCloudPtr world_pointcloud = PointCloudPtr(new PointCloud);
    PointCloudCommon::pointCloudProjection(lidar_pose, down_pointcloud, world_pointcloud);

    // 去畸变且降采样的点云
    frame->setUndistortedPointCloud(undis_pointcloud, down_pointcloud, world_pointcloud, map_pointcloud_full,
                                    map_pointcloud);

    return true;
}

int LidarMap::findFeaturesInMap(ikd_Tree::KD_TREE<PointType>::Ptr ikdtree, LidarFrame::Ptr ref_frame,
                                LidarFrame::Ptr cur_frame) {

    auto pointcloud_lidar = cur_frame->pointCloudLidar();
    auto pointcloud_world = cur_frame->pointCloudWorld();
    size_t points_size    = pointcloud_lidar->size();

    // 特征保存在参考帧
    auto &features = ref_frame->features();

    // 分配内存
    features.resize(points_size);
    for (size_t k = 0; k < points_size; k++) {
        features[k] = std::make_shared<LidarFeature>();
    }

    // 世界系到参考雷达系的位姿变换
    Pose pose_ref_world;
    pose_ref_world.R = ref_frame->pose().R.transpose();
    pose_ref_world.t = -pose_ref_world.R * ref_frame->pose().t;

    vector<int> feature_types(points_size, 0);
    vector<int> feature_covered(points_size, 0);
    auto finding_function = [&](const tbb::blocked_range<size_t> &range) {
        for (size_t k = range.begin(); k != range.end(); k++) {
            const PointType &world = pointcloud_world->points[k];
            const PointType &lidar = pointcloud_lidar->points[k];

            // 投影到参考雷达帧坐标系
            PointType reference;
            reference.getVector3fMap() =
                (pose_ref_world.R * world.getVector3fMap().cast<double>() + pose_ref_world.t).cast<float>();

            vector<float> point_square_distance;
            vector<double> point_to_plane_dis;
            PointVector nearest_points;
            Vector3d unit_normal_vector;
            double norm_inverse = 0;
            double point_std    = point_to_plane_std_;
            auto feature_type   = LidarFeature::FEATURE_NONE;

            // 最近邻点搜索
            ikdtree->nearestSearch(reference, PLANE_ESTIMATION_POINT_SZIE, nearest_points, point_square_distance,
                                   MAXIMUM_SEARCH_SQUARE_DISTANCE);

            if (nearest_points.size() == static_cast<size_t>(PLANE_ESTIMATION_POINT_SZIE)) {
                feature_covered[k] = 1;

                if (point_square_distance.back() < MAXIMUM_SEARCH_SQUARE_DISTANCE) {
                    // 拟合平面
                    bool is_plane =
                        PointCloudCommon::planeEstimation(nearest_points, plane_estimation_threshold_,
                                                          unit_normal_vector, norm_inverse, point_to_plane_dis);
                    // 平面检查
                    if (is_plane) {
                        // s = dis / sqrt(norm)
                        double dis =
                            fabs(unit_normal_vector.dot(reference.getVector3fMap().cast<double>()) + norm_inverse);
                        double scalar = dis / sqrt(lidar.getVector3fMap().norm());
                        if ((scalar < 0.1) && (dis < 4.5 * point_to_plane_std_)) {
                            feature_type     = LidarFeature::FEATURE_PLANE;
                            feature_types[k] = 1;
                        }
                    }
                }
            }

            // 更新特征
            features[k]->updateFeature(nearest_points, unit_normal_vector, norm_inverse, feature_type, false);
            features[k]->setPointStd(point_std);
        }
    };
    tbb::parallel_for(tbb::blocked_range<size_t>(0, points_size), finding_function);

    // 统计有效特征
    int num_features = std::accumulate(feature_types.begin(), feature_types.end(), 0);
    int num_covered  = std::accumulate(feature_covered.begin(), feature_covered.end(), 0);

    cur_frame->setNumFeatures(num_features);
    cur_frame->setNumCovered(num_covered);

    return num_features;
}

bool LidarMap::addNewPointCloudToMap(PointCloudPtr map_pointcloud) {
    if (!marginalized_frame_) {
        return false;
    }

    // 可视化点云
    PointCloudCommon::pointCloudProjection(marginalized_frame_->pose(), marginalized_frame_->pointCloudMapFull(),
                                           map_pointcloud);
    marginalized_frame_ = nullptr;

    // 保存点云地图
    if (is_save_pointcloud_ && !map_pointcloud->empty()) {
        *saved_pointcloud_ += *map_pointcloud;
    }

    return true;
}

bool LidarMap::updateLidarFeaturesInMap() {
    TimeCost timecost;

    // 搜索特征点
    vector<int> frame_features(keyframes_.size());
    size_t frame_size = keyframes_.size();
    auto cur_frame    = keyframes_.back();

    auto updating_function = [&](const tbb::blocked_range<size_t> &range) {
        for (size_t k = range.begin(); k != range.end(); k++) {
            auto ref_frame = keyframes_[k];

            auto ikdtree      = lidar_frame_map_[ref_frame->id()];
            frame_features[k] = findFeaturesInMap(ikdtree, ref_frame, cur_frame);
        }
    };
    tbb::parallel_for(tbb::blocked_range<size_t>(0, frame_size - 1), updating_function);

    // 拷贝与最近的关键帧的特征用于可视化
    cur_frame->features() = keyframes_[keyframes_.size() - 2]->features();
    frame_features.back() = cur_frame->numFeatures();

    stat_newest_features_  = frame_features.back();
    stat_average_features_ = std::accumulate(frame_features.begin(), frame_features.end(), 0.0) / keyframes_.size();

    stat_update_feature_timecost_ = timecost.costInMillisecond();
    LOGI << "Find the newest " << stat_newest_features_ << " featureas and average " << stat_average_features_
         << " features at " << Logging::doubleData(keyframes_.back()->stamp()) << ", "
         << "update map cost " << stat_update_feature_timecost_ << " ms";

    return true;
}

bool LidarMap::saveMapPointCloud() {
    if (is_save_pointcloud_) {
        LOGW << "Start to save pcd file";

        // 随机降采样 1/10
        pcl::RandomSample<PointType> random_sample;
        random_sample.setSample(saved_pointcloud_->size() / 10);

        PointCloud downsampled;
        random_sample.setInputCloud(saved_pointcloud_);
        random_sample.filter(downsampled);

        LOGW << "Total " << downsampled.size() << " downsampled points";

        if (!downsampled.empty()) {
            pcl::PCDWriter pcd_writer;
            pcd_writer.writeBinaryCompressed(pcd_file_, downsampled);
        }
    }

    return is_save_pointcloud_;
}

const vector<double> &LidarMap::statisticParameters() {
    statistic_parameters_.clear();

    statistic_parameters_.push_back(keyframes_.back()->stamp());
    statistic_parameters_.push_back(stat_average_features_);
    statistic_parameters_.push_back(stat_newest_features_);
    statistic_parameters_.push_back(keyframes_.back()->pointCloudLidarFull()->size());
    statistic_parameters_.push_back(stat_keyframe_interval_);
    statistic_parameters_.push_back(stat_keyframe_translation_);
    statistic_parameters_.push_back(stat_keyframe_rotation_ * R2D);
    statistic_parameters_.push_back(stat_update_feature_timecost_);

    return statistic_parameters_;
}

} // namespace lidar
