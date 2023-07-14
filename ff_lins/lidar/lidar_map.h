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

#ifndef LIDAR_MAP_H
#define LIDAR_MAP_H

#include "common/integration_state.h"
#include "common/types.h"

#include "lidar/ikd-Tree/ikd_Tree.h"
#include "lidar/lidar_frame.h"
#include "lidar/pointcloud.h"

#include <tbb/tbb.h>
#include <pcl/filters/voxel_grid.h>

#include <memory>

namespace lidar {

using std::string;
using std::vector;
class LidarMap {

public:
    typedef std::shared_ptr<LidarMap> Ptr;
    typedef std::unique_lock<std::mutex> Lock;

    LidarMap() = delete;
    LidarMap(const string &configfile, const string &outputpath);

    bool lidarFrameProcessing(const std::deque<std::pair<IMU, IntegrationState>> &ins_window, const Pose &pose_b_l,
                              LidarFrame::Ptr frame);
    bool lidarFrameProcessing(const IntegrationState &state, const Pose &pose_b_l, LidarFrame::Ptr frame);

    bool addNewPointCloudToMap(PointCloudPtr map_pointcloud);
    bool updateLidarFeaturesInMap();

    const vector<double> &statisticParameters();
    bool saveMapPointCloud();

    bool keyframeSelection(LidarFrame::Ptr frame);
    void mergeNonKeyframes(const vector<LidarFrame::Ptr> &nonkeyframes, LidarFrame::Ptr keyframe);
    void addNewKeyFrame(LidarFrame::Ptr frame);
    void removeOldestLidarFrame();
    void removeNewestLidarFrame();

    const std::deque<LidarFrame::Ptr> &keyframes() {
        return keyframes_;
    }

    bool isWindowFull() {
        return keyframes_.size() > window_size_;
    }

    bool isWindowHalfFull() {
        return keyframes_.size() > window_size_ / 2;
    }

private:
    int findFeaturesInMap(ikd_Tree::KD_TREE<PointType>::Ptr ikdtree, LidarFrame::Ptr ref_frame,
                          LidarFrame::Ptr cur_frame);

private:
    // 平面拟合的点数
    const int PLANE_ESTIMATION_POINT_SZIE = PointCloudCommon::PLANE_ESTIMATION_POINT_SZIE;
    // 最近邻点搜索的最大平方距离
    const double MAXIMUM_SEARCH_SQUARE_DISTANCE = 5.0; // 5 m^2

    // 关键帧选择最小旋转角度
    const double MINIMUM_KEYFRAME_ROTATION = 10 * D2R; // 10 deg
    // 关键帧选择最长时间间隔
    const double MAXIMUM_KEYFRAME_INTERVAL = 0.5 * 0.95; // 0.5 s

    // 地图
    std::unordered_map<ulong, ikd_Tree::KD_TREE<PointType>::Ptr> lidar_frame_map_;
    pcl::VoxelGrid<PointType> downsample_filter_;
    LidarFrame::Ptr marginalized_frame_;

    // 雷达数据
    std::deque<LidarFrame::Ptr> keyframes_;
    tbb::task_group task_group_;

    // 点云处理参数
    double plane_estimation_threshold_;
    double downsample_size_;
    double point_to_plane_std_;
    double lidar_frame_rate_;
    double lidar_frame_dt_;
    double minimum_keyframe_translation_;
    int point_filter_num_;

    size_t window_size_;

    bool is_save_pointcloud_;
    string pcd_file_;
    PointCloudPtr saved_pointcloud_;

    // 统计参数
    vector<double> statistic_parameters_;
    int stat_average_features_;
    int stat_newest_features_;

    double stat_keyframe_interval_;
    double stat_keyframe_translation_;
    double stat_keyframe_rotation_;
    double stat_update_feature_timecost_;
};

} // namespace lidar

#endif // LIDAR_MAP_H
