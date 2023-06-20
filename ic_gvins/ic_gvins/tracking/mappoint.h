/*
 * IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
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

#ifndef GVINS_MAPPOINT_H
#define GVINS_MAPPOINT_H

#include "tracking/camera.h"
#include "tracking/feature.h"

#include <Eigen/Geometry>
#include <atomic>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>

using cv::Mat;
using Eigen::Vector3d;

enum MapPointType {
    MAPPOINT_NONE              = -1,
    MAPPOINT_TRIANGULATED      = 0,
    MAPPOINT_DEPTH_ASSOCIATED  = 1,
    MAPPOINT_DEPTH_INITIALIZED = 2,
    MAPPOINT_FIXED             = 3,
};

class MapPoint {

public:
    typedef std::shared_ptr<MapPoint> Ptr;

    static constexpr double DEFAULT_DEPTH  = 10.0;
    static constexpr double NEAREST_DEPTH  = 1;   // 最近可用路标点深度
    static constexpr double FARTHEST_DEPTH = 200; // 最远可用路标点深度
    
    MapPoint() = delete;
    MapPoint(ulong id, const std::shared_ptr<Frame> &ref_frame, Vector3d pos, cv::Point2f keypoint, double depth,
             MapPointType type);

    Vector3d &pos() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return pos_;
    };

    int observedTimes() const {
        return observed_times_;
    }

    ulong id() const {
        return id_;
    }

    static MapPoint::Ptr createMapPoint(std::shared_ptr<Frame> &ref_frame, Vector3d &pos, cv::Point2f &keypoint,
                                        double depth, MapPointType type);

    void addObservation(const Feature::Ptr &feature);

    void increaseUsedTimes() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        used_times_++;
    }

    void decreaseUsedTimes() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        if (used_times_) {
            used_times_--;
        }
    }

    int usedTimes() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return used_times_;
    }

    void addOptimizedTimes() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        optimized_times_++;
    }

    int optimizedTimes() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return optimized_times_;
    }

    void removeAllObservations() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        observations_.clear();
    }

    std::vector<std::weak_ptr<Feature>> observations() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return observations_;
    }

    void setOutlier(bool isoutlier) {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);

        isoutlier_ = isoutlier;
    }

    bool isOutlier() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return isoutlier_;
    }

    void setReferenceFrame(const std::shared_ptr<Frame> &frame, Vector3d pos, cv::Point2f keypoint, double depth,
                           MapPointType type);

    double depth() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return depth_;
    }

    void updateDepth(double depth) {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        depth_ = depth;
    }

    ulong referenceFrameId();

    MapPointType &mapPointType() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return mappoint_type_;
    }

    std::shared_ptr<Frame> referenceFrame() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return ref_frame_.lock();
    }

    const cv::Point2f &referenceKeypoint() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return ref_frame_keypoint_;
    }

    bool isNeedUpdate() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return isneedupdate_;
    }

private:
    std::vector<std::weak_ptr<Feature>> observations_; // 一个保存了所有观测到这个地图点的特征点的vector，每个特征点都是一个weak_ptr，表示这个特征点可能已经不存在了

    std::mutex mappoint_mutex_;   // 地图点锁
    bool isneedupdate_{false}; // 判定该地图点是否需要更新，默认为false

    Vector3d pos_, pos_tmp_; // 分别表示地图点的位置和临时位置

    // 参考帧中的深度
    double depth_{DEFAULT_DEPTH}, depth_tmp_{DEFAULT_DEPTH}; // 分别表示地图点的深度和临时深度，初始化为默认深度
    cv::Point2f ref_frame_keypoint_, ref_frame_keypoint_tmp_; // 两个二维点，分别表示在参考帧中地图点对应的特征点的像素坐标和临时像素坐标
    std::weak_ptr<Frame> ref_frame_, ref_frame_tmp_; // 两个指向Frame的weak_ptr，分别表示地图点的参考帧和临时参考帧

    int optimized_times_; // 表示地图点被优化的次数
    int used_times_; // 表示地图点被使用的次数
    int observed_times_; // 表示地图点被观测的次数
    bool isoutlier_; // 表示地图点是否是一个异常值

    ulong id_; //表示地图点的ID
    MapPointType mappoint_type_{MAPPOINT_NONE}, mappoint_type_tmp_{MAPPOINT_NONE}; // 分别表示地图点的类型和临时类型，初始化为MAPPOINT_NONE
};

#endif // GVINS_MAPPOINT_H
