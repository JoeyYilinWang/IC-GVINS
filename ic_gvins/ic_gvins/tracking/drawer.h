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

#ifndef GVINS_DRAWER_H
#define GVINS_DRAWER_H

#include "tracking/frame.h"
#include "tracking/map.h"

#include <memory>

class Drawer {

public:
    typedef std::shared_ptr<Drawer> Ptr;

    virtual ~Drawer() = default;
    
    virtual void run()         = 0;
    virtual void setFinished() = 0;

    void setMap(Map::Ptr map) {
        map_ = std::move(map);
    };
    
    /**
     * @brief 增加新地图点
     */
    virtual void addNewFixedMappoint(Vector3d point)    = 0;

    /**
     * @brief 实际上就是更新位姿而已
     */
    virtual void updateMap(const Eigen::Matrix4d &pose) = 0;

    /**
     * @brief 更新图像，将该图像帧复制到成员变量中
     */
    virtual void updateFrame(Frame::Ptr frame)                                            = 0;

    /**
     * @brief 更新2d地图点及对应匹配点
    */
    virtual void updateTrackedMapPoints(vector<cv::Point2f> map, vector<cv::Point2f> matched,
                                        vector<MapPointType> mappoint_type)               = 0;
    /**
     * @brief 更新新的参考点及对应当前点
     */                                     
    virtual void updateTrackedRefPoints(vector<cv::Point2f> ref, vector<cv::Point2f> cur) = 0;

protected:
    /**
     * @brief 画跟踪图
     */
    void drawTrackingImage(const Mat &raw, Mat &drawed);

protected:
    Map::Ptr map_;

    vector<cv::Point2f> pts2d_cur_, pts2d_ref_, pts2d_map_, pts2d_matched_;
    vector<MapPointType> mappoint_type_;
};

#endif // GVINS_DRAWER_H
