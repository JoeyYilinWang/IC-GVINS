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

#include "tracking/drawer.h"

void Drawer::drawTrackingImage(const Mat &raw, Mat &drawed) {

    int rectangle_width  = 21;
    float rectangle_half = 10;
    int line_width       = 2;

    if (raw.rows < 600) {
        rectangle_width = 15;
        rectangle_half  = 7;
    }

    cv::Size rectangle_size      = cv::Size(rectangle_width, rectangle_width);
    cv::Point2f rectangle_center = cv::Point2f(rectangle_half, rectangle_half);

    // 颜色转换
    if (raw.channels() == 1) {
        cv::cvtColor(raw, drawed, cv::COLOR_GRAY2BGR);
    } else {
        raw.copyTo(drawed);
    }

    // 跟踪上的地图点
    // 将匹配上的点进行连线，但这里有个疑惑
    // 匹配点是针对两帧来说的，但这里确在同一帧进行连线。假设第二个点是针对该帧的，那么前一个点的坐标如果也针对该帧的话又有何意义呢？因为前一个点一定对应前一帧
    for (size_t k = 0; k < pts2d_matched_.size(); k++) {
        if (mappoint_type_[k] == MAPPOINT_TRIANGULATED) {
            // 绘制线段，起点为pts2d_map_[k]，终点为pts2d_matched_[k]，纯红色，线宽度为2像素，平滑
            cv::line(drawed, pts2d_map_[k], pts2d_matched_[k], cv::Scalar(0, 0, 255), line_width, cv::LINE_AA);
            // 绘制矩形，矩形的左上角坐标为pts2d_matched_[k] - rectangle_center。矩形大小为rectangle_size。纯黄色；边框宽度为2像素
            cv::rectangle(drawed, cv::Rect(pts2d_matched_[k] - rectangle_center, rectangle_size),
                          cv::Scalar(255, 255, 0), line_width);
        } else if (mappoint_type_[k] == MAPPOINT_DEPTH_ASSOCIATED) {
            cv::line(drawed, pts2d_map_[k], pts2d_matched_[k], cv::Scalar(0, 0, 255), line_width, cv::LINE_AA);
            // 如果地图点并未进行三角化，但已进行深度关联，则与上述唯一区别是矩形颜色为纯绿色
            cv::rectangle(drawed, cv::Rect(pts2d_matched_[k] - rectangle_center, rectangle_size), cv::Scalar(0, 255, 0),
                          line_width);
        } else if (mappoint_type_[k] == MAPPOINT_DEPTH_INITIALIZED) {
            cv::line(drawed, pts2d_map_[k], pts2d_matched_[k], cv::Scalar(0, 0, 255), line_width, cv::LINE_AA);
            // 如果地图点不仅没有三角化，连深度关联都没搞，只进行了深度初始化，那么与上述操作唯一区别是矩形颜色为青色
            cv::rectangle(drawed, cv::Rect(pts2d_matched_[k] - rectangle_center, rectangle_size),
                          cv::Scalar(0, 255, 255), line_width);
        }
    }

    // 跟踪上的参考帧点
    for (size_t k = 0; k < pts2d_cur_.size(); k++) {
        // 参考点和当前帧的点画到同样的图上
        cv::line(drawed, pts2d_ref_[k], pts2d_cur_[k], cv::Scalar(0, 0, 255), line_width, cv::LINE_AA);
        // 蓝色矩形框
        cv::rectangle(drawed, cv::Rect(pts2d_cur_[k] - rectangle_center, rectangle_size), cv::Scalar(255, 0, 0),
                      line_width);
    }
}