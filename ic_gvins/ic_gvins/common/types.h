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

#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

typedef struct GNSS {
    double time; // 时间戳

    Vector3d blh; // 经纬高
    Vector3d std; // 标准差，分别表示经纬高估计的精确度

    bool isyawvalid; // yaw角是否有效
    double yaw; //得到的yaw角
} GNSS;

typedef struct PVA { // position, velocity, attitude
    double time; // 时间戳
 
    Vector3d blh; // 经纬高 
    Vector3d vel; // 三维速度
    Vector3d att; // 姿态
} PVA;

typedef struct IMU {
    double time; // imu的GNSS时间
    double dt; // 与前一帧imu数据的时间差

    Vector3d dtheta; // 角度增量
    Vector3d dvel; // 速度增量

    double odovel; // 里程计速度
} IMU;

typedef struct Pose {
    Matrix3d R; // 旋转
    Vector3d t; // 平移
} Pose;

#endif // TYPES_H
