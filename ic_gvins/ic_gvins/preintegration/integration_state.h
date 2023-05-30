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

#ifndef INTEGRATION_DEFINE_H
#define INTEGRATION_DEFINE_H

#include <Eigen/Geometry>
#include <vector>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using std::vector;

typedef struct IntegrationState {
    double time;

    Vector3d p{0, 0, 0}; // position 
    Quaterniond q{0, 0, 0, 0}; // rotation in quaternions
    Vector3d v{0, 0, 0}; // velocity

    Vector3d bg{0, 0, 0}; // gyro bias
    Vector3d ba{0, 0, 0}; // acc bias
    
    Vector3d s{0, 0, 0}; // 不知道是啥
    double sodo{0}; // odometry scale factor
    Vector2d avb{0, 0}; // What is it?

    Vector3d sg{0, 0, 0}; // gyro scale factor
    Vector3d sa{0, 0, 0}; // acc scale factor
} IntegrationState;

typedef struct IntegrationStateData {
    double time;

    double pose[7]; // pose : 3 + 4 = 7

    // mix parameters
    // vel + bias : 3 + 6 = 9
    // vel + bias + sodo : 3 + 6 + 1 = 10
    // vel + bias + sodo + abv : 3 + 6 + 1 + 2 = 12
    // vel + bias + scale : 3 + 6 + 6 = 15
    // vel + bias + sodo + scale : 3 + 6 + 1 + 6 = 16
    // vel + bias + sodo + scale + abv : 3 + 6 + 1 + 6 + 2 = 18
    double mix[18];  // 这里没有包含IntegrationState中的成员s 
} IntegrationStateData;

typedef struct IntegrationParameters {
    // IMU噪声为白噪声, 积分为随机游走, 即VRW和ARW
    // 零偏和比例因子建模为一阶高斯马尔卡夫过程, 参数为标准差及相关时间

    double acc_vrw;       // 速度随机游走, m / s^1.5
    double gyr_arw;       // 角度随机游走, rad / s^0.5
    double gyr_bias_std;  // 陀螺零偏标准差, rad / s
    double acc_bias_std;  // 加表零偏标准差, m / s^2
    double gyr_scale_std; // 陀螺比例因子标准差
    double acc_scale_std; // 加表比例因子标准差
    double corr_time;     // 相关时间, s

    double gravity; // 当地重力, m / s^2

    Vector3d odo_std; // 里程计白噪声, m/s
    double odo_srw;   // 里程计比例因子随机游走, PPM / sqrt(Hz)

    Vector3d abv;  // b系与v系的安装角, rad
    Vector3d lodo; // b系下的里程计杆臂, m

    Vector3d station; // 站心坐标系原点
} IntegrationParameters;

typedef struct IntegrationConfiguration {
    bool isuseodo; // 是否使用里程计
    bool iswithscale; // unknown
    bool iswithearth; // 是否伴随地球

    Vector3d origin; // 站心原点坐标，本质上是ENU坐标系原点在世界坐标系下坐标，是不断移动的
    Vector3d gravity; // 重力
    Vector3d iewn; // 地球自转角速度投影到n系
}  IntegrationConfiguration;

#endif // INTEGRATION_DEFINE_H
