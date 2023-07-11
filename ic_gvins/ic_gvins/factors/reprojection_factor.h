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

#ifndef REPROJECTION_FACTOR_H
#define REPROJECTION_FACTOR_H

#include <ceres/ceres.h>

#include <Eigen/Geometry>

#include "common/rotation.h"

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

// ceres::SizedCostFunction<>是ceres::CostFunction的派生类
// 第一个参数2表示残差数量为2，后面的数字表示参数块的大小。
// 也就是说该损失函数的残差数量为2，有5个参数块，大小分别为7，7，7，1，1
class ReprojectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1, 1> {

public:
    ReprojectionFactor() = delete; // 删除默认构造函数

    // 标准差为归一化相机下的重投影误差观测, pixel / f
    ReprojectionFactor(Vector3d pts0, Vector3d pts1, Vector3d vel0, Vector3d vel1, double td0, double td1, double std)
        : pts0_(std::move(pts0))
        , pts1_(std::move(pts1))
        , vel0_(std::move(vel0))
        , vel1_(std::move(vel1))
        , td0_(td0)
        , td1_(td1) {

        sqrt_info_.setZero();
        // sqrt_info为观测权重的平方根
        // 在最小二乘问题或者SLAM问题中，通常要考虑观测的置信度或者噪声等级，这通常由权重矩阵来表示
        // 权重矩阵通常是协方差矩阵的逆，如果噪声是高斯分布的，协方差矩阵能够很好的描述噪声特性。
        // 而在最小二乘问题中，通常处理的是权重矩阵的平方根，因为这样可以简化求解。
        // 对于相机的观测模型，考虑到像素噪声是各项异性的，所以sqrt_info_是一个2*2的矩阵
        sqrt_info_(0, 0) = 1.0 / std;
        sqrt_info_(1, 1) = 1.0 / std;
    }

    // 观测:
    // 第二帧的相同特征点归一化坐标（pts_1_td.head(2)）
    // 估计:
    // 根据输入参数对参考帧(第一帧)的特征点向第二帧进行重投影的特征点归一化坐标(pts_1 / d1.head(2))
    // 重投影过程:
    // 0. 根据估计时延与给定时延的差值，计算出估计时延下的pts_0_td(用于计算第二帧的重投影)、pts_1_td(第二帧的观测)
    // 1. 根据参考帧的逆深度恢复特征点在参考帧对应的相机坐标系坐标   id0                      pts_0_td --> pts_c_0
    // 2. 根据camera与imu的位姿关系恢复到参考帧对应的imu坐标系坐标  tic qic                 pts_c_0  --> pts_b_0
    // 3. 根据参考帧imu坐标系与导航坐标系之间的位姿关系，恢复到导航坐标系下的坐标 p0 q0         pts_b_0  --> pts_n    
    // 4. 根据第二帧imu坐标系与导航坐标系之间的位姿关系，转到第二帧imu坐标系坐标 p1 q1          pts_n   --> pts_b_1
    // 5. 根据camera与imu的位姿关系，转到第二帧camera坐标系坐标 tic qic                    pts_b_1  --> pts_c_1(pts_1)
    // 残差:
    // 估计 - 观测， 并使用权重矩阵加权
    // 雅克比矩阵:
    // 是一个雅克比矩阵的集合，该集合有5个雅克比矩阵，每个雅克比矩阵都是残差对参数的偏导。
    // 维度分别为2*7, 2*7, 2*7, 2*1, 2*1。只不过每个雅克比矩阵都是由一维double矩阵储存的
    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        // 参考帧0对应的imu2nav位姿，参数块大小为7
        Eigen::Vector3d p0(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond q0(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
        // 参考帧1对应的imu2nav位姿，参数块大小为7
        Eigen::Vector3d p1(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond q1(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        // 相机外参，参数块大小为7
        // camera2imu
        Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

        // 逆深度，参数快大小为1    
        // 逆深度是一种在计算机视觉和机器人学中经常用到的表达深度的方式，它的值等于1除以实际深度。
        // 逆深度参数化在在一些计算机视觉中非常有用，特别在处理遥远物体（深度很大，逆深度接近于0）和近距离物体（深度较小，逆深度较大）时。
        // 逆深度的主要优点是它对于所有深度值（从很近到很远）都提供了相对均匀的参数范围。
        double id0 = parameters[3][0];

        // 时间延时，参数块大小为1
        double td = parameters[4][0];

        // td-td0_为估计的时延与给定时延的差值，表达了图像帧0与imu的估计时延和给定的时延的差异
        // 时延估计的差值 * vel0_为估计时延与给定时延的归一化像素差异
        // pts_0_td为估计时延下的特征点归一化像素值
        Eigen::Vector3d pts_0_td = pts0_ - (td - td0_) * vel0_;
        // pts_1_td为估计时延下相同特征点归一化像素值
        Eigen::Vector3d pts_1_td = pts1_ - (td - td1_) * vel1_;

        // 根据图像帧0的特征点的逆深度，恢复特征点相机坐标系下的三维坐标
        Eigen::Vector3d pts_c_0 = pts_0_td / id0;
        // 根据前面给定的body坐标系(imu)和camera坐标系坐标的转换关系，得到body坐标系下的坐标
        Eigen::Vector3d pts_b_0 = qic * pts_c_0 + tic;
        // 将图像帧0对应的imu坐标系下的坐标转换为导航坐标下坐标
        Eigen::Vector3d pts_n   = q0 * pts_b_0 + p0;
        // 根据得到的导航坐标系以及图像帧1对应的imu2nav变换，得到图像帧1对应的imu坐标系下的坐标
        Eigen::Vector3d pts_b_1 = q1.inverse() * (pts_n - p1);
        // 根据cam2imu进而得到图像帧1对应的相机坐标系下的三维坐标
        Eigen::Vector3d pts_1   = qic.inverse() * (pts_b_1 - tic);
        // 得到图像帧1特征点的深度
        double d1 = pts_1.z();

        // 残差, 没有考虑参考帧的残差
        Eigen::Map<Eigen::Vector2d> residual(residuals);
        // 计算归一化像素残差，后一项为观测，前一项为估计
        // 后一项是根据给定的pts_1的值，估计的td，给定的vel1，估计的td计算得到的，这里只有td是需要估计的。可作为观测
        // 而前一项是根据给定的pts0_，给定的td_0，给定的vel0, 估计的td，估计的逆深度id0，估计的qic和tic，估计的q0、p0，估计的q1、p1计算得到的。作为估计
        residual = (pts_1 / d1).head(2) - pts_1_td.head(2);
        // 给残差加上权重，保证算法的鲁棒性。
        residual = sqrt_info_ * residual; 

        if (jacobians) {
            Eigen::Matrix3d cb0n = q0.toRotationMatrix(); // imu0->nav
            Eigen::Matrix3d cnb1 = q1.toRotationMatrix().transpose(); // nav->imu1
            Eigen::Matrix3d cbc  = qic.toRotationMatrix().transpose(); // imu->cam
            Eigen::Matrix<double, 2, 3> reduce;
            reduce << 1.0 / d1, 0, -pts_1(0) / (d1 * d1), 0, 1.0 / d1, -pts_1(1) / (d1 * d1);

            reduce = sqrt_info_ * reduce;

            if (jacobians[0]) {
                // 优化变量imu2nav位姿对于观测值（维度为2）的雅克比矩阵
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                
                Eigen::Matrix<double, 3, 6> jaco_i;
                // 计算过程：
                // pts_n  = q0 * pts_b_0 + p0
                // pts_b_1 = q1.inverse() * (pts_n - p1)
                // pts_c_1 = qic.inverse() * (pts_b_1 - tic)
                //         = qic.inverse() * (q1.inverse() * (pts_n - p1) - tic)
                //         = cbc * (cnb1 * (q0 * pts_b_0 + p0 - p1) - tic)
                //         = cbc * (cnb1 * q0 * pts_b_0 + cnb1 * p0 - cnb1 * p1 - tic)
                //         = cbc * cnb1 * q0 * pts_b_0 + cbc * cnb1 * p0 - cbc * cnb1 * p1 - cbc * tic
                // d(pts_c_1 - pts_c_td)/d(p0)，其中pts_c_td为常数，所以等于d(pts_c_1) / d(p0) = cbc * cnb1 
                jaco_i.leftCols<3>()  = cbc * cnb1; 

                // 同理，d(pts_c_1) / d(q0) = d(cbc * cnb1 * q0 * pts_b_0 + ...) / d(q0) = cbc * cnb1 * d(q0 * pts_b_0) / d(q0)
                // d(q0.toRotationMatrix() * pts_b_0) / d(q0.toRotationMatrix())表达的是旋转后的向量对旋转本身的导数
                jaco_i.rightCols<3>() = -cbc * cnb1 * cb0n * Rotation::skewSymmetric(pts_b_0);
                // 将3D误差转化为2D的误差，reduce矩阵是一个投影的操作
                jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
                // 四元数有一个归一化的操作，因此最后一项可以不管设置为0
                jacobian_pose_i.rightCols<1>().setZero();
            }
            
            // 同样处理方式
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

                Eigen::Matrix<double, 3, 6> jaco_j;
                // 计算过程
                // pts_c_1 = cbc * cnb1 * q0 * pts_b_0 + cbc * cnb1 * p0 - cbc * cnb1 * p1 - cbc * tic
                // d(pts_c_1 - pts_c_td) / d(p1) = -cbc * cnb1
                jaco_j.leftCols<3>()  = -cbc * cnb1; 

                jaco_j.rightCols<3>() = cbc * Rotation::skewSymmetric(pts_b_1);

                jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
                jacobian_pose_j.rightCols<1>().setZero();
            }

            if (jacobians[2]) {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);

                Eigen::Matrix<double, 3, 6> jaco_ex;
                
                // 计算过程
                // 
                jaco_ex.leftCols<3>() = cbc * (cnb1 * cb0n - Eigen::Matrix3d::Identity());
                Eigen::Matrix3d tmp_r = cbc * cnb1 * cb0n * cbc.transpose();

                jaco_ex.rightCols<3>() = -tmp_r * Rotation::skewSymmetric(pts_c_0) +
                                         Rotation::skewSymmetric(tmp_r * pts_c_0) +
                                         Rotation::skewSymmetric(cbc * (cnb1 * (cb0n * tic + p0 - p1) - tic));

                jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
                jacobian_ex_pose.rightCols<1>().setZero();
            }

            if (jacobians[3]) {
                Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[3]);
                jacobian_feature = -reduce * cbc * cnb1 * cb0n * cbc.transpose() * pts_0_td / (id0 * id0);
            }

            if (jacobians[4]) {
                Eigen::Map<Eigen::Vector2d> jacobian_td(jacobians[4]);
                jacobian_td = -reduce * cbc * cnb1 * cb0n * cbc.transpose() * vel0_ / id0 + sqrt_info_ * vel1_.head(2);
            }
        }

        return true;
    }

private:
    // 归一化相机坐标系下的坐标
    Vector3d pts0_; // 特征点在图像帧0的归一化坐标
    Vector3d pts1_; // 相同特征点在图像帧1的归一化坐标

    Vector3d vel0_, vel1_;  // 分别表示同一特征点在图像帧0和图像帧1的归一化速度
    double td0_, td1_; // 分别表示图像帧0、图像帧1与imu的延时。但这两个值不一定准确的，由于硬件设备特性、环境因素等原因，这两个值还会有变化
 
    Eigen::Matrix2d sqrt_info_; // 
};

#endif // REPROJECTION_FACTOR_H
