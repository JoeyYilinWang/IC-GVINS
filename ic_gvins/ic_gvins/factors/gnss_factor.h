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

#ifndef GNSS_FACTOR_H
#define GNSS_FACTOR_H

#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "common/types.h"

// ceres::SizedCostFunction是Ceres Solver库中的模板类，用于表示一个可调节大小的损失函数
// 在ceres::SizedCostFunction<3, 7>中，模板参数'3'和'7'分别表示损失函数的输出尺寸（即残差的数量）和输入尺寸（及参数快的数量）。所以这意味着产生3个残差并接收7个输入参数的损失函数。
// GnssFactor继承了该类，定义了用于GNSS定位问题的损失函数，将预测位置和实际观测进行比较以计算残差。
class GnssFactor : public ceres::SizedCostFunction<3, 7> {

public:
    explicit GnssFactor(GNSS gnss, Vector3d lever)
        : gnss_(std::move(gnss))
        , lever_(std::move(lever)) {
    }

    void updateGnssState(const GNSS &gnss) {
        gnss_ = gnss;
    }

    // 观测: 
    // gnss_blh(3d)
    // 估计:
    // 参数: p(3d)+q(4d)
    // 所以估计为p + q.toRotionMatrix() * lever_
    // 残差:
    // 估计 - 观测，为3d的向量。使用权重矩阵进行加权
    // 雅可比矩阵：
    // 残差对参数p、q的偏导数，所以形成一个3 * 7的雅可比矩阵。
    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        Vector3d p{parameters[0][0], parameters[0][1], parameters[0][2]};
        Quaterniond q{parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]};

        // 创建一个名为error的Map对象，这个对象将residuals视为一个三维的double向量，任何对error的修改都会影响到residuals
        Eigen::Map<Eigen::Matrix<double, 3, 1>> error(residuals);

        // p和q都是预测估计值，gnss_.blh为观测值
        error = p + q.toRotationMatrix() * lever_ - gnss_.blh;

        // 定义观测权重矩阵，权重为标准差的倒数
        Matrix3d sqrt_info_ = Matrix3d::Zero();
        sqrt_info_(0, 0)    = 1.0 / gnss_.std[0];
        sqrt_info_(1, 1)    = 1.0 / gnss_.std[1];
        sqrt_info_(2, 2)    = 1.0 / gnss_.std[2];

        // 残差与权重相乘，构建加权残差
        error = sqrt_info_ * error;

        if (jacobians) {
            if (jacobians[0]) {
                // 这条语句可以看出，jacobians不是一个雅克比矩阵，而是雅克比矩阵的集合，也不是以矩阵的方式保存的，而是通过数组的方式保存的
                // 通过内存映射的方式访问雅克比矩阵集合的第一个雅克比矩阵
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
                
                jacobian_pose.setZero();
                // 计算过程：
                // error(residuals) = p + q.toRotationMatrix() *lever_ - gnss_.blh
                // d(error) / d(p) = I
                jacobian_pose.block<3, 3>(0, 0) = Matrix3d::Identity();
                // 计算过程：
                // d(error) / d(q.toRotationMatrix()) = -q.toRotationMatrix * Rotation::skewSymmetric(lever_)
                jacobian_pose.block<3, 3>(0, 3) = -q.toRotationMatrix() * Rotation::skewSymmetric(lever_);
                // 这里的雅克比矩阵同样如残差一样，加一项权重
                jacobian_pose = sqrt_info_ * jacobian_pose;
            }
        }

        return true;
    }

private:
    GNSS gnss_;
    Vector3d lever_;
};

#endif // GNSS_FACTOR_H
