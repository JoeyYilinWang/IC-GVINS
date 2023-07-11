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

#ifndef RESIDUAL_BLOCK_INFO_H
#define RESIDUAL_BLOCK_INFO_H

#define POSE_LOCAL_SIZE 6
#define POSE_GLOBAL_SIZE 7

#include <ceres/ceres.h>
#include <memory>

// 残差块信息
class ResidualBlockInfo {

public:
    ResidualBlockInfo(std::shared_ptr<ceres::CostFunction> cost_function,
                      std::shared_ptr<ceres::LossFunction> loss_function, std::vector<double *> parameter_blocks,
                      std::vector<int> marg_para_index)
        : cost_function_(std::move(cost_function))
        , loss_function_(std::move(loss_function))
        , parameter_blocks_(std::move(parameter_blocks))
        , marg_para_index_(std::move(marg_para_index)) {
    }

    void Evaluate() {
        // 每个观测值都产生一个误差，所以num_residuals()的返回值基本上就是观测值的数量
        residuals_.resize(cost_function_->num_residuals());

        // 容器，储存的是块的尺寸
        // parameter_block_sizes()返回一个包含每个参数块大小的向量。这个向量的长度等于参数块的数量，向量中的每个元素代表相应参数块中的参数个数
        std::vector<int> block_sizes = cost_function_->parameter_block_sizes();

        // 根据参数块的数量定义原始雅克比矩阵的数量
        auto raw_jacobians           = new double *[block_sizes.size()];
        jacobians_.resize(block_sizes.size()); // 雅克比矩阵集合中包含的雅克比矩阵个数也与参数块的个数相同

        // 对参数块大小进行遍历
        for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
            // 对雅克比矩阵进行尺寸重构，行数为残差的数量，列数为参数块的大小
            jacobians_[i].resize(cost_function_->num_residuals(), block_sizes[i]);
            // raw_jacobians保存雅克比矩阵的数据指针
            raw_jacobians[i] = jacobians_[i].data();
        }

        // parameter_blocks_.data()为输入
        // residuals_.data()为输出
        // raw_jacobians也为输出
        cost_function_->Evaluate(parameter_blocks_.data(), residuals_.data(), raw_jacobians);
        
        // 释放raw_jacobians指向的内存地址，但要注意的是jacobians_还保有着数据
        delete[] raw_jacobians;

        // 在本工程中，loss_function设置为nullptr，因此不执行下面的代码
        // 但该部分的目的是实现在含有噪声或异常值的情况下，对非线性优化问题中的残差和雅可比矩阵进行调整，提高求解结果的鲁棒性。
        if (loss_function_) {
            // 鲁棒核函数调整, 参考ceres/internal/ceres/corrector.cc
            double residual_scaling, alpha_sq_norm;

            double sq_norm, rho[3];
            
            sq_norm = residuals_.squaredNorm();
            loss_function_->Evaluate(sq_norm, rho);

            double sqrt_rho1 = sqrt(rho[1]);

            if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
                residual_scaling = sqrt_rho1;
                alpha_sq_norm    = 0.0;
            } else {
                // 0.5 *  alpha^2 - alpha - rho'' / rho' *  z'z = 0
                const double D     = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
                const double alpha = 1.0 - sqrt(D);
                residual_scaling   = sqrt_rho1 / (1 - alpha);
                alpha_sq_norm      = alpha / sq_norm;
            }

            for (size_t i = 0; i < parameter_blocks_.size(); i++) {
                // J = sqrt_rho1 * (J - alpha_sq_norm * r* (r.transpose() * J))
                jacobians_[i] =
                    sqrt_rho1 * (jacobians_[i] - alpha_sq_norm * residuals_ * (residuals_.transpose() * jacobians_[i]));
            }
            residuals_ *= residual_scaling;
        }
    }

    // 返回雅克比矩阵的集合
    const std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> &jacobians() {
        return jacobians_;
    }

    // 返回保存参数快大小的集合
    const std::vector<int> &parameterBlockSizes() {
        return cost_function_->parameter_block_sizes();
    }

    // 返回参数块的集合
    const std::vector<double *> &parameterBlocks() {
        return parameter_blocks_;
    }

    // 返回残差
    const Eigen::VectorXd &residuals() {
        return residuals_;
    }

    // 返回边缘化参数块的索引集合
    const std::vector<int> &marginalizationParametersIndex() {
        return marg_para_index_;
    }

private:
    // ceres::CostFunction是ceresSovler库的一个重要组成部分。用于解决复杂的非线性最小二乘问题和非线性优化问题。
    std::shared_ptr<ceres::CostFunction> cost_function_;
    // 上述的costfunction是希望最小化的原始问题，而下面的lossfunction用于调整costfunction的贡献，以减少离群值的影响。lossfunction可理解为一种权重函数，根据残差的大小改变每个数据点在总体成本中的权重
    std::shared_ptr<ceres::LossFunction> loss_function_;

    std::vector<double *> parameter_blocks_; // 容器，包含参数快的数据指针
 
    std::vector<int> marg_para_index_; // 容器，包含边缘化参数块索引

    // 雅克比矩阵集合，矩阵的行列在编译时是未知的，因此使用Eigen::Dynamic关键字。同时也是行优先存储
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians_;
    Eigen::VectorXd residuals_; // 定义残差向量
};

#endif // RESIDUAL_BLOCK_INFO_H
