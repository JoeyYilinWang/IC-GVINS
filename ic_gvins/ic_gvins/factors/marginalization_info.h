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

#ifndef MARGINILAZATION_INFO_H
#define MARGINILAZATION_INFO_H

#include "factors/residual_block_info.h"

#include <memory>
#include <unordered_map>

class MarginalizationInfo {

public:
    MarginalizationInfo() = default;

    ~MarginalizationInfo() {
        for (auto &block : parameter_block_data_)
            delete[] block.second;
    }

    bool isValid() const {
        return isvalid_;
    }

    // 将要被边缘化的尺寸转化为 local_size
    static int localSize(int size) {
        return size == POSE_GLOBAL_SIZE ? POSE_LOCAL_SIZE : size;
    }
    // 将尺寸恢复到 global_size
    static int globalSize(int size) {
        return size == POSE_LOCAL_SIZE ? POSE_GLOBAL_SIZE : size;
    }

    void addResidualBlockInfo(const std::shared_ptr<ResidualBlockInfo> &blockinfo) {
        // factors_是仅包含某一类的factors还是所有类型的factors呢？需要之后对其进行论证
        factors_.push_back(blockinfo);

        const auto &parameter_blocks = blockinfo->parameterBlocks();
        const auto &block_sizes      = blockinfo->parameterBlockSizes();

        for (size_t k = 0; k < parameter_blocks.size(); k++) {
            // parameter_blocks[k]为参数块数据指针
            // reinterpret_cast<long>将此指针转化为长整型
            // parameters_ids_也是一个无序表，它的key为参数块的数据指针转成的长整型，value为参数块的id
            // 因此parameter_block_size_的key为参数块的索引值，value为参数块的大小
            parameter_block_size_[parameters_ids_[reinterpret_cast<long>(parameter_blocks[k])]] = block_sizes[k];
        }

        // blockinfo->marginalizationParametersIndex()返回被边缘化的参数块的索引集合
        // 对要被边缘化的参数快对应的索引进行遍历
        // 要注意的是这里的索引仅相对当前的因子所包含的参数块来说的，而不是所有因子的参数块
        // 所以parameters_ids_是包含所有因子对应的参数快的id。key为参数快的长整型地址，value为索引
        for (int index : blockinfo->marginalizationParametersIndex()) {
            // parameters_blocks[index]为要被边缘化的参数块数据地址
            // reinterpret_cast<long>将该地址转化为长整型
            // parameter_ids_的key为参数块地址转化为长整型的变量, value为参数块id
            // parameter_block_index_的key为参数块的id，value为参数快的在总参数块中的位置 id
            // 目前，parameter_block_index_仅保存了要被边缘化的参数块。后面在更新阶段，则会加入要被保留的参数块
            parameter_block_index_[parameters_ids_[reinterpret_cast<long>(parameter_blocks[index])]] = 0;
        }
    }

    // 更新参数块的索引，key为参数块的数据指针转成的长整型，value为参数块id
    void updateParamtersIds(const std::unordered_map<long, long> &parameters_ids) {
        parameters_ids_ = parameters_ids;
    }
        
    bool marginalization() {

        // 判断是否存在要被边缘化的参数块
        // 而且已经将要被边缘化的参数块索引放到前面，要被保留的参数块索引放到后面
        if (!updateParameterBlocksIndex()) {
            isvalid_ = false; // 则将有效性设置为false

            // 释放内存
            releaseMemory();
            
            return false;
        }

        // 计算每个残差块参数, 进行参数内存拷贝
        preMarginalization();

        // 构造增量线性方程
        constructEquation();

        // Schur消元
        schurElimination();

        // 求解线性化雅克比和残差
        linearization();

        // 释放内存
        releaseMemory();

        return true;
    }

    std::vector<double *> getParamterBlocks(std::unordered_map<long, double *> &address) {
        std::vector<double *> remained_block_addr;

        remained_block_data_.clear();
        remained_block_index_.clear();
        remained_block_size_.clear();

        for (const auto &block : parameter_block_index_) {
            // 保留的参数
            if (block.second >= marginalized_size_) {
                remained_block_data_.push_back(parameter_block_data_[block.first]);
                remained_block_size_.push_back(parameter_block_size_[block.first]);
                remained_block_index_.push_back(parameter_block_index_[block.first]);
                remained_block_addr.push_back(address[block.first]);
            }
        }

        return remained_block_addr;
    }

    const Eigen::MatrixXd &linearizedJacobians() {
        return linearized_jacobians_;
    }

    const Eigen::VectorXd &linearizedResiduals() {
        return linearized_residuals_;
    }

    int marginalizedSize() const {
        return marginalized_size_;
    }

    int remainedSize() const {
        return remained_size_;
    }

    const std::vector<int> &remainedBlockSize() {
        return remained_block_size_;
    }

    const std::vector<int> &remainedBlockIndex() {
        return remained_block_index_;
    }

    const std::vector<double *> &remainedBlockData() {
        return remained_block_data_;
    }

private:
    // 线性化
    void linearization() {
        // SVD分解求解雅克比, Hp = J^T * J = V * S^{1/2} * S^{1/2} * V^T
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(Hp_);
        Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > EPS).select(saes2.eigenvalues().array(), 0));
        Eigen::VectorXd S_inv =
            Eigen::VectorXd((saes2.eigenvalues().array() > EPS).select(saes2.eigenvalues().array().inverse(), 0));

        Eigen::VectorXd S_sqrt     = S.cwiseSqrt();
        Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

        // J0 = S^{1/2} * V^T
        linearized_jacobians_ = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
        // e0 = -{J0^T}^{-1} * bp = - S^{-1/2} * V^T * bp
        linearized_residuals_ = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * -bp_;
    }

    // Schur消元, 求解 Hp * dx_r = bp
    // 这段代码通过消去一部分变量，将一个大的线性方程组转化为一个更小的线性方程组
    void schurElimination() {
        // H0 * dx = b0
        Eigen::MatrixXd Hmm = 0.5 * (H0_.block(0, 0, marginalized_size_, marginalized_size_) +
                                     H0_.block(0, 0, marginalized_size_, marginalized_size_).transpose());
        Eigen::MatrixXd Hmr = H0_.block(0, marginalized_size_, marginalized_size_, remained_size_);
        Eigen::MatrixXd Hrm = H0_.block(marginalized_size_, 0, remained_size_, marginalized_size_);
        Eigen::MatrixXd Hrr = H0_.block(marginalized_size_, marginalized_size_, remained_size_, remained_size_);
        Eigen::VectorXd bmm = b0_.segment(0, marginalized_size_);
        Eigen::VectorXd brr = b0_.segment(marginalized_size_, remained_size_);

        // SVD分解Amm求逆
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Hmm);
        Eigen::MatrixXd Hmm_inv =
            saes.eigenvectors() *
            Eigen::VectorXd((saes.eigenvalues().array() > EPS).select(saes.eigenvalues().array().inverse(), 0))
                .asDiagonal() *
            saes.eigenvectors().transpose();

        // Hp = Hrr - Hrm * Hmm^-1 * Hmr
        Hp_ = Hrr - Hrm * Hmm_inv * Hmr;
        // bp = br - Hrm * Hmm^-1 * bm
        bp_ = brr - Hrm * Hmm_inv * bmm;
    }

    // 构造增量方程 H * dx = b, 计算 H 和 b
    // 该方程实际上是由牛顿法推导出来的，H对应Hessian矩阵，dx为增量
    void constructEquation() {
        H0_ = Eigen::MatrixXd::Zero(local_size_, local_size_);
        b0_ = Eigen::VectorXd::Zero(local_size_);

        // 遍历每个因子
        for (const auto &factor : factors_) {
            // 遍历因子中每个参数块
            for (size_t i = 0; i < factor->parameterBlocks().size(); i++) {
                // row0为当前处理的参数块的id对应的参数块实际位置(index)
                int row0 =
                    parameter_block_index_[parameters_ids_[reinterpret_cast<long>(factor->parameterBlocks()[i])]];
                // rows为当前处理的参数块的尺寸
                // 比如当前处理的参数块为imu2nav的位姿，则rows为7
                int rows = parameter_block_size_[parameters_ids_[reinterpret_cast<long>(factor->parameterBlocks()[i])]];
                // 将rows的size设置为local_size，6
                rows     = localSize(rows);
                
                // Eigen::MatrixXd::leftCols(rows)取rows列
                Eigen::MatrixXd jacobian_i = factor->jacobians()[i].leftCols(rows);
                // 从该因子的第i个参数块开始
                for (size_t j = i; j < factor->parameterBlocks().size(); ++j) {
                    // col0为针对当前因子的第j个参数块的id对应的参数块实际位置(index)
                    int col0 =
                        parameter_block_index_[parameters_ids_[reinterpret_cast<long>(factor->parameterBlocks()[j])]];
                    // cols为针对当前因子的第j个参数块的id对应的参数块的尺寸
                    int cols =
                        parameter_block_size_[parameters_ids_[reinterpret_cast<long>(factor->parameterBlocks()[j])]];
                    // 同样也需要将cols的size设置为local_size, 6
                    cols = localSize(cols);

                    // 获取当前因子的第j个参数块对应的雅克比矩阵第前cols列
                    Eigen::MatrixXd jacobian_j = factor->jacobians()[j].leftCols(cols);

                    // H = J^T * J
                    if (i == j) {
                        // Hmm, Hrr
                        // 选取H0_从row0，col0开始的，高度为rows，宽度为cols的矩阵
                        // 构造对应的近似hessian矩阵
                        H0_.block(row0, col0, rows, cols) += jacobian_i.transpose() * jacobian_j;
                    } else {
                        // Hmr, Hrm = Hmr^T
                        // 如果jacobian_i与jacobian_j不相等
                        H0_.block(row0, col0, rows, cols) += jacobian_i.transpose() * jacobian_j;
                        H0_.block(col0, row0, cols, rows) = H0_.block(row0, col0, rows, cols).transpose();
                    }
                }
                // b = - J^T * e
                b0_.segment(row0, rows) -= jacobian_i.transpose() * factor->residuals();
            }
        }
    }

    bool updateParameterBlocksIndex() {
        int index = 0;
        // 只有被边缘化的参数预先加入了表
        for (auto &block : parameter_block_index_) {
            // 对每个参数块的起始位置进行更新，因此parameter_block_index_的key为参数块整个的id（表示第几个参数块），而value为参数块起始位置的索引id（表示位置）
            block.second = index; 
            // block.first为参数块的id
            // parameter_block_size为存放参数块大小的无序map，key为参数块的索引值，value为大小。因此parameter_block_size_[block.first]获取到对应索引参数块的大小
            // localSize()将global_size转化为local_size
            index += localSize(parameter_block_size_[block.first]);
        }
        marginalized_size_ = index;

        // 加入保留的参数, 分配索引
        for (const auto &block : parameter_block_size_) {
            if (parameter_block_index_.find(block.first) == parameter_block_index_.end()) {
                parameter_block_index_[block.first] = index;
                index += localSize(block.second);
            }
        }
        // remained_size_表示剩下参数块的尺寸(要注意的是，此时的要被边缘化的参数块和被保留的参数块的尺寸都转化为了local_size)
        remained_size_ = index - marginalized_size_;

        // 此时的local_size_指向了所有参数块local_size尺寸的总和
        local_size_ = index;

        // 如果存在要被边缘化的参数块则返回true
        return marginalized_size_ > 0;
    }

    // 边缘化预处理, 评估每个因子
    void preMarginalization() {
        for (const auto &factor : factors_) {
            factor->Evaluate();
            
            std::vector<int> block_sizes = factor->parameterBlockSizes();
            // 对每个因子的每个参数块进行处理
            for (size_t k = 0; k < block_sizes.size(); k++) {
                // 获取每个参数块的id
                long id  = parameters_ids_[reinterpret_cast<long>(factor->parameterBlocks()[k])];
                // 获取每个参数块的尺寸
                int size = block_sizes[k];

                // 拷贝参数块数据
                if (parameter_block_data_.find(id) == parameter_block_data_.end()) {
                    auto *data = new double[size];
                    memcpy(data, factor->parameterBlocks()[k], sizeof(double) * size);
                    parameter_block_data_[id] = data;
                }
            }
        }
    }

    void releaseMemory() {
        // 释放因子所占有的内存, 尤其是边缘化因子及其占有的边缘化信息数据结构
        factors_.clear();
    }

private:
    // 增量线性方程参数
    Eigen::MatrixXd H0_, Hp_;
    Eigen::VectorXd b0_, bp_;

    // 储存参数块的id，key为参数块数据地址，value为参数块id
    std::unordered_map<long, long> parameters_ids_;

    // 存放参数块的尺寸，key为参数块id，value为参数块的尺寸
    std::unordered_map<long, int> parameter_block_size_;
    // 存放参数块的位置index，key参数块的id，value为参数块的位置index
    std::unordered_map<long, int> parameter_block_index_;
    // 存放参数块数据指针，key为参数块的id，value为参数快的数据指针
    std::unordered_map<long, double *> parameter_block_data_;

    // 保留的参数
    // 下面三个容器中的元素应该是一一对应的
    std::vector<int> remained_block_size_;  // 剩下参数块的大小
    std::vector<int> remained_block_index_; // 剩下参数块的索引
    std::vector<double *> remained_block_data_; // 剩下参数块的数据指针

    // local size in total
    int marginalized_size_{0};
    int remained_size_{0};
    int local_size_{0}; // 要被边缘化的参数块与要被保留的参数块的local_size之和

    // 储存多个因子的向量
    std::vector<std::shared_ptr<ResidualBlockInfo>> factors_;

    const double EPS = 1e-8;

    // 边缘化求解的残差和雅克比
    Eigen::MatrixXd linearized_jacobians_;
    Eigen::VectorXd linearized_residuals_;

    // 若无待边缘化参数, 则无效
    bool isvalid_{true};
};

#endif // MARGINILAZATION_INFO_H
