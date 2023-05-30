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

#include "misc.h"

#include "common/angle.h"
#include "common/earth.h"
#include "common/logging.h"
#include "common/rotation.h"

// 根据时间得到IMU数据窗口中的索引值，该索引对应的IMU时间比提供的时间稍大。
size_t MISC::getInsWindowIndex(const std::deque<std::pair<IMU, IntegrationState>> &window, double time) {
    // 返回时间大于输入的第一个索引

    if (window.empty() || (window.front().first.time > time) || (window.back().first.time <= time)) {
        return 0;
    }

    size_t index = 0;
    size_t sta = 0, end = window.size();

    // 二分法搜索
    int counts = 0;
    while (true) {
        auto mid = (sta + end) / 2;

        auto first  = window[mid - 1].first.time;
        auto second = window[mid].first.time;

        if ((first <= time) && (time < second)) {
            index = mid;
            break;
        } else if (first > time) {
            end = mid;
        } else if (second <= time) {
            sta = mid;
        }

        // 65536 / 200 = 327.68
        if (counts++ > 15) {
            LOGE << "Failed to get ins window index at " << Logging::doubleData(time);
            break;
        }
    }

    return index;
}


bool MISC::getCameraPoseFromInsWindow(const std::deque<std::pair<IMU, IntegrationState>> &window, const Pose &pose_b_c,
                                      double time, Pose &pose) {
    // 根据时间输入获得对应时间的IMU数据索引
    // 该索引对应的时间大于给定时间且小于下一条索引时间
    size_t index = getInsWindowIndex(window, time);
    IntegrationState state;
    if (index > 0) {
        auto state0 = window[index - 1].second;
        auto state1 = window[index].second; 

        // state0 < time < state1
        // 根据前后两帧的积分状态进行插值
        statePoseInterpolation(state0, state1, time, state);
        // pose_b_c为body2cam的旋转矩阵
        pose = stateToCameraPose(state, pose_b_c);
        return true;
    } else { // index == 0表示没更新index，进而表示给定时间在整体IMU序列的左边或者右边。直接使用IMU序列最新数据进行处理
        pose = stateToCameraPose(window.back().second, pose_b_c);
        return false;
    }
}

// 根据midtime的前后两状态的时间进行状态的线性插值
void MISC::statePoseInterpolation(const IntegrationState &state0, const IntegrationState &state1, double midtime,
                                  IntegrationState &state) {
    

    Vector3d dp    = state1.p - state0.p;
    Quaterniond dq = state1.q.inverse() * state0.q;
    Vector3d rvec  = Rotation::quaternion2vector(dq);

    double scale = (midtime - state0.time) / (state1.time - state0.time);
    rvec         = rvec * scale;
    dq           = Rotation::rotvec2quaternion(rvec);

    state.p = state0.p + dp * scale;
    state.q = state0.q * dq.inverse();
    state.q.normalize();
}

// 将imu状态中的位置和姿态转化为Camera的位置和姿态
// 输入：imu估计的状态、body2camera的转化矩阵
// 输出：camera的状态
Pose MISC::stateToCameraPose(const IntegrationState &state, const Pose &pose_b_c) {
    Pose pose;

    pose.t = state.p + state.q.toRotationMatrix() * pose_b_c.t;
    pose.R = state.q.toRotationMatrix() * pose_b_c.R;
    return pose;
}

// 将分离的位姿合并为一个大的变换矩阵
Eigen::Matrix4d MISC::pose2Twc(const Pose &pose) {
    Eigen::Matrix4d Twc = Eigen::Matrix4d ::Zero();
    Twc(3, 3)           = 1;

    Twc.block<3, 3>(0, 0) = pose.R;
    Twc.block<3, 1>(0, 3) = pose.t;

    return Twc;
}

// 判断两个时间点是否属于同一个时间节点
bool MISC::isTheSameTimeNode(double time0, double time1, double interval) {
    return fabs(time0 - time1) < interval;
}

// 根据给定的时间和时间间隔阈值，得到时间序列中与给定时间最接近时间的序列索引
size_t MISC::getStateDataIndex(const std::deque<double> &timelist, double time, double interval) {
    size_t index = 0;

    size_t sta = 0, end = timelist.size();
    int counts = 0;
    while (true) {
        auto mid      = (sta + end) / 2;
        auto mid_time = timelist[mid];

        if (isTheSameTimeNode(mid_time, time, interval)) {
            index = mid;
            break;
        } else if (mid_time > time) {
            end = mid;
        } else if (mid_time < time) {
            sta = mid;
        }

        if (counts++ > 10) {
            LOGW << "Failed to get state data index at " << Logging::doubleData(time);
            break;
        }
    }

    return index;
}

// 惯导的机械编排算法
// config为配置参数
// imu_pre为前一时刻imu数据，imu_cur为当前时刻的imu数据
// state为输出结果，对应imu_cur的时间
void MISC::insMechanization(const IntegrationConfiguration &config, const IMU &imu_pre, const IMU &imu_cur,
                            IntegrationState &state) {

    // IMU零偏补偿
    IMU imu_cur2, imu_pre2;
    imu_cur2.dtheta = imu_cur.dtheta - imu_cur.dt * state.bg;
    imu_cur2.dvel   = imu_cur.dvel - imu_cur.dt * state.ba;
    imu_pre2.dtheta = imu_pre.dtheta - imu_pre.dt * state.bg;
    imu_pre2.dvel   = imu_pre.dvel - imu_pre.dt * state.ba;

    // IMU比例因子误差补偿
    if (config.iswithscale) {
        for (int k = 0; k < 3; k++) {
            imu_cur2.dtheta[k] *= (1.0 - state.sg[k]);
            imu_cur2.dvel[k] *= (1.0 - state.sa[k]);
            imu_pre2.dtheta[k] *= (1.0 - state.sg[k]);
            imu_pre2.dvel[k] *= (1.0 - state.sa[k]);
        }
    }

    double dt  = imu_cur.dt;
    state.time = imu_cur.time;

    // 双子样
    // b系比力积分项没使用补偿过的imu数据
    Vector3d dvfb = imu_cur.dvel + 0.5 * imu_cur.dtheta.cross(imu_cur.dvel) +
                    1.0 / 12.0 * (imu_pre.dtheta.cross(imu_cur.dvel) + imu_pre.dvel.cross(imu_cur.dtheta));
    // imu前后两帧的等效旋转矢量
    Vector3d dtheta = imu_cur2.dtheta + 1.0 / 12.0 * imu_pre2.dtheta.cross(imu_cur2.dtheta);

    // 速度变化量
    Vector3d dvel;

    if (config.iswithearth) {
        // Part5 (49b)
        Vector3d dv_cor_g = (config.gravity - 2.0 * config.iewn.cross(state.v)) * dt;

        // 地球自转补偿项
        Vector3d dnn    = -config.iewn * dt;

        // n(k)到n(k-1)的旋转四元数
        Quaterniond qnn = Rotation::rotvec2quaternion(dnn);

        // Part5 (49a)
        // dvel为导航坐标系下的速度变化量
        // state.q.toRotationMatrix()对应n(k-1)到b(k-1)的旋转矩阵
        // dvfb为在比力作用下速度增量由b(k-1)到b(k)的投影
        // dv_cor_g为重力/柯氏补偿
        dvel = 0.5 * (Matrix3d::Identity() + qnn.toRotationMatrix()) * state.q.toRotationMatrix() * dvfb + dv_cor_g;

        
        state.q = qnn * state.q * Rotation::rotvec2quaternion(dtheta);
        state.q.normalize();
    } else {
        
        dvel = state.q.toRotationMatrix() * dvfb + config.gravity * dt;

        state.q *= Rotation::rotvec2quaternion(dtheta);
        state.q.normalize();
    }

    // 前后历元平均速度计算位置
    state.p += dt * state.v + 0.5 * dt * dvel;
    // 最后更新速度
    state.v += dvel;
}

void MISC::redoInsMechanization(const IntegrationConfiguration &config, const IntegrationState &updated_state,
                                size_t reserved_ins_num, std::deque<std::pair<IMU, IntegrationState>> &ins_windows) {
    // 最新更新过的位姿
    auto state   = updated_state;
    size_t index = getInsWindowIndex(ins_windows, state.time);
    
    if (index == 0) {
        LOGE << "Failed to get right index in mechanization";
        return;
    }

    double dt = ins_windows.back().first.time - state.time;
    LOGI << "Redo INS mechanization at " << Logging::doubleData(state.time) << " to "
         << Logging::doubleData(ins_windows.back().first.time) << " is " << dt << " s";
    if (dt > 1.0) {
        LOGW << "Do INS mechanization with a long time " << dt << " s";
    }

    IMU imu0 = ins_windows[index - 1].first;
    IMU imu1 = ins_windows[index].first;
    IMU imu;
    
    int isneed = isNeedInterpolation(imu0, imu1, state.time);

    // 如果靠近imu0，则需要按照该时刻状态参数重新计算imu1时刻的状态
    if (isneed == -1) {
        insMechanization(config, imu0, imu1, state);
        ins_windows[index].second = state;
    // 如果靠近imu1，则仅需要将imu1时刻的状态进行赋值即可
    } else if (isneed == 1) {
        // 当前时刻状态即为状态量
        state.time                = imu1.time;
        ins_windows[index].second = state;

    // 如果两者都不靠近，则：
    } else if (isneed == 2) {
        imuInterpolation(imu1, imu, imu1, state.time); // 首先进行插值
        insMechanization(config, imu0, imu, state); // 然后依次进行机械编排
        insMechanization(config, imu, imu1, state);
        ins_windows[index].second = state; // 最终得到更新后的状态
    }
    imu0 = imu1;

    // 依次更新index后面的状态
    for (size_t k = index + 1; k < ins_windows.size(); k++) {
        imu1 = ins_windows[k].first;
        insMechanization(config, imu0, imu1, state);
        ins_windows[k].second = state;

        imu0 = imu1;
    }

    // 移除过期的IMU历元
    if (index < reserved_ins_num) {
        return;
    }
    size_t counts = index - reserved_ins_num;
    for (size_t k = 0; k < counts; k++) {
        ins_windows.pop_front();
    }
}


int MISC::isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid) {
    double time = mid;

    // 靠近整秒
    if (imu0.time < time && imu1.time > time) {
        double dt = time - imu0.time;

        // 前一个历元接近
        if (dt < MINIMUM_TIME_INTERVAL) {
            return -1;
        }

        // 后一个历元接近
        dt = imu1.time - time;
        if (dt < MINIMUM_TIME_INTERVAL) {
            return 1;
        }

        // 需内插
        return 2;
    }

    return 0;
}

// 根据给定中间时刻对imu数据进行插值
// imu01为已知imu数据。
// mid中间时间，该时间在imu01之前，但在imu01的前一帧imu数据之后
// imu00为mid时刻对应的imu数据
// imu11为imu01.time时刻的数据，但dt为imu11
void MISC::imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid) {
    double time = mid;

    double scale = (imu01.time - time) / imu01.dt;
    IMU buff     = imu01;

    imu00.time   = time;
    imu00.dt     = buff.dt - (buff.time - time);
    imu00.dtheta = buff.dtheta * (1 - scale);
    imu00.dvel   = buff.dvel * (1 - scale);
    imu00.odovel = buff.odovel * (1 - scale);

    imu11.time   = buff.time;
    imu11.dt     = buff.time - time;
    imu11.dtheta = buff.dtheta * scale;
    imu11.dvel   = buff.dvel * scale;
    imu11.odovel = buff.odovel * scale;
}

bool MISC::getImuSeriesFromTo(const std::deque<std::pair<IMU, IntegrationState>> &ins_windows, double start, double end,
                              vector<IMU> &series) {
    // 获得起点和终点附近的索引
    size_t is = getInsWindowIndex(ins_windows, start);
    size_t ie = getInsWindowIndex(ins_windows, end);

    if ((is == 0) && (ie == 0)) {
        LOGE << "Failed to get right IMU series " << Logging::doubleData(start) << " to " << Logging::doubleData(end);
        return false;
    }

    IMU imu0, imu1, imu;
    series.clear();

    // 内插起点
    // imu0为ins_windows中与start时间最接近的前一个imu索引
    imu0 = ins_windows[is - 1].first;
    // imu1为ins_windows中与end时间最接近的后一个imu索引 
    imu1 = ins_windows[is].first;

    int isneed = isNeedInterpolation(imu0, imu1, start);

    // 如果start与imu0距离小于阈值（可视为接近），则将imu0与imu1都存入IMU容器中
    if (isneed == -1) {
        series.push_back(imu0);
        series.push_back(imu1);
    } else if (isneed == 1) { // 若start与imu1距离小于阈值，则仅将imu1存入IMU容器中
        series.push_back(imu1);
    } else if (isneed == 2) { // 若start与两者都不小于阈值，则需要内插，得到新的imu数据。分别为imu,imu1
        imuInterpolation(imu1, imu, imu1, start);
        series.push_back(imu);
        series.push_back(imu1);
    }

    // 中间部分
    for (size_t k = is + 1; k < ie - 1; k++) {
        series.push_back(ins_windows[k].first);
    }

    // 内插终点
    imu0 = ins_windows[ie - 1].first;
    imu1 = ins_windows[ie].first;

    isneed = isNeedInterpolation(imu0, imu1, end);
    if (isneed == -1) {
        series.push_back(imu0);
    } else if (isneed == 1) {
        series.push_back(imu0);
        series.push_back(imu1);
    } else if (isneed == 2) {
        series.push_back(imu0);
        imuInterpolation(imu1, imu, imu1, end);
        series.push_back(imu);
    }
    // IMU容器中最后一个元素的时间设置为end。但并没有设置第一个元素的时间
    series.back().time = end;

    return true;
}


// imu的零速检测
bool MISC::detectZeroVelocity(const vector<IMU> &imu_buffer, double imudatarate, vector<double> &average) {

    auto size          = static_cast<double>(imu_buffer.size());
    double size_invert = 1.0 / size;

    double sum[6];
    double std[6];

    average.resize(6);
    average[0] = average[1] = average[2] = average[3] = average[4] = average[5] = 0;

    // 对imu_buffer中的imu数据增量进行累加
    for (const auto &imu : imu_buffer) {
        average[0] += imu.dtheta.x();
        average[1] += imu.dtheta.y();
        average[2] += imu.dtheta.z();
        average[3] += imu.dvel.x();
        average[4] += imu.dvel.y();
        average[5] += imu.dvel.z();
    }
    
    // 求imu增量的平均值
    average[0] *= size_invert;
    average[1] *= size_invert;
    average[2] *= size_invert;
    average[3] *= size_invert;
    average[4] *= size_invert;
    average[5] *= size_invert;


    sum[0] = sum[1] = sum[2] = sum[3] = sum[4] = sum[5] = 0;

    // 计算imu增量数据与平均值的差的平方和
    for (const auto &imu : imu_buffer) {
        sum[0] += (imu.dtheta.x() - average[0]) * (imu.dtheta.x() - average[0]);
        sum[1] += (imu.dtheta.y() - average[1]) * (imu.dtheta.y() - average[1]);
        sum[2] += (imu.dtheta.z() - average[2]) * (imu.dtheta.z() - average[2]);
        sum[3] += (imu.dvel.x() - average[3]) * (imu.dvel.x() - average[3]);
        sum[4] += (imu.dvel.y() - average[4]) * (imu.dvel.y() - average[4]);
        sum[5] += (imu.dvel.z() - average[5]) * (imu.dvel.z() - average[5]);
    }

    // 计算出imu数据增量的标准差，用于衡量imu增量的分散程度
    std[0] = sqrt(sum[0] * size_invert) * imudatarate; // 等价于standard deviation * imudatarate
    std[1] = sqrt(sum[1] * size_invert) * imudatarate;  
    std[2] = sqrt(sum[2] * size_invert) * imudatarate;
    std[3] = sqrt(sum[3] * size_invert) * imudatarate;
    std[4] = sqrt(sum[4] * size_invert) * imudatarate;
    std[5] = sqrt(sum[5] * size_invert) * imudatarate;

    // 下面表示如果imu数据增量很集中，则说明三轴加速度和角速度是恒定的
    // 当三轴加速度和角速度恒定时只有IMU相对地球静止时才会发生，原因：
    // 1. IMU一定不发生转动，因为如果转动，则三轴加速度就不恒定了
    // 2. IMU会相对地球发生直线运动么？严格来说是可以的，假如IMU做自由落体运动则比力为0，且三轴角速度也为0，那么此时IMU不是零速。但该情况基本不存在
    // 3. 上述类似情况发生情形很少。所以当满足两个条件时，一般会认为IMU处于零速状态
    if ((std[0] < ZERO_VELOCITY_GYR_THRESHOLD) && (std[1] < ZERO_VELOCITY_GYR_THRESHOLD) &&
        (std[2] < ZERO_VELOCITY_GYR_THRESHOLD) && (std[3] < ZERO_VELOCITY_ACC_THRESHOLD) &&
        (std[4] < ZERO_VELOCITY_ACC_THRESHOLD) && (std[5] < ZERO_VELOCITY_ACC_THRESHOLD)) {

        return true;
    }

    return false;
}

void MISC::writeNavResult(const IntegrationConfiguration &config, const IntegrationState &state,
                          const FileSaver::Ptr &navfile, const FileSaver::Ptr &errfile,
                          const FileSaver::Ptr &trajfile) {
    static int counts = 0;
    if (counts++ % 10) {
        return;
    }

    // 保存结果
    vector<double> result;

    double time  = state.time;
    Pose global  = Earth::local2global(config.origin, Pose{state.q.toRotationMatrix(), state.p});
    Vector3d pos = global.t;
    pos.segment(0, 2) *= R2D;
    Vector3d att = Rotation::matrix2euler(global.R) * R2D;
    Vector3d vel = state.v;
    Vector3d bg  = state.bg * R2D * 3600;
    Vector3d ba  = state.ba * 1e5;

    {
        // navigation file
        result.clear();

        result.push_back(0);
        result.push_back(time);
        result.push_back(pos[0]);
        result.push_back(pos[1]);
        result.push_back(pos[2]);
        result.push_back(vel[0]);
        result.push_back(vel[1]);
        result.push_back(vel[2]);
        result.push_back(att[0]);
        result.push_back(att[1]);
        result.push_back(att[2]);
        navfile->dump(result);
        navfile->flush();
    }

    {
        // imu error file
        result.clear();

        result.push_back(time);
        result.push_back(bg[0]);
        result.push_back(bg[1]);
        result.push_back(bg[2]);
        result.push_back(ba[0]);
        result.push_back(ba[1]);
        result.push_back(ba[2]);

        if (config.iswithscale) {
            Vector3d sg = state.sg * 1e6;
            Vector3d sa = state.sa * 1e6;
            result.push_back(sg[0]);
            result.push_back(sg[1]);
            result.push_back(sg[2]);
            result.push_back(sa[0]);
            result.push_back(sa[1]);
            result.push_back(sa[2]);
        }

        result.push_back(state.sodo);
        errfile->dump(result);
        errfile->flush();
    }

    {
        // trajectory
        result.clear();

        result.push_back(time);
        result.push_back(state.p[0]);
        result.push_back(state.p[1]);
        result.push_back(state.p[2]);
        result.push_back(state.q.x());
        result.push_back(state.q.y());
        result.push_back(state.q.z());
        result.push_back(state.q.w());

        trajfile->dump(result);
    }
}
