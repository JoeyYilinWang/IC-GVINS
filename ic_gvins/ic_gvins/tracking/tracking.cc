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

#include "tracking.h"

#include "common/angle.h"
#include "common/logging.h"
#include "common/rotation.h"

#include <tbb/tbb.h>
#include <yaml-cpp/yaml.h>

Tracking::Tracking(Camera::Ptr camera, Map::Ptr map, Drawer::Ptr drawer, const string &configfile,
                   const string &outputpath)
    : frame_cur_(nullptr)
    , frame_ref_(nullptr)
    , camera_(std::move(camera))
    , map_(std::move(map))
    , drawer_(std::move(drawer))
    , isnewkeyframe_(false)
    , isinitializing_(true)
    , histogram_(0) {

    logfilesaver_ = FileSaver::create(outputpath + "/tracking.txt", 3);
    if (!logfilesaver_->isOpen()) {
        LOGE << "Failed to open data file";
        return;
    }

    YAML::Node config;
    std::vector<double> vecdata;
    config = YAML::LoadFile(configfile);

    track_check_histogram_ = config["track_check_histogram"].as<bool>(); // 配置文件中这里设置为了false
    track_min_parallax_    = config["track_min_parallax"].as<double>(); // 关键帧最小像素视差设置为20
    track_max_features_    = config["track_max_features"].as<int>(); // 最大提取特征点数量为200
    track_max_interval_    = config["track_max_interval"].as<double>(); // 最大的关键帧间隔设置为0.5秒
    track_max_interval_ *= 0.95; // 错开整时间间隔

    is_use_visualization_   = config["is_use_visualization"].as<bool>(); // 开启可视化
    reprojection_error_std_ = config["reprojection_error_std"].as<double>(); // 重投影误差设置为1.5用于优化和排除离群点

    // 直方图均衡化
    clahe_ = cv::createCLAHE(3.0, cv::Size(21, 21));

    // 分块索引
    // lround为四舍五入操作符
    block_cols_ = static_cast<int>(lround(camera_->width() / TRACK_BLOCK_SIZE));
    block_rows_ = static_cast<int>(lround(camera_->height() / TRACK_BLOCK_SIZE));
    block_cnts_ = block_cols_ * block_rows_;

    int col, row;
    row = camera_->height() / block_rows_;
    col = camera_->width() / block_cols_;
    block_indexs_.emplace_back(std::make_pair(col, row));
    for (int i = 0; i < block_rows_; i++) {
        for (int j = 0; j < block_cols_; j++) {
            block_indexs_.emplace_back(std::make_pair(col * j, row * i)); // 得到诸多分块的行列索引
        }
    }

    // 每个分块提取的角点数量
    track_max_block_features_ = 
        static_cast<int>(lround(static_cast<double>(track_max_features_) / static_cast<double>(block_cnts_)));

    // 每个格子的提取特征数量平方面积为格子面积的 2/3
    track_min_pixel_distance_ = static_cast<int>(round(TRACK_BLOCK_SIZE / sqrt(track_max_block_features_ * 1.5)));
}

// 计算直方图
double Tracking::calculateHistigram(const Mat &image) {
    Mat histogram; 
    int channels[]         = {0}; // 因为是灰度图像，所以通道只有一个，标志为0
    int histsize           = 256; // 因为是通道为1，所以直方图维度也是一维的，而且直方图的bin数量为256
    float range[]          = {0, 256}; // 设置像素统计范围，这里设置为0到256（实际上应该为0到255，但也无伤大雅）
    const float *histrange = {range}; 
    bool uniform = true, accumulate = false; // uniform设置为true表示bin是均匀的。accumulate设置为false表示在每次调用时我们都要清空不累加

    // 根据上述参数，可以得到像素灰度的统计直方图histogram了
    cv::calcHist(&image, 1, channels, Mat(), histogram, 1, &histsize, &histrange, uniform, accumulate);

    double hist = 0; 
    // 遍历直方图，一条for循环完成以下任务
    // k / 256，将灰度进行归一化
    // 将bin（k）上统计的像素数量与上面归一化值相乘
    // 遍历所有bin，完成上述任务，然后再相加。就得到了针对该图像所有像素的归一化灰度值之和
    for (int k = 0; k < 256; k++) {
        hist += histogram.at<float>(k) * (float) k / 256.0;
    }
    // 除以总像素数量，就得到了平均归一化灰度值
    hist /= (image.cols * image.rows);

    return hist;
}

bool Tracking::preprocessing(Frame::Ptr frame) {
    // 当处理该帧时，首先默认将其认为是非关键帧
    isnewkeyframe_ = false;

    // 彩色转灰度
    if (frame->image().channels() == 3) {
        cv::cvtColor(frame->image(), frame->image(), cv::COLOR_BGR2GRAY);
    }

    if (track_check_histogram_) { // 由于已设置为false，因此不检查直方图
        // 计算直方图参数
        double hist = calculateHistigram(frame->image());
        if (histogram_ != 0) {
            double rate = fabs((hist - histogram_) / histogram_);

            // 图像直方图变化比例大于10%, 则跳过当前帧
            if (rate > 0.1) {
                LOGW << "Histogram change too large at " << Logging::doubleData(frame->stamp()) << " with " << rate;
                passed_cnt_++; // 跳过帧的数量+1

                if (passed_cnt_ > 1) { 
                    histogram_ = 0;
                }
                return false;
            }
        }
        histogram_ = hist;
    }

    frame_pre_ = frame_cur_;
    frame_cur_ = std::move(frame);

    // 直方图均衡化，增强图像对比度
    clahe_->apply(frame_cur_->image(), frame_cur_->image());

    return true;
}

TrackState Tracking::track(Frame::Ptr frame) {
    // Tracking

    timecost_.restart();

    TrackState track_state = TRACK_PASSED;

    // 预处理
    if (!preprocessing(std::move(frame))) {
        return track_state;
    }

    if (isinitializing_) {
        // Initialization
        if (frame_ref_ == nullptr) {
            doResetTracking();

            frame_ref_ = frame_cur_;

            featuresDetection(frame_ref_, false);

            return TRACK_FIRST_FRAME;
        }

        if (pts2d_ref_.empty()) {
            featuresDetection(frame_ref_, false);
        }

        // 从参考帧跟踪过来的特征点
        trackReferenceFrame();

        if (parallax_ref_ < track_min_parallax_) {
            showTracking();
            return TRACK_INITIALIZING;
        }

        LOGI << "Initialization tracking with parallax " << parallax_ref_;

        triangulation();

        if (doResetTracking()) {
            LOGW << "Reset initialization";
            showTracking();

            makeNewFrame(KEYFRAME_NORMAL);
            return TRACK_FIRST_FRAME;
        }

        // 初始化两帧都是关键帧
        frame_ref_->setKeyFrame(KEYFRAME_NORMAL);

        // 新关键帧, 地图更新, 数据转存
        makeNewFrame(KEYFRAME_NORMAL);
        last_keyframe_ = frame_cur_;

        isinitializing_ = false;

        track_state = TRACK_TRACKING;
    } else {
        // Tracking

        // 跟踪上一帧中带路标点的特征, 利用预测的位姿先验
        trackMappoint();

        // 未关联路标点的新特征, 补偿旋转预测
        trackReferenceFrame();

        // 检查关键帧类型
        auto keyframe_state = checkKeyFrameSate();

        // 正常关键帧, 需要三角化路标点
        if ((keyframe_state == KEYFRAME_NORMAL) || (keyframe_state == KEYFRAME_REMOVE_OLDEST)) {
            // 三角化补充路标点
            triangulation();
        } else {
            // 添加新的特征
            featuresDetection(frame_cur_, true);
        }

        // 跟踪失败, 路标点数据严重不足
        if (doResetTracking()) {
            makeNewFrame(KEYFRAME_NORMAL);
            return TRACK_LOST;
        }

        // 观测帧, 进行插入
        if (keyframe_state != KEYFRAME_NONE) {
            makeNewFrame(keyframe_state);
        }

        track_state = TRACK_TRACKING;

        if (keyframe_state != KEYFRAME_NONE) {
            writeLoggingMessage();
        }
    }

    // 显示跟踪情况
    showTracking();

    return track_state;
}

bool Tracking::isGoodDepth(double depth, double scale) {
    return ((depth > MapPoint::NEAREST_DEPTH) && (depth < MapPoint::FARTHEST_DEPTH * scale));
}

void Tracking::makeNewFrame(int state) {
    frame_cur_->setKeyFrame(state);
    isnewkeyframe_ = true;

    // 仅当正常关键帧才更新参考帧
    if ((state == KEYFRAME_NORMAL) || (state == KEYFRAME_REMOVE_OLDEST)) {
        frame_ref_ = frame_cur_;

        featuresDetection(frame_ref_, true);
    }
}

keyFrameState Tracking::checkKeyFrameSate() {
    keyFrameState keyframe_state = KEYFRAME_NONE;

    // 相邻时间太短, 不进行关键帧处理
    double dt = frame_cur_->stamp() - last_keyframe_->stamp();
    if (dt < TRACK_MIN_INTERVAl) {
        return keyframe_state;
    }

    double parallax = (parallax_map_ * parallax_map_counts_ + parallax_ref_ * parallax_ref_counts_) /
                      (parallax_map_counts_ + parallax_ref_counts_);
    if (parallax > track_min_parallax_) {
        // 新的关键帧, 满足最小像素视差

        keyframe_state = map_->isWindowFull() ? KEYFRAME_REMOVE_OLDEST : KEYFRAME_NORMAL;

        LOGI << "Keyframe at " << Logging::doubleData(frame_cur_->stamp()) << ", mappoints "
             << frame_cur_->numFeatures() << ", interval " << dt << ", parallax " << parallax;
    } else if (dt > track_max_interval_) {
        // 普通观测帧, 非关键帧
        keyframe_state = KEYFRAME_REMOVE_SECOND_NEW;
        LOGI << "Keyframe at " << Logging::doubleData(frame_cur_->stamp()) << " due to long interval";
    }

    // 切换上一关键帧, 用于时间间隔计算
    if (keyframe_state != KEYFRAME_NONE) {
        last_keyframe_ = frame_cur_;

        // 更新路标点在观测帧中的使用次数
        for (auto &mappoint : tracked_mappoint_) {
            mappoint->increaseUsedTimes();
        }

        // 输出关键帧信息
        logging_data_.clear();

        logging_data_.push_back(frame_cur_->stamp());
        logging_data_.push_back(dt);
        logging_data_.push_back(parallax);
        logging_data_.push_back(relativeTranslation());
        logging_data_.push_back(relativeRotation());
    }

    return keyframe_state;
}

void Tracking::writeLoggingMessage() {
    logging_data_.push_back(static_cast<double>(frame_cur_->features().size()));
    logging_data_.push_back(timecost_.costInMillisecond());

    logfilesaver_->dump(logging_data_);
    logfilesaver_->flush();
}

bool Tracking::doResetTracking() {
    if (!frame_cur_->numFeatures()) {
        isinitializing_ = true;
        frame_ref_      = frame_cur_;
        pts2d_new_.clear();
        pts2d_ref_.clear();
        pts2d_ref_frame_.clear();
        velocity_ref_.clear();
        return true;
    }

    return false;
}

double Tracking::relativeTranslation() {
    return (frame_cur_->pose().t - frame_ref_->pose().t).norm();
}

double Tracking::relativeRotation() {
    Matrix3d R     = frame_cur_->pose().R.transpose() * frame_ref_->pose().R;
    Vector3d euler = Rotation::matrix2euler(R);

    // Only for heading
    return fabs(euler[1] * R2D);
}

void Tracking::showTracking() {
    if (!is_use_visualization_) {
        return;
    }

    drawer_->updateFrame(frame_cur_);
}

bool Tracking::trackMappoint() {

    // 上一帧中的路标点
    mappoint_matched_.clear();
    vector<cv::Point2f> pts2d_map, pts2d_matched, pts2d_map_undis;
    vector<MapPointType> mappoint_type;
    auto features = frame_pre_->features();
    for (auto &feature : features) {
        auto mappoint = feature.second->getMapPoint();
        if (mappoint && !mappoint->isOutlier()) {
            mappoint_matched_.push_back(mappoint);
            pts2d_map_undis.push_back(feature.second->keyPoint());
            pts2d_map.push_back(feature.second->distortedKeyPoint());
            mappoint_type.push_back(mappoint->mapPointType());

            // 预测的特征点
            auto pixel = camera_->world2pixel(mappoint->pos(), frame_cur_->pose());

            pts2d_matched.emplace_back(pixel);
        }
    }
    if (pts2d_matched.empty()) {
        LOGE << "No feature with mappoint in previous frame";
        return false;
    }

    // 预测的特征点像素坐标添加畸变, 用于跟踪
    camera_->distortPoints(pts2d_matched);

    vector<uint8_t> status, status_reverse;
    vector<float> error;
    vector<cv::Point2f> pts2d_reverse = pts2d_map;

    // 正向光流
    cv::calcOpticalFlowPyrLK(frame_pre_->image(), frame_cur_->image(), pts2d_map, pts2d_matched, status, error,
                             cv::Size(21, 21), TRACK_PYRAMID_LEVEL,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                             cv::OPTFLOW_USE_INITIAL_FLOW);
    // 反向光流
    cv::calcOpticalFlowPyrLK(frame_cur_->image(), frame_pre_->image(), pts2d_matched, pts2d_reverse, status_reverse,
                             error, cv::Size(21, 21), TRACK_PYRAMID_LEVEL,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                             cv::OPTFLOW_USE_INITIAL_FLOW);

    // 跟踪失败的
    for (size_t k = 0; k < status.size(); k++) {
        if (status[k] && status_reverse[k] && !isOnBorder(pts2d_matched[k]) &&
            (ptsDistance(pts2d_reverse[k], pts2d_map[k]) < 0.5)) {
            status[k] = 1;
        } else {
            status[k] = 0;
        }
    }
    reduceVector(pts2d_map, status);
    reduceVector(pts2d_matched, status);
    reduceVector(mappoint_matched_, status);
    reduceVector(mappoint_type, status);
    reduceVector(pts2d_map_undis, status);

    if (pts2d_matched.empty()) {
        LOGE << "Track previous with mappoint failed";
        // 清除上一帧的跟踪
        if (is_use_visualization_) {
            drawer_->updateTrackedMapPoints({}, {}, {});
        }
        parallax_map_        = 0;
        parallax_map_counts_ = 0;
        return false;
    }

    // 匹配后的点, 需要重新矫正畸变
    auto pts2d_matched_undis = pts2d_matched;
    camera_->undistortPoints(pts2d_matched_undis);

    // 匹配的3D-2D
    frame_cur_->clearFeatures();
    tracked_mappoint_.clear();

    double dt = frame_cur_->stamp() - frame_pre_->stamp();
    for (size_t k = 0; k < pts2d_matched_undis.size(); k++) {
        auto mappoint = mappoint_matched_[k];

        // 将3D-2D匹配到的landmarks指向到当前帧
        auto velocity = (camera_->pixel2cam(pts2d_matched_undis[k]) - camera_->pixel2cam(pts2d_map_undis[k])) / dt;
        auto feature  = Feature::createFeature(frame_cur_, {velocity.x(), velocity.y()}, pts2d_matched_undis[k],
                                               pts2d_matched[k], FEATURE_MATCHED);
        mappoint->addObservation(feature);
        feature->addMapPoint(mappoint);
        frame_cur_->addFeature(mappoint->id(), feature);

        // 用于更新使用次数
        tracked_mappoint_.push_back(mappoint);
    }

    // 路标点跟踪情况
    if (is_use_visualization_) {
        drawer_->updateTrackedMapPoints(pts2d_map, pts2d_matched, mappoint_type);
    }

    parallax_map_counts_ = parallaxFromReferenceMapPoints(parallax_map_);

    LOGI << "Track " << tracked_mappoint_.size() << " map points";

    return true;
}

bool Tracking::trackReferenceFrame() {

    if (pts2d_ref_.empty()) {
        LOGW << "No new feature in previous frame " << Logging::doubleData(frame_cur_->stamp());
        return false;
    }
s
    // 补偿旋转预测
    Matrix3d r_cur_pre = frame_cur_->pose().R.transpose() * frame_pre_->pose().R;

    // 原始畸变补偿
    auto pts2d_new_undis = pts2d_new_;
    camera_->undistortPoints(pts2d_new_undis);

    pts2d_cur_.clear();
    for (const auto &pp_pre : pts2d_new_undis) {
        Vector3d pc_pre = camera_->pixel2cam(pp_pre);
        Vector3d pc_cur = r_cur_pre * pc_pre;

        // 加畸变并投影
        auto pp_cur = camera_->distortCameraPoint(pc_cur);
        pts2d_cur_.emplace_back(pp_cur);
    }

    // 跟踪参考帧
    vector<uint8_t> status, status_reverse;
    vector<float> error;
    vector<cv::Point2f> pts2d_reverse = pts2d_new_;

    // 根据前一帧frame_pre_的2d特征点的初始位置pts2d_new_追踪当前帧frame_cur_的特征点新位置pts2d_cur_
    cv::calcOpticalFlowPyrLK(frame_pre_->image(), frame_cur_->image(), pts2d_new_, pts2d_cur_, status, error,
                             cv::Size(21, 21), TRACK_PYRAMID_LEVEL,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                             cv::OPTFLOW_USE_INITIAL_FLOW);

    // 反向光流
    cv::calcOpticalFlowPyrLK(frame_cur_->image(), frame_pre_->image(), pts2d_cur_, pts2d_reverse, status_reverse, error,
                             cv::Size(21, 21), TRACK_PYRAMID_LEVEL,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                             cv::OPTFLOW_USE_INITIAL_FLOW);

    // 剔除跟踪失败的, 正向反向跟踪在0.5个像素以内
    for (size_t k = 0; k < status.size(); k++) {
        if (status[k] && status_reverse[k] && !isOnBorder(pts2d_cur_[k]) &&
            (ptsDistance(pts2d_reverse[k], pts2d_new_[k]) < 0.5)) {
            status[k] = 1;
        } else {
            status[k] = 0;
        }
    }
    reduceVector(pts2d_ref_, status); // 在参考点中剔除跟踪失败的
    reduceVector(pts2d_cur_, status); // 剔除当前帧跟踪失败的
    reduceVector(pts2d_new_, status); // 剔除前一帧跟踪失败的
    reduceVector(pts2d_ref_frame_, status); 
    reduceVector(velocity_ref_, status);

    if (pts2d_ref_.empty()) {
        LOGW << "No new feature in previous frame";
        drawer_->updateTrackedRefPoints({}, {});
        return false;
    }

    // 原始带畸变的角点
    pts2d_new_undis      = pts2d_new_;
    auto pts2d_cur_undis = pts2d_cur_;

    camera_->undistortPoints(pts2d_new_undis);
    camera_->undistortPoints(pts2d_cur_undis);

    // 计算像素速度
    velocity_cur_.clear();
    double dt = frame_cur_->stamp() - frame_pre_->stamp();

    for (size_t k = 0; k < pts2d_cur_undis.size(); k++) {
        Vector3d vel      = (camera_->pixel2cam(pts2d_cur_undis[k]) - camera_->pixel2cam(pts2d_new_undis[k])) / dt;
        Vector2d velocity = {vel.x(), vel.y()};
        velocity_cur_.push_back(velocity);

        // 在关键帧后新增加的特征
        if (pts2d_ref_frame_[k]->id() > frame_ref_->id()) {
            velocity_ref_[k] = velocity;
        }
    }

    // 计算视差
    auto pts2d_ref_undis = pts2d_ref_;
    camera_->undistortPoints(pts2d_ref_undis);
    parallax_ref_counts_ = parallaxFromReferenceKeyPoints(pts2d_ref_undis, pts2d_cur_undis, parallax_ref_);

    // Fundamental粗差剔除
    if (pts2d_cur_.size() >= 15) {
        cv::findFundamentalMat(pts2d_new_undis, pts2d_cur_undis, cv::FM_RANSAC, reprojection_error_std_, 0.99, status);

        reduceVector(pts2d_ref_, status);
        reduceVector(pts2d_cur_, status);
        reduceVector(pts2d_ref_frame_, status);
        reduceVector(velocity_cur_, status);
        reduceVector(velocity_ref_, status);
    }

    if (pts2d_cur_.empty()) {
        LOGW << "No new feature in previous frame";
        drawer_->updateTrackedRefPoints({}, {});
        return false;
    }

    // 从参考帧跟踪过来的新特征点
    if (is_use_visualization_) {
        drawer_->updateTrackedRefPoints(pts2d_ref_, pts2d_cur_);
    }

    // 用于下一帧的跟踪
    pts2d_new_ = pts2d_cur_;

    LOGI << "Track " << pts2d_new_.size() << " reference points";

    return !pts2d_new_.empty();
}

void Tracking::featuresDetection(Frame::Ptr &frame, bool ismask) {

    // 特征点足够则无需提取
    // 要注意的是，pts2d_ref_的点并不特别属于该帧，pts2d_ref_表示所有参考帧对应的某些特征点，而且与后面的pts2d_cur_、pts2d_new_一一对应的
    // 因为当前帧
    int num_features = static_cast<int>(frame->features().size() + pts2d_ref_.size());
    if (num_features > (track_max_features_ - 5)) {
        return;
    }

    // 初始化分配内存
    int features_cnts[block_cnts_]; // 用于统计各区块特征点的数量
    vector<vector<cv::Point2f>> block_features(block_cnts_); // 储存所有区块特征点
    // 必要的分配内存, 否则并行会造成数据结构错乱
    for (auto &block : block_features) {
        block.reserve(track_max_block_features_);
    }
    for (int k = 0; k < block_cnts_; k++) {
        features_cnts[k] = 0;
    }

    // 计算每个分块已有特征点数量
    int col, row;
    for (const auto &feature : frame->features()) {
        col = int(feature.second->keyPoint().x / (float) block_indexs_[0].first);
        row = int(feature.second->keyPoint().y / (float) block_indexs_[0].second);
        features_cnts[row * block_cols_ + col]++;
    }
    // pts2d_new_表示三角化后剩下的点
    for (auto &pts2d : pts2d_new_) {
        col = int(pts2d.x / (float) block_indexs_[0].first);
        row = int(pts2d.y / (float) block_indexs_[0].second);
        features_cnts[row * block_cols_ + col]++;
    }

    // 设置感兴趣区域, 没有特征的区域
    Mat mask = Mat(camera_->size(), CV_8UC1, 255);
    if (ismask) {
        // 已经跟踪上的点
        // 要注意的是，该蒙版是以当前帧存在的特征点进行处理的
        for (const auto &pt : frame_cur_->features()) {
            // 在mask图像上以特定圆心（pt.second->keypoint()）和半径（track_min_pixel_distance_）绘制一个填充颜色为黑色的园
            cv::circle(mask, pt.second->keyPoint(), track_min_pixel_distance_, 0, cv::FILLED);
        }

        // 还在跟踪的点
        // 同样，pts_new_也是pts_cur_三角化之后剩下的点
        for (const auto &pts2d : pts2d_new_) {
            cv::circle(mask, pts2d, track_min_pixel_distance_, 0, cv::FILLED);
        }
    }

    // 亚像素角点提取参数
    cv::Size win_size          = cv::Size(5, 5);
    cv::Size zero_zone         = cv::Size(-1, -1);
    cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 20, 0.01);

    // 定义一个并行处理函数，使用TBB（Threading Building Blocks）库进行多线程处理
    auto tracking_function = [&](const tbb::blocked_range<int> &range) {
        for (int k = range.begin(); k != range.end(); k++) {
            int blocl_track_num = track_max_block_features_ - features_cnts[k]; // 计算每个块除已有的特征点外还需要多少特征点
            if (blocl_track_num > 0) {

                int cols = k % block_cols_;
                int rows = k / block_cols_;

                int col_sta = cols * block_indexs_[0].first;
                int col_end = col_sta + block_indexs_[0].first;
                int row_sta = rows * block_indexs_[0].second;
                int row_end = row_sta + block_indexs_[0].second;
                if (k != (block_cnts_ - 1)) { // 如果k没到末尾，则区块尾部少五个元素
                    col_end -= 5;
                    row_end -= 5;
                }
                // 提取区块和掩码
                Mat block_image = frame->image().colRange(col_sta, col_end).rowRange(row_sta, row_end);
                Mat block_mask  = mask.colRange(col_sta, col_end).rowRange(row_sta, row_end);

                // 使用Shi-Tomasi算法检测角点
                // 这些检测到的角点被存储在block_features[k]中
                cv::goodFeaturesToTrack(block_image, block_features[k], blocl_track_num, 0.01,
                                        track_min_pixel_distance_, block_mask);
                if (!block_features[k].empty()) {
                    // 被用于优化block_image图像中检测到的角点位置，优化后的角点位置被存储在block_features[k]中。
                    // 这样可以提高角点位置的精度，有助于提高后续处理（如光流跟踪，图像配准等）的准确性
                    cv::cornerSubPix(block_image, block_features[k], win_size, zero_zone, term_crit);
                }
            }
        }
    };

    // 将图像的处理任务划分为多个小任务，然后在多个线程上并行处理这些小任务，从而提高了处理效率。这种方法尤其适合在多核处理器上处理大规模的数据
    tbb::parallel_for(tbb::blocked_range<int>(0, block_cnts_), tracking_function);

    // 调整角点的坐标
    int num_new_features = 0; // 用于统计新加特征点的数量

    // 连续跟踪的角点, 未三角化的点
    if (!ismask) {
        pts2d_new_.clear();
        pts2d_ref_.clear();
        pts2d_ref_frame_.clear();
        velocity_ref_.clear();
    }

    for (int k = 0; k < block_cnts_; k++) {
        col = k % block_cols_;
        row = k / block_cols_;

        // 由于block_features[k]保存的都是各区块上的特征点坐标，因此需要恢复到原始坐标
        for (const auto &point : block_features[k]) {
            float x = static_cast<float>(col * block_indexs_[0].first) + point.x;
            float y = static_cast<float>(row * block_indexs_[0].second) + point.y;

            auto pts2d = cv::Point2f(x, y);
            pts2d_ref_.push_back(pts2d); // 新特征点压入pts2d_ref_
            pts2d_new_.push_back(pts2d); // 新特征点压入pts2d_new_
            pts2d_ref_frame_.push_back(frame); // 将当前帧作为参考帧
            velocity_ref_.emplace_back(0, 0); // 为每个特征点初始化2d速度

            num_new_features++; 
        }
    }

    LOGI << "Add " << num_new_features << " new features to " << num_features;
}

/**
 * @brief 三角化
 * @return 三角化是否成功的判断
 */
bool Tracking::triangulation() {
    // 无跟踪上的特征
    if (pts2d_cur_.empty()) {
        return false;
    }

    Pose pose0;
    Pose pose1 = frame_cur_->pose();

    Eigen::Matrix<double, 3, 4> T_c_w_0, T_c_w_1;
    T_c_w_1 = pose2Tcw(pose1).topRows<3>();

    int num_succeeded = 0;
    int num_outlier   = 0;
    int num_reset     = 0;
    int num_outtime   = 0;

    // 原始带畸变的角点
    auto pts2d_ref_undis = pts2d_ref_;
    auto pts2d_cur_undis = pts2d_cur_;

    // 矫正畸变以进行三角化
    camera_->undistortPoints(pts2d_ref_undis);
    camera_->undistortPoints(pts2d_cur_undis);

    // 计算使用齐次坐标, 相机坐标系
    vector<uint8_t> status;
    // 以当前跟踪上的特征点为准
    for (size_t k = 0; k < pts2d_cur_.size(); k++) {
        auto pp0 = pts2d_ref_undis[k];
        auto pp1 = pts2d_cur_undis[k];

        // 参考帧
        auto frame_ref = pts2d_ref_frame_[k];
        if (frame_ref->id() > frame_ref_->id()) {
            pts2d_ref_frame_[k] = frame_cur_;
            pts2d_ref_[k]       = pts2d_cur_[k];
            status.push_back(1);
            num_reset++;
            continue;
        }

        // 移除长时间跟踪导致参考帧已经不在窗口内的观测
        if (map_->isWindowNormal() && !map_->isKeyFrameInMap(frame_ref)) {
            status.push_back(0);
            num_outtime++;
            continue;
        }

        // 进行必要的视差检查, 保证三角化有效
        pose0           = frame_ref->pose();
        double parallax = keyPointParallax(pts2d_ref_undis[k], pts2d_cur_undis[k], pose0, pose1);
        if (parallax < TRACK_MIN_PARALLAX) {
            status.push_back(1);
            continue;
        }

        T_c_w_0 = pose2Tcw(pose0).topRows<3>();

        // 三角化
        Vector3d pc0 = camera_->pixel2cam(pts2d_ref_undis[k]);
        Vector3d pc1 = camera_->pixel2cam(pts2d_cur_undis[k]);
        Vector3d pw;
        triangulatePoint(T_c_w_0, T_c_w_1, pc0, pc1, pw);

        // 三角化错误的点剔除
        if (!isGoodToTrack(pp0, pose0, pw, 1.0, 3.0) || !isGoodToTrack(pp1, pose1, pw, 1.0, 3.0)) {
            status.push_back(0);
            num_outlier++;
            continue;
        }
        status.push_back(0);
        num_succeeded++;

        // 新的路标点, 加入新的观测, 路标点加入地图
        auto pc       = camera_->world2cam(pw, frame_ref->pose());
        double depth  = pc.z();
        // 地图点以参考帧为关键帧
        auto mappoint = MapPoint::createMapPoint(frame_ref, pw, pts2d_ref_undis[k], depth, MAPPOINT_TRIANGULATED);
    
        // 初始化当前帧的特征点
        auto feature = Feature::createFeature(frame_cur_, velocity_cur_[k], pts2d_cur_undis[k], pts2d_cur_[k],
                                              FEATURE_TRIANGULATED);
        mappoint->addObservation(feature); // 增加当前帧特征点的观测
        feature->addMapPoint(mappoint); // 以该地图点作为特征点的对应三维点
        frame_cur_->addFeature(mappoint->id(), feature); // 当前图像帧增加特征点
        mappoint->increaseUsedTimes(); // 地图点增加使用次数

        // 初始化参考帧的特征点
        feature = Feature::createFeature(frame_ref, velocity_ref_[k], pts2d_ref_undis[k], pts2d_ref_[k],
                                         FEATURE_TRIANGULATED);
        mappoint->addObservation(feature); // 增加参考帧特征点的观测
        feature->addMapPoint(mappoint); // 以地图点作为特征点对应的三维点
        frame_ref->addFeature(mappoint->id(), feature); // 将参考帧增加特征点
        mappoint->increaseUsedTimes(); // 地图点增加使用次数

        // 新三角化的路标点缓存到最新的关键帧, 不直接加入地图
        // 这样做的好处是有助于减少全局地图中的噪声，因为在添加路标点到全局地图之前，我们可以在当前帧中对这些新的路标点进行额外的检查或过滤。
        frame_cur_->addNewUnupdatedMappoint(mappoint);
    }

    // 由于视差不够未及时三角化的角点
    reduceVector(pts2d_ref_, status);
    reduceVector(pts2d_ref_frame_, status);
    reduceVector(pts2d_cur_, status);
    reduceVector(velocity_ref_, status);

    pts2d_new_ = pts2d_cur_; // 将成功三角化的点、三角化错误的点以及过时的关键帧特征点去除之后剩下的点

    LOGI << "Triangulate " << num_succeeded << " 3D points with " << pts2d_cur_.size() << " left, " << num_reset
         << " reset, " << num_outtime << " outtime and " << num_outlier << " outliers";
    return true;
}

/**
 * @brief 基于两个相机视图的三角化，计算三维点在世界坐标系下的坐标
 * @param pose0 第一个相机视图的位姿
 * @param pose1 第二个相机视图的位姿
 * @param pc0 第一个相机视角下的三维点相机坐标系的归一化坐标
 * @param pc1 第二个相机视角下的三维点相机坐标系的归一化坐标
 * @param pw (output)世界坐标系下的坐标
 */
void Tracking::triangulatePoint(const Eigen::Matrix<double, 3, 4> &pose0, const Eigen::Matrix<double, 3, 4> &pose1,
                                const Eigen::Vector3d &pc0, const Eigen::Vector3d &pc1, Eigen::Vector3d &pw) {
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();

    design_matrix.row(0) = pc0[0] * pose0.row(2) - pose0.row(0);    
    design_matrix.row(1) = pc0[1] * pose0.row(2) - pose0.row(1);
    design_matrix.row(2) = pc1[0] * pose1.row(2) - pose1.row(0);
    design_matrix.row(3) = pc1[1] * pose1.row(2) - pose1.row(1);

    Eigen::Vector4d point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    pw                    = point.head<3>() / point(3);
}

/**
 * @brief 检测像素点是否适合追踪。通常在视觉跟踪、三维重建和SLAM等计算机视觉任务是必要的。目的是排除一些可能引起误导或错误的点，如深度不正确或重投影误差过大的点
 * @param pp 像素点坐标
 * @param pose 位姿
 * @param pw 像素点对应的世界坐标系的坐标
 * @param scale 尺度因子，与重投影误差标准差阈值相乘
 * @param depth_scale 尺度因子，与深度最远阈值相乘
 * @return 判断量
 */
bool Tracking::isGoodToTrack(const cv::Point2f &pp, const Pose &pose, const Vector3d &pw, double scale,
                             double depth_scale) {
    // 当前相机坐标系
    Vector3d pc = camera_->world2cam(pw, pose);

    // 深度检查
    if (!isGoodDepth(pc[2], depth_scale)) {
        return false;
    }

    // 重投影误差检查
    if (camera_->reprojectionError(pose, pw, pp).norm() > reprojection_error_std_ * scale) {
        return false;
    }

    return true;
}

/**
 * @brief 被用于各种需要删除向量元素的场景。例如在计算机视觉中，可能需要一些条件来删除某些特征点
 * @param status 状态容器
 * @param vector (intput/output)作为要被删除元素的容器输入，以及经过删除元素之后的容器输出
 */
template <typename T> void Tracking::reduceVector(T &vec, vector<uint8_t> status) {
    size_t index = 0;
    for (size_t k = 0; k < vec.size(); k++) {
        if (status[k]) {
            vec[index++] = vec[k];
        }
    }
    vec.resize(index);
}

// 功能：
// 计算两像素间的距离
// 参数：
// pt1， pt2分别为像素点坐标
// 返回值
// 欧式距离
double Tracking::ptsDistance(cv::Point2f &pt1, cv::Point2f &pt2) {
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

// 功能：
// 判断像素点是否处于图像边缘位置上
// 参数：
// pts 像素点
// 返回值
// 判断结果bool型
bool Tracking::isOnBorder(const cv::Point2f &pts) {
    return pts.x < 5.0 || pts.y < 5.0 || (pts.x > (camera_->width() - 5.0)) || (pts.y > (camera_->height() - 5.0));
}

// 功能：
// 旋转与平移分离的位姿转化为变换矩阵的统一形式
// 参数：
// pose 旋转与平移分离的位姿形式
// 返回值：
// 表示位姿的变换矩阵Tcw
Eigen::Matrix4d Tracking::pose2Tcw(const Pose &pose) {
    Eigen::Matrix4d Tcw;
    Tcw.setZero();
    Tcw(3, 3) = 1;

    Tcw.block<3, 3>(0, 0) = pose.R.transpose();
    Tcw.block<3, 1>(0, 3) = -pose.R.transpose() * pose.t;
    return Tcw;
}

// 功能：
// 计算视差（视觉差异）
// 参数：
// pp0与pp1为相互匹配的像素点。pose0为pp0对应的位姿，pose1位pp1对应的位姿
// 返回值：
// 视差
double Tracking::keyPointParallax(const cv::Point2f &pp0, const cv::Point2f &pp1, const Pose &pose0,
                                  const Pose &pose1) {
    // 像素值到归一化相机坐标系坐标的转换                                    
    Vector3d pc0 = camera_->pixel2cam(pp0);
    Vector3d pc1 = camera_->pixel2cam(pp1);

    // 补偿掉旋转
    // 这里并没补偿位移，可能是因为作者把短时间运动形式限定为只有旋转
    Vector3d pc01 = pose1.R.transpose() * pose0.R * pc0;

    // 相差的像素大小
    return (pc01.head<2>() - pc1.head<2>()).norm() * camera_->focalLength();
}

// 功能：
// 根据参考帧的特征点计算参考帧与当前帧的视差
// 参数：
// parallax(output) 视差
// 返回值：
// 用于计算视差的特征点对数量
int Tracking::parallaxFromReferenceMapPoints(double &parallax) {

    parallax      = 0; // 视差初始化为0
    int counts    = 0; // 用于计算视差的特征点对数量初始化
    auto features = frame_ref_->features(); // 读取当前参考帧的所有特征点

    for (auto &feature : features) { 
        auto mappoint = feature.second->getMapPoint(); // 读取特征点对应的地图点
        if (mappoint && !mappoint->isOutlier()) {
            // 取最新的一个路标点观测
            auto observations = mappoint->observations(); // 读取该地图点的所有观测
            if (observations.empty()) {
                continue;
            }
            auto feat = observations.back().lock(); // 取路标点最新的观测
            if (feat && !feat->isOutlier()) {
                auto frame = feat->getFrame();
                if (frame && (frame == frame_cur_)) {
                    // 对应同一路标点在当前帧的像素观测
                    parallax += keyPointParallax(feature.second->keyPoint(), feat->keyPoint(), frame_ref_->pose(),
                                                 frame_cur_->pose());
                    counts++;
                }
            }
        }
    }

    if (counts != 0) {
        parallax /= counts;
    }

    return counts;
}

// 功能：
// 根据参考帧对应的参考点计算，参考帧与当前帧的视差
// 参数：
// ref 所有参考点（不仅包含当前参考帧上的点，还包括历史参考帧上的点）
// cur 当前帧与当前参考帧参考点匹配的特征点
// parallax(output) 当前参考帧与当前帧的视差
// 返回值：
// 用于视差计算的特征点对数量
int Tracking::parallaxFromReferenceKeyPoints(const vector<cv::Point2f> &ref, const vector<cv::Point2f> &cur,
                                             double &parallax) {
    parallax   = 0; // 视差初始化为0
    int counts = 0; // 用于计算视差的参考点数量
    // 对参考点对应的帧指针容器进行循环，如果参考点对应的帧为当前参考帧，则计算对应参考点和当前帧点的视差，并累加
    for (size_t k = 0; k < pts2d_ref_frame_.size(); k++) {
        if (pts2d_ref_frame_[k] == frame_ref_) {
            parallax += keyPointParallax(ref[k], cur[k], frame_ref_->pose(), frame_cur_->pose());
            counts++; // 累加用于视差计算的特征点对数量
        }
    }
    if (counts != 0) {
        parallax /= counts; 
    }

    return counts;
}
