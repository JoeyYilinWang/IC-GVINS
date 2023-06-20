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

#include "tracking/camera.h"

Camera::Camera(Mat intrinsic, Mat distortion, const cv::Size &size)
    : distortion_(std::move(distortion))
    , intrinsic_(std::move(intrinsic)) {

    fx_   = intrinsic_.at<double>(0, 0);
    skew_ = intrinsic_.at<double>(0, 1);
    cx_   = intrinsic_.at<double>(0, 2);
    fy_   = intrinsic_.at<double>(1, 1);
    cy_   = intrinsic_.at<double>(1, 2);

    k1_ = distortion_.at<double>(0);
    k2_ = distortion_.at<double>(1);
    p1_ = distortion_.at<double>(2);
    p2_ = distortion_.at<double>(3);
    k3_ = distortion_.at<double>(4);

    width_  = size.width;
    height_ = size.height;

    // 相机畸变矫正初始化
    // chatgpt:
    // undissrc_是对应x方向的像素映射；undisdst_对应y方向的像素映射
    initUndistortRectifyMap(intrinsic_, distortion_, Mat(), intrinsic_, size, CV_16SC2, undissrc_, undisdst_);
}

Camera::Ptr Camera::createCamera(const std::vector<double> &intrinsic, const std::vector<double> &distortion,
                                 const std::vector<int> &size) {
    // Intrinsic matrix
    Mat intrinsic_mat;
    if (intrinsic.size() == 4) {
        intrinsic_mat =
            (cv::Mat_<double>(3, 3) << intrinsic[0], 0, intrinsic[2], 0, intrinsic[1], intrinsic[3], 0, 0, 1);
    }
     // Distortion parameters
    Mat distortion_mat;
    if (distortion.size() == 4) {
        distortion_mat = (cv::Mat_<double>(5, 1) << distortion[0], distortion[1], distortion[2], distortion[3], 0.0);
    } else if (distortion.size() == 5) {
        distortion_mat =
            (cv::Mat_<double>(5, 1) << distortion[0], distortion[1], distortion[2], distortion[3], distortion[4]);
    }

    return std::make_shared<Camera>(intrinsic_mat, distortion_mat, cv::Size(size[0], size[1]));
}

// 将点进行矫正，转换为矫正过的版本
void Camera::undistortPoints(std::vector<cv::Point2f> &pts) {
    cv::undistortPoints(pts, pts, intrinsic_, distortion_, Mat(), intrinsic_);
}

// ChatGPT:
// 对多点进行径向畸变矫正，转化为矫正过的版本
void Camera::distortPoints(std::vector<cv::Point2f> &pts) const {
    for (auto &pt : pts) {
        auto pc   = pixel2cam(pt);
        double x  = pc.x();
        double y  = pc.y();
        double r2 = x * x + y * y;
        double rr = (1 + k1_ * r2 + k2_ * r2 * r2 + k3_ * r2 * r2 * r2);

        pc.x() = x * rr + 2 * p1_ * x * y + p2_ * (r2 + 2 * x * x);
        pc.y() = y * rr + p1_ * (r2 + 2 * y * y) + 2 * p2_ * x * y;

        pt = cam2pixel(pc);
    }
}

// ChatGPT:
// 对单点进行径向畸变矫正，转化为矫正过的版本
void Camera::distortPoint(cv::Point2f &pp) const {
    auto pc   = pixel2cam(pp);
    double x  = pc.x();
    double y  = pc.y();
    double r2 = x * x + y * y;
    double rr = (1 + k1_ * r2 + k2_ * r2 * r2 + k3_ * r2 * r2 * r2);

    pc.x() = x * rr + 2 * p1_ * x * y + p2_ * (r2 + 2 * x * x);
    pc.y() = y * rr + p1_ * (r2 + 2 * y * y) + 2 * p2_ * x * y;

    pp = cam2pixel(pc);
}

// ChatGPT:
// 在相机坐标和像素坐标之间进行转换和畸变校正，以便在图像上准确地定位相机坐标对应的点。
cv::Point2f Camera::distortCameraPoint(const Vector3d &pc) const {
    Vector3d pc1;

    double x  = pc.x() / pc.z();
    double y  = pc.y() / pc.z();
    double r2 = x * x + y * y;
    double rr = (1 + k1_ * r2 + k2_ * r2 * r2 + k3_ * r2 * r2 * r2);

    pc1.x() = static_cast<float>(x * rr + 2 * p1_ * x * y + p2_ * (r2 + 2 * x * x));
    pc1.y() = static_cast<float>(y * rr + p1_ * (r2 + 2 * y * y) + 2 * p2_ * x * y);
    pc1.z() = 1.0;

    return cam2pixel(pc1);
}

// ChatGPT:
// 将输入图像中的畸变进行修复，得到矫正后的输出图像
void Camera::undistortImage(const Mat &src, Mat &dst) {
    cv::remap(src, dst, undissrc_, undisdst_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
}

// 像素坐标恢复到归一化相机坐标
Vector3d Camera::pixel2cam(const cv::Point2f &pixel) const {
    double y = (pixel.y - cy_) / fy_;
    double x = (pixel.x - cx_ - skew_ * y) / fx_;
    return {x, y, 1.0};
}

// 相机坐标投影至像素坐标
cv::Point2f Camera::cam2pixel(const Vector3d &cam) const {
    return cv::Point2f((fx_ * cam[0] + skew_ * cam[1]) / cam[2] + cx_, fy_ * cam[1] / cam[2] + cy_);
}

// 将像素坐标转化为相机坐标系下的单位向量 
Vector3d Camera::pixel2unitcam(const cv::Point2f &pixel) const {
    return pixel2cam(pixel).normalized();
}

// 将像素坐标转化为世界坐标系下的坐标
Vector3d Camera::pixel2world(const cv::Point2f &pixel, const Pose &pose) const {
    return cam2world(pixel2cam(pixel), pose);
}

// 将世界坐标系下的坐标转化为像素坐标
cv::Point2f Camera::world2pixel(const Vector3d &world, const Pose &pose) const {
    return cam2pixel(world2cam(world, pose));
}

// 世界坐标系坐标转化为相机坐标系坐标 
Vector3d Camera::world2cam(const Vector3d &world, const Pose &pose) {
    return pose.R.transpose() * (world - pose.t);
}

// 相机坐标系坐标转化为世界坐标系坐标
Vector3d Camera::cam2world(const Vector3d &cam, const Pose &pose) {
    return pose.R * cam + pose.t;
}

/*
@brief 计算重投影误差
@param pose 要评估正确性的位姿
@param pw 世界坐标系下的点坐标
@param pp 像素坐标系下的坐标
@return vector<int>{error along x, error along y} 
*/
Vector2d Camera::reprojectionError(const Pose &pose, const Vector3d &pw, const cv::Point2f &pp) const {
    cv::Point2f ppp = world2pixel(pw, pose);

    return {ppp.x - pp.x, ppp.y - pp.y};
}
