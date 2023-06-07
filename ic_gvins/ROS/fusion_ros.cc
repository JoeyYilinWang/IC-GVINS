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

#include "fusion_ros.h"
#include "drawer_rviz.h"

#include "ic_gvins/common/angle.h"
#include "ic_gvins/common/gpstime.h"
#include "ic_gvins/common/logging.h"
#include "ic_gvins/misc.h"
#include "ic_gvins/tracking/frame.h"

#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <sensor_msgs/image_encodings.h>

#include <atomic>
#include <csignal>
#include <memory>

std::atomic<bool> isfinished{false}; // 全局变量，设为原子类型，用于多线程工作时的数据保护

void sigintHandler(int sig); 
void checkStateThread(std::shared_ptr<FusionROS> fusion); // 查看线程的状态

void FusionROS::setFinished() {
    if (gvins_ && gvins_->isRunning()) {
        gvins_->setFinished();
    }
}

void FusionROS::run() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); // 用于读取参数的NodeHandle 

    // message topic
    string imu_topic, gnss_topic, image_topic, livox_topic;
    // 从parameters库中读取参数，没有的话使用默认值。
    pnh.param<string>("imu_topic", imu_topic, "/imu0");  
    pnh.param<string>("gnss_topic", gnss_topic, "/gnss0");
    pnh.param<string>("image_topic", image_topic, "/cam0");

    // GVINS parameter
    string configfile;
    pnh.param<string>("configfile", configfile, "gvins.yaml");

    // Load configurations
    YAML::Node config;
    std::vector<double> vecdata;
    try {
        config = YAML::LoadFile(configfile);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file" << std::endl;
        return;
    }
    auto outputpath        = config["outputpath"].as<string>();
    auto is_make_outputdir = config["is_make_outputdir"].as<bool>();

    // Create the output directory
    if (!boost::filesystem::is_directory(outputpath)) {
        boost::filesystem::create_directory(outputpath);
    }
    // Make sure the directory has been created
    if (!boost::filesystem::is_directory(outputpath)) {
        std::cout << "Failed to open outputpath" << std::endl;
        return;
    }
    
    if (is_make_outputdir) {
        absl::CivilSecond cs = absl::ToCivilSecond(absl::Now(), absl::LocalTimeZone());
        absl::StrAppendFormat(&outputpath, "/T%04d%02d%02d%02d%02d%02d", cs.year(), cs.month(), cs.day(), cs.hour(),
                              cs.minute(), cs.second());
        boost::filesystem::create_directory(outputpath);
    }

    // GNSS outage configurations，不知道GNSS outage参数的作用
    isusegnssoutage_ = config["isusegnssoutage"].as<bool>();
    gnssoutagetime_  = config["gnssoutagetime"].as<double>();
    gnssthreshold_   = config["gnssthreshold"].as<double>(); 

    // Glog output path
    FLAGS_log_dir = outputpath;

    // 跟rviz画图显示有关，Drawer作为基类可用于其派生类的初始化工作
    Drawer::Ptr drawer = std::make_shared<DrawerRviz>(nh);
    // GVINS初始化，包括配置文件参数读取，结果文件输出路径设置，设置三个线程并开始工作
    gvins_             = std::make_shared<GVINS>(configfile, outputpath, drawer);

    // check is initialized
    if (!gvins_->isRunning()) { 
        LOGE << "Fusion ROS terminate";
        return;
    }

    // subscribe message
    // 同一个节点订阅不同的topic，不同topic产生不同消息，并存放在各自的队列中，各消息队列都对应各自回调函数地址
    // 在没有特殊操作下，节点调用回调函数是单线程的
    // 有些特殊操作是允许节点以多线程方式操作不同回调函数的，但这里不涉及
    ros::Subscriber imu_sub   = nh.subscribe<sensor_msgs::Imu>(imu_topic, 200, &FusionROS::imuCallback, this);
    ros::Subscriber gnss_sub  = nh.subscribe<sensor_msgs::NavSatFix>(gnss_topic, 1, &FusionROS::gnssCallback, this);
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>(image_topic, 20, &FusionROS::imageCallback, this);

    LOGI << "Waiting ROS message...";

    // enter message loopback，不断调用回调函数处理实时接收到的消息
    ros::spin(); 
}

// 由于imu消息提供频率很快，因此相同时间内会被调用更多次
void FusionROS::imuCallback(const sensor_msgs::ImuConstPtr &imumsg) {
    imu_pre_ = imu_;
    // Time convertion，将晶振时间转化为秒
    double unixsecond = imumsg->header.stamp.toSec();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec); // 将晶振时间(以s为单位)转化为GNSS的精准时钟
    
    imu_.time = weeksec;  // 将imu_的晶振时间转换为GNSS采用的周秒
    // delta time
    imu_.dt = imu_.time - imu_pre_.time; // imu_与前一时刻的imu_pre_的时间差

    // IMU measurements, Front-Right-Down，将角速度和比力转化为增量（用于机械编排算法使用）
    imu_.dtheta[0] = imumsg->angular_velocity.x * imu_.dt;
    imu_.dtheta[1] = imumsg->angular_velocity.y * imu_.dt;
    imu_.dtheta[2] = imumsg->angular_velocity.z * imu_.dt;
    imu_.dvel[0]   = imumsg->linear_acceleration.x * imu_.dt;
    imu_.dvel[1]   = imumsg->linear_acceleration.y * imu_.dt;
    imu_.dvel[2]   = imumsg->linear_acceleration.z * imu_.dt; 

    // 此时当前时刻的imu_数据是imumsg经过转化的数据了

    // Not ready
    if (imu_pre_.time == 0) { // 一般情况下周秒不可能为0，所以若为0则表示还未准备好，这里直接返回。
        return;
    }
    
    imu_buffer_.push(imu_); // imu_buffer_存储imu当前数据imu_
    while (!imu_buffer_.empty()) {
        auto imu = imu_buffer_.front(); // 取最前面的imu数据，实际上就是刚才压入的IMU数据
        
        // Add new IMU to GVINS
        if (gvins_->addNewImu(imu)) { // 将队列最前的IMU数据输入到gvins中处理
            imu_buffer_.pop(); // 如果gvins成功读入该数据则把该数据pop出来
        } else {
            // Thread lock failed, try next time
            break; // 如果gvins模块没有成功得到该imu数据，则重来一遍
        }
    }
}

void FusionROS::gnssCallback(const sensor_msgs::NavSatFixConstPtr &gnssmsg) {
    // Time convertion
    double unixsecond = gnssmsg->header.stamp.toSec(); // gnssmsg中提供的时间也是unix时间计时的
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    gnss_.time = weeksec; // 将gnss数据时间设置为GNSS周时

    gnss_.blh[0] = gnssmsg->latitude * D2R; // 纬度转化为弧度
    gnss_.blh[1] = gnssmsg->longitude * D2R; // 经度转化为弧度
    gnss_.blh[2] = gnssmsg->altitude; // 海拔高度获取

    // gnss_.std衡量定位不确定性指标
    gnss_.std[0] = sqrt(gnssmsg->position_covariance[4]); // N 
    gnss_.std[1] = sqrt(gnssmsg->position_covariance[0]); // E
    gnss_.std[2] = sqrt(gnssmsg->position_covariance[8]); // D

    gnss_.isyawvalid = false;

    // Exception，异常值，因为不可能为0
    if ((gnss_.std[0] == 0) || (gnss_.std[1] == 0) || (gnss_.std[2] == 0)) {
        return;
    }

    // Remove bad GNSS
    bool isoutage = false;

    // 只有不确定性小于阈值了才说明当前gnss数据是可靠的
    if ((gnss_.std[0] < gnssthreshold_) && (gnss_.std[1] < gnssthreshold_) && (gnss_.std[2] < gnssthreshold_)) {

        // 在配置文件中已经将isusegnssoutage_设置为false，因此该代码段是没用的。
        if (isusegnssoutage_ && (weeksec >= gnssoutagetime_)) { 
            isoutage = true;
        }

        // add new GNSS to GVINS
        if (!isoutage) {
            gvins_->addNewGnss(gnss_);
        }
    }
}

void FusionROS::imageCallback(const sensor_msgs::ImageConstPtr &imagemsg) {
    Mat image;
    // Copy image data
    if (imagemsg->encoding == sensor_msgs::image_encodings::MONO8) {
        image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC1);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width);
    } else if (imagemsg->encoding == sensor_msgs::image_encodings::BGR8) {
        image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC3);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width * 3);
        // ROS_INFO("image is RGB format");
    }

    // Time convertion
    double unixsecond = imagemsg->header.stamp.toSec();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    // Add new Image to GVINS
    frame_ = Frame::createFrame(weeksec, image);
    
    frame_buffer_.push(frame_);
    while (!frame_buffer_.empty()) {
        // ROS_INFO("frame_buffer size is: %ld", frame_buffer_.size()); 
        auto frame = frame_buffer_.front();
        if (gvins_->addNewFrame(frame)) {
            frame_buffer_.pop();
            // ROS_INFO("After pop operation, frame buffer size is: %ld", frame_buffer_.size()); 
        } else {
            break;
        }
    }

    // 纪录所有模态的数据输入时间
    LOG_EVERY_N(INFO, 20) << "Raw data time " << Logging::doubleData(imu_.time) << ", "
                          << Logging::doubleData(gnss_.time) << ", " << Logging::doubleData(frame_->stamp());
}

// 信号处理函数
void sigintHandler(int sig) {
    std::cout << "Terminate by Ctrl+C " << sig << std::endl;
    isfinished = true; 
}

void checkStateThread(std::shared_ptr<FusionROS> fusion) {
    std::cout << "Check thread is started..." << std::endl;

    auto fusion_ptr = std::move(fusion); // 内存拷贝，并不重新分配内存
    while (!isfinished) { // 程序开始时isfinished默认为false，因此会不断执行sleep(1)沉睡1s语句。等到isfinished变为true时再执行后面的语句
        sleep(1);
    }

    // Exit the GVINS thread
    fusion_ptr->setFinished();

    std::cout << "GVINS has been shutdown ..." << std::endl;

    // Shutdown ROS
    ros::shutdown();

    std::cout << "ROS node has been shutdown ..." << std::endl;
}

int main(int argc, char *argv[]) {
    // Glog initialization，用于报告程序运行状况
    Logging::initialization(argv, true, true);

    // ROS node named "gvins_node"，不使用默认整数信号handler
    ros::init(argc, argv, "gvins_node", ros::init_options::NoSigintHandler);

    // Register signal handler，宏SIGINT==2，是"Ctrl+C"的标志。信号函数sigintHandler用于接收这个宏并做出一系列操作
    std::signal(SIGINT, sigintHandler);

    auto fusion = std::make_shared<FusionROS>(); // FusionROS的构造函数是DEFAULT
    
    // Check thread，该线程一直在运行，时刻监控是否接收到中断信号。若受到中断信号，直接关闭ROS。
    std::thread check_thread(checkStateThread, fusion); 

    // 这里没有用到check_thread.join()，join()函数表示主函数阻塞，等待该线程运行完成。
    // 这里没用到join()的原因是checkStateThread函数中有一条while循环语句: sleep(1)，使该线程阻塞。如果使用join()则会使主函数永远阻塞下去


    std::cout << "Fusion process is started..." << std::endl;

    // Enter message loop
    fusion->run();

    return 0;
}
