#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sys/stat.h>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "stereo.hpp"

#include <opencv2/core/core.hpp>
#include "System.h"

using std::placeholders::_1;
using std::placeholders::_2;

#include <unistd.h>
#define GETPID getpid


namespace orbslam3_ros2
{
StereoSlamNode::StereoSlamNode(const rclcpp::NodeOptions & options)
: SlamNode(nullptr, options)
{
    RCLCPP_INFO(this->get_logger(), "Inicializando StereoSlamNode...");
    this->declare_parameter<std::string>("voc_file", "");
    this->declare_parameter<std::string>("settings_file", "");
    this->declare_parameter<bool>("do_rectify", true);
    this->declare_parameter<bool>("rescale", false);
    std::string strVocFile= this->get_parameter("voc_file").as_string();
    std::string strSettingsFile = this->get_parameter("settings_file").as_string(); 
    this->get_parameter("rescale", rescale);
    this->get_parameter("do_rectify", doRectify);

    if (strVocFile.empty() || strSettingsFile.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Fill 'voc_file' and 'settings_file' parameters");
        rclcpp::shutdown();
        return;
    }

    // ORB_SLAM3::System::STEREO = 1
    m_SLAM = new ORB_SLAM3::System(strVocFile, strSettingsFile, ORB_SLAM3::System::STEREO, false);
    
    RCLCPP_INFO(this->get_logger(), "ORB_SLAM3 System Inicializado!");

    left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "camera/left");
    right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "camera/right");
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
}

StereoSlamNode::~StereoSlamNode() 
{
    m_SLAM->Shutdown();
}

void StereoSlamNode::GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr msgLeft, const sensor_msgs::msg::Image::ConstSharedPtr msgRight) {
    cv::Mat imgLeft, imgRight;
    try {
        imLeft = cv_bridge::toCvShare(msgLeft, msgLeft->encoding)->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    try {
        imRight = cv_bridge::toCvShare(msgRight, msgLeft->encoding)->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    if (rescale){
        cv::resize(imLeft.clone(), imgLeft, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
        cv::resize(imRight.clone(), imgRight, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
    }
    std::stringstream left, right;
    left << "pid: " << GETPID() << ", ptr: " << msgLeft.get();
    right << "pid: " << GETPID() << ", ptr: " << msgRight.get();
    // RCLCPP_WARN(this->get_logger(), "l: %s, r: %s", left.str().c_str(), right.str().c_str());

    SE3 = m_SLAM->TrackStereo(imgLeft, imgRight, Utility::StampToSec(msgLeft->header.stamp));
    current_frame_time_ = now();
    Update();
    TrackedImage(imgLeft);
}
}
RCLCPP_COMPONENTS_REGISTER_NODE(orbslam3_ros2::StereoSlamNode);


