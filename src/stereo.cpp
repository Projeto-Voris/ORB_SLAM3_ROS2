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



StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, rclcpp::Node* node, rclcpp::NodeOptions options, const std::string &strSettingsFile, const std::string &strDoRectify)
: SlamNode(pSLAM, node, options)
{
    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;
    this->declare_parameter<bool>("rescale", false);
    this->get_parameter("rescale", rescale);

    left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "camera/left");
    right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "camera/right");
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
}

StereoSlamNode::~StereoSlamNode() 
{
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
        cv::resize(imLeft.clone(), imgLeft, cv::Size(800,600), cv::INTER_LINEAR);
        cv::resize(imRight.clone(), imgRight, cv::Size(800,600), cv::INTER_LINEAR);
    }
    std::stringstream left, right;
    left << "pid: " << GETPID() << ", ptr: " << msgLeft.get();
    right << "pid: " << GETPID() << ", ptr: " << msgRight.get();
    RCLCPP_WARN(this->get_logger(), "l: %s, r: %s", left.str().c_str(), right.str().c_str());

    SE3 = m_SLAM->TrackStereo(imgLeft, imgRight, Utility::StampToSec(msgLeft->header.stamp));
    current_frame_time_ = now();
    Update();
    TrackedImage(imgLeft);
}


