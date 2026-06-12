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

    auto slam_cb_group = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

    // 2. Configure as opções de inscrição
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = slam_cb_group;

    RCLCPP_INFO(this->get_logger(), "Inicializando StereoSlamNode...");
    this->declare_parameter<std::string>("voc_file", "");
    this->declare_parameter<std::string>("settings_file", "");
    this->declare_parameter("resize_factor", 0.25);
    this->declare_parameter<bool>("clahe", true);
    this->declare_parameter<bool>("do_rectify", true);
    std::string strVocFile= this->get_parameter("voc_file").as_string();
    std::string strSettingsFile = this->get_parameter("settings_file").as_string(); 
    this->get_parameter("do_rectify", doRectify);
    this->get_parameter("clahe", apply_clahe);

    if (strVocFile.empty() || strSettingsFile.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Fill 'voc_file' and 'settings_file' parameters");
        rclcpp::shutdown();
        return;
    }

    if (apply_clahe){
        clahe_->setClipLimit(5.0);
        clahe_->setTilesGridSize(cv::Size(5, 5));
    }

    // ORB_SLAM3::System::STEREO = 1
    RCLCPP_INFO(this->get_logger(), "Rectify images: %d", doRectify);
    m_SLAM = new ORB_SLAM3::System(strVocFile, strSettingsFile, ORB_SLAM3::System::STEREO, false);
    
    RCLCPP_INFO(this->get_logger(), "ORB_SLAM3 System Inicializado!");

    left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "camera/left", rmw_qos_profile_sensor_data, sub_options);
    right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "camera/right", rmw_qos_profile_sensor_data, sub_options);
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
}

StereoSlamNode::~StereoSlamNode() 
{
    m_SLAM->Shutdown();
}

void StereoSlamNode::GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr msgLeft, const sensor_msgs::msg::Image::ConstSharedPtr msgRight) {
    cv::Mat imgLeft, imgRight;
    double resize = this->get_parameter("resize_factor").as_double();
    try {
        imLeft = cv_bridge::toCvShare(msgLeft, sensor_msgs::image_encodings::MONO8)->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    try {
        imRight = cv_bridge::toCvShare(msgRight, sensor_msgs::image_encodings::MONO8)->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::resize(imLeft.clone(), imgLeft, cv::Size(), resize, resize, cv::INTER_LINEAR);
    cv::resize(imRight.clone(), imgRight, cv::Size(), resize, resize, cv::INTER_LINEAR);

    if(apply_clahe){
        clahe_->apply(imgLeft, imgLeft);
        clahe_->apply(imgRight, imgRight);
    }

    SE3 = m_SLAM->TrackStereo(imgLeft, imgRight, Utility::StampToSec(msgLeft->header.stamp));
    current_frame_time_ = now();
    Update();
    TrackedImage(imgLeft);
}
}
RCLCPP_COMPONENTS_REGISTER_NODE(orbslam3_ros2::StereoSlamNode);


