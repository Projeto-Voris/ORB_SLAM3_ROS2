#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sys/stat.h> // Para mkdir

#include "rclcpp/rclcpp.hpp"
#include "monocular.hpp"

#include "System.h"


namespace orbslam3_ros2
{
MonocularSlamNode::MonocularSlamNode(const rclcpp::NodeOptions & options)
:   SlamNode(nullptr, options)
{
    RCLCPP_INFO(this->get_logger(), "Inicializando MonocularSlamNode...");
    this->declare_parameter<std::string>("voc_file", "");
    this->declare_parameter<std::string>("settings_file", "");
    this->declare_parameter("resize_factor", 0.25);
    this->declare_parameter<bool>("clahe", false);

    std::string strVocFile= this->get_parameter("voc_file").as_string();
    std::string strSettingsFile = this->get_parameter("settings_file").as_string(); 
    this->get_parameter("clahe", apply_clahe);

    if (strVocFile.empty() || strSettingsFile.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Fill 'voc_file' and 'settings_file' parameters");
        rclcpp::shutdown();
        return;
    }

    if (apply_clahe){
        clahe_->setClipLimit(2.0);
        clahe_->setTilesGridSize(cv::Size(5, 5));
    }

    // ORB_SLAM3::System::STEREO = 1
    m_SLAM = new ORB_SLAM3::System(strVocFile, strSettingsFile, ORB_SLAM3::System::MONOCULAR, false);
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
}

MonocularSlamNode::~MonocularSlamNode()
{
    m_SLAM->Shutdown();
}

void MonocularSlamNode::GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat img;
    double resize = this->get_parameter("resize_factor").as_double();

    try
    {
        img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;
        cv::resize(img, img, cv::Size(), resize, resize, cv::INTER_LINEAR);

        if(apply_clahe){
            clahe_->apply(img, img);
        }
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // std::cout<<"one frame has been sent"<<std::endl;
    current_frame_time_ = now();
    SE3 = m_SLAM->TrackMonocular(img, Utility::StampToSec(msg->header.stamp));
    Update();
    TrackedImage(img);
}
}
RCLCPP_COMPONENTS_REGISTER_NODE(orbslam3_ros2::MonocularSlamNode);