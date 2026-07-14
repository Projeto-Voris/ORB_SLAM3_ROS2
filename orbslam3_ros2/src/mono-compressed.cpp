#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sys/stat.h> // Para mkdir

#include "rclcpp/rclcpp.hpp"
#include "monocular_compressed.hpp"

#include "System.h"


namespace orbslam3_ros2
{
MonocularCompressedSlamNode::MonocularCompressedSlamNode(const rclcpp::NodeOptions & options)
:   SlamNode(nullptr, options)
{
    RCLCPP_INFO(this->get_logger(), "Inicializando CompressedSlamNode...");
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
    m_image_subscriber = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "camera/compressed",
        10,
        std::bind(&MonocularCompressedSlamNode::GrabImage, this, std::placeholders::_1));
    }

MonocularCompressedSlamNode::~MonocularCompressedSlamNode()
{
    m_SLAM->Shutdown();
}

void MonocularCompressedSlamNode::GrabImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // Copy the ros compressed image message to cv::Mat.
    double resize = this->get_parameter("resize_factor").as_double();
    cv::Mat img;
    try
    {
        img_cam = cv_bridge::toCvCopy(msg, "mono8")->image;
        cv::resize(img_cam, img, cv::Size(), resize, resize, cv::INTER_LINEAR);
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
    SE3 = m_SLAM->TrackMonocular(img_cam, Utility::StampToSec(msg->header.stamp));
    Update();
    TrackedImage(img_cam);
}
}
RCLCPP_COMPONENTS_REGISTER_NODE(orbslam3_ros2::MonocularCompressedSlamNode);
