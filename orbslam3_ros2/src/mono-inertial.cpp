#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sys/stat.h> // Para mkdir

#include "rclcpp/rclcpp.hpp"
#include "mono_inertial.hpp"

#include "System.h"

namespace orbslam3_ros2
{
MonoInertialNode::MonoInertialNode(const rclcpp::NodeOptions & options) :
    SlamNode(nullptr, options)
{
    RCLCPP_INFO(this->get_logger(), "Inicializando CompressedSlamNode...");
    this->declare_parameter<std::string>("voc_file", "");
    this->declare_parameter<std::string>("settings_file", "");
    this->declare_parameter<bool>("clahe", false);
    this->declare_parameter("resize_factor", 0.25);

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
    m_SLAM = new ORB_SLAM3::System(strVocFile, strSettingsFile, ORB_SLAM3::System::IMU_MONOCULAR, false);
    auto imu_qos_profile = rclcpp::SensorDataQoS();
    auto img_qos_profile = rclcpp::SensorDataQoS();
    imu_qos_profile.keep_last(100);
    img_qos_profile.keep_last(10);

    subImu_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", imu_qos_profile, std::bind(&MonoInertialNode::GrabImu, this, std::placeholders::_1));
    subImg_ = this->create_subscription<sensor_msgs::msg::Image>("camera", img_qos_profile, std::bind(&MonoInertialNode::GrabImage, this, std::placeholders::_1));

    syncThread_ = new std::thread(&MonoInertialNode::SyncWithImu, this);

}

MonoInertialNode::~MonoInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    m_SLAM->Shutdown();
}

void MonoInertialNode::GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "IMU received");
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void MonoInertialNode::GrabImage(const sensor_msgs::msg::Image::SharedPtr msgLeft)
{
    // RCLCPP_INFO(this->get_logger(), "Left image received");
    bufMutexImg_.lock();

    if (!imgBuf_.empty())
        imgBuf_.pop();
    imgBuf_.push(msgLeft);

    bufMutexImg_.unlock();
}

cv::Mat MonoInertialNode::GetImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    double resize = this->get_parameter("resize_factor").as_double();
    cv::Mat img;

    try
    {
        img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;
        cv::resize(img, img, cv::Size(), resize, resize, cv::INTER_LINEAR);
        
        if(apply_clahe){
            clahe_->apply(img, img);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void MonoInertialNode::SyncWithImu()
{
    while (1)
    {
        
        cv::Mat img;
        double tImg = 0;
        if (!imgBuf_.empty() && !imuBuf_.empty())
        {
            tImg = Utility::StampToSec(imgBuf_.front()->header.stamp);

            if (tImg > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;
            {
            bufMutexImg_.lock();
            img = GetImage(imgBuf_.front());
            imgBuf_.pop();
            bufMutexImg_.unlock();
            }
            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImg)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();
            current_frame_time_ = now();
            SE3 = m_SLAM->TrackMonocular(img, tImg, vImuMeas);
            
            Update();
            TrackedImage(img);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }


    }
}
}
RCLCPP_COMPONENTS_REGISTER_NODE(orbslam3_ros2::MonoInertialNode);