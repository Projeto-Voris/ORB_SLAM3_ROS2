#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sys/stat.h>
#include <string>
#include <iomanip>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "stereo_inertial.hpp"
#include "System.h"

using std::placeholders::_1;



namespace orbslam3_ros2
{
StereoInertialNode::StereoInertialNode(const rclcpp::NodeOptions & options) :
    SlamNode(nullptr, options)
{
    RCLCPP_INFO(this->get_logger(), "Inicializando StereoInertialSlamNode...");
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
    m_SLAM = new ORB_SLAM3::System(strVocFile, strSettingsFile, ORB_SLAM3::System::IMU_STEREO, false);
    

    subImu_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 100, std::bind(&StereoInertialNode::GrabImu, this, _1));
    subImgLeft_ = this->create_subscription<sensor_msgs::msg::Image>("camera/left", 10, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    subImgRight_ = this->create_subscription<sensor_msgs::msg::Image>("camera/right", 10, std::bind(&StereoInertialNode::GrabImageRight, this, _1));
    
    syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);
}

StereoInertialNode::~StereoInertialNode()
{
    syncThread_->join();
    delete syncThread_;
    m_SLAM->Shutdown();
}

void StereoInertialNode::GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void StereoInertialNode::GrabImageLeft(const sensor_msgs::msg::Image::SharedPtr msgLeft)
{
    bufMutexLeft_.lock();
    if (!imgLeftBuf_.empty())
        imgLeftBuf_.pop();
    imgLeftBuf_.push(msgLeft);
    bufMutexLeft_.unlock();
}

void StereoInertialNode::GrabImageRight(const sensor_msgs::msg::Image::SharedPtr msgRight)
{
    bufMutexRight_.lock();
    if (!imgRightBuf_.empty())
        imgRightBuf_.pop();
    imgRightBuf_.push(msgRight);
    bufMutexRight_.unlock();
}

cv::Mat StereoInertialNode::GetImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    cv::Mat resized_img;
    rescale = this->get_parameter("rescale").as_bool();
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        if (rescale){
        cv::resize(cv_ptr->image, resized_img, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
        }
        else{
            resized_img = cv_ptr->image;
        }
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    return resized_img;
}

void StereoInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.05;

    while (1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf_.empty() && !imgRightBuf_.empty() && !imuBuf_.empty())
        {
            tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);

            bufMutexRight_.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1)
            {
                imgRightBuf_.pop();
                tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
            }
            bufMutexRight_.unlock();

            bufMutexLeft_.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1)
            {
                imgLeftBuf_.pop();
                tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            }
            bufMutexLeft_.unlock();

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                RCLCPP_WARN(this->get_logger(), "dt dif: %f", std::min(abs(tImLeft - tImRight), abs(tImRight - tImLeft)));
                continue;
            }
            if (tImLeft > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            bufMutexLeft_.lock();
            imLeft = GetImage(imgLeftBuf_.front());
            imgLeftBuf_.pop();
            bufMutexLeft_.unlock();

            bufMutexRight_.lock();
            imRight = GetImage(imgRightBuf_.front());
            imgRightBuf_.pop();
            bufMutexRight_.unlock();
            
            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImLeft)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            SE3 = m_SLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
            
            // CSV Logging
            
            Update();
            TrackedImage(imLeft);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
}
RCLCPP_COMPONENTS_REGISTER_NODE(orbslam3_ros2::StereoInertialNode);
