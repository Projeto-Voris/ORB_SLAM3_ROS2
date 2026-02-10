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





StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *pSLAM, rclcpp::Node* node ,const std::string &strSettingsFile, const std::string &strDoRectify, const std::string &strDoEqual) :
    SlamNode(pSLAM, node)
{
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    subImu_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 100, std::bind(&StereoInertialNode::GrabImu, this, _1));
    subImgLeft_ = this->create_subscription<sensor_msgs::msg::Image>("camera/left", 10, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    subImgRight_ = this->create_subscription<sensor_msgs::msg::Image>("camera/right", 10, std::bind(&StereoInertialNode::GrabImageRight, this, _1));
    
    syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);
}

StereoInertialNode::~StereoInertialNode()
{
    syncThread_->join();
    delete syncThread_;
    SLAM_->Shutdown();
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
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

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        cv::resize(cv_ptr->image, resized_img, cv::Size(800, 600), cv::INTER_LINEAR);
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

            if (bClahe_)
            {
                clahe_->apply(imLeft, imLeft);
                clahe_->apply(imRight, imRight);
            }

            if (doRectify_)
            {
                cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
            }

            SE3 = m_SLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
            
            // CSV Logging
            
            Update();
            TrackedImage(imLeft);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}