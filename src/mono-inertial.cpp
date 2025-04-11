#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "mono-inertial-node.hpp"

#include "System.h"

using std::placeholders::_1;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings visualization" << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    bool visualization = true;
    
    auto node = std::make_shared<rclcpp::Node>("orb_slam");

    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, visualization);

    std::shared_ptr<MonoInertialNode> slam_ros;
    slam_ros = std::make_shared<MonoInertialNode>(&pSLAM, node.get());
    std::cout << "============================" << std::endl;

    rclcpp::spin(slam_ros->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
MonoInertialNode::MonoInertialNode(ORB_SLAM3::System *pSLAM, rclcpp::Node* node) :
    SlamNode(pSLAM, node)
{
    subImu_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 100, std::bind(&MonoInertialNode::GrabImu, this, _1));
    subImg_ = this->create_subscription<sensor_msgs::msg::Image>("camera", 10, std::bind(&MonoInertialNode::GrabImage, this, _1));

    syncThread_ = new std::thread(&MonoInertialNode::SyncWithImu, this);

}

MonoInertialNode::~MonoInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    //SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
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

    if (cv_ptr->image.type() == 0)
    {
        return resized_img;
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return resized_img;
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
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }


    }
}
