#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sys/stat.h> // Para mkdir

#include "rclcpp/rclcpp.hpp"
#include "mono_inertial_compressed.hpp"

#include "System.h"

using std::placeholders::_1;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    for (int i = 4; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--with-saver" || a == "--inproc-saver") {
            run_saver_in_process = true;
        }
    }
    
    auto node = std::make_shared<rclcpp::Node>("orb_slam");
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(run_saver_in_process); // Enable intra-process communication if saver is in the same process


    auto slam_node = std::make_shared<MonoInertialCompressedNode>(options);
    std::cout << "============================" << std::endl;

    rclcpp::spin(slam_node);
    rclcpp::shutdown();

    return 0;
}

MonoInertialCompressedNode::MonoInertialCompressedNode(const rclcpp::NodeOptions & options) :
    SlamNode(nullptr, options)
{
    RCLCPP_INFO(this->get_logger(), "Inicializando CompressedSlamNode...");
    this->declare_parameter<std::string>("voc_file", "");
    this->declare_parameter<std::string>("settings_file", "");
    this->declare_parameter<bool>("do_rectify", true);
    this->declare_parameter<bool>("rescale", false);
    std::string strVocFile= this->get_parameter("voc_file").as_string();
    std::string strSettingsFile = this->get_parameter("settings_file").as_string(); 
    this->get_parameter("rescale", rescale);

    if (strVocFile.empty() || strSettingsFile.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Fill 'voc_file' and 'settings_file' parameters");
        rclcpp::shutdown();
        return;
    }

    // ORB_SLAM3::System::STEREO = 1
    m_SLAM = new ORB_SLAM3::System(strVocFile, strSettingsFile, ORB_SLAM3::System::IMU_MONOCULAR, false);
    auto imu_qos_profile = rclcpp::SensorDataQoS();
    auto img_qos_profile = rclcpp::SensorDataQoS();
    imu_qos_profile.keep_last(100);
    img_qos_profile.keep_last(10);

    subImu_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", imu_qos_profile, std::bind(&MonoInertialCompressedNode::GrabImu, this, _1));
    subImg_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("camera/compressed", img_qos_profile, std::bind(&MonoInertialCompressedNode::GrabImage, this, _1));

    syncThread_ = new std::thread(&MonoInertialCompressedNode::SyncWithImu, this);

}

MonoInertialCompressedNode::~MonoInertialCompressedNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    //SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonoInertialCompressedNode::GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "IMU received");
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void MonoInertialCompressedNode::GrabImage(const sensor_msgs::msg::CompressedImage::SharedPtr msgLeft)
{
    // RCLCPP_INFO(this->get_logger(), "Left image received");
    bufMutexImg_.lock();

    if (!imgBuf_.empty())
        imgBuf_.pop();
    imgBuf_.push(msgLeft);

    bufMutexImg_.unlock();
}

cv::Mat MonoInertialCompressedNode::GetImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat img;

    try
    {
        img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    return img;
}

void MonoInertialCompressedNode::SyncWithImu()
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