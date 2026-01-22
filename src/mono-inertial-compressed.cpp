#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sys/stat.h> // Para mkdir

#include "rclcpp/rclcpp.hpp"
#include "mono-inertial-compressed-node.hpp"

#include "System.h"

using std::placeholders::_1;

// Variável global para acesso ao SLAM
ORB_SLAM3::System* pSLAM_global = nullptr;

// Função para criar diretório se não existir
void CreateDirectoryIfNotExists(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
        mkdir(path.c_str(), 0777);
    }
}

// Função para salvamento dos dados inerciais
void SaveMonoInertialCompressedData() {
    if(!pSLAM_global) return;
    
    const std::string output_dir = std::string(getenv("HOME")) + "/ros2_ws/src/orbslam3_ros2/results/mono_inertial_compressed";
    CreateDirectoryIfNotExists(output_dir);
    
    // Salva todos os dados implementados
    pSLAM_global->SaveMapPoints(output_dir + "/map_points.ply");
    pSLAM_global->SaveKeyFrameTrajectory(output_dir + "/keyframe_trajectory.txt");
    pSLAM_global->SaveTrajectoryKITTI(output_dir + "/trajectory_kitti.txt");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono_inertial_compressed path_to_vocabulary path_to_settings visualization" << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    bool visualization = false;
    
    auto node = std::make_shared<rclcpp::Node>("orb_slam");

    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, visualization);
    pSLAM_global = &pSLAM; // Atribui à variável global

    auto slam_node = std::make_shared<MonoInertialCompressedNode>(&pSLAM, node.get());
    std::cout << "============================" << std::endl;

    // Configura o handler para salvar dados no shutdown
    rclcpp::on_shutdown([&]() {
        SaveMonoInertialCompressedData();
        pSLAM.Shutdown();
    });

    rclcpp::spin(slam_node);
    rclcpp::shutdown();

    return 0;
}

MonoInertialCompressedNode::MonoInertialCompressedNode(ORB_SLAM3::System *pSLAM, rclcpp::Node* node) :
    SlamNode(pSLAM, node)
{
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