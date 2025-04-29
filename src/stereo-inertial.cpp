#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sys/stat.h> // Para mkdir
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "stereo-inertial-node.hpp"
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

// Função para salvamento dos dados estéreo-inerciais
void SaveStereoInertialData() {
    if(!pSLAM_global) return;
    
    const std::string output_dir = std::string(getenv("HOME")) + "/ros2_ws/src/orbslam3_ros2/results/stereo_inertial";
    CreateDirectoryIfNotExists(output_dir);
    
    // Salva todos os dados implementados
    pSLAM_global->SaveMapPoints(output_dir + "/map_points.ply");
    pSLAM_global->SaveKeyFrameTrajectory(output_dir + "/keyframe_trajectory.txt");
    pSLAM_global->SaveTrajectoryKITTI(output_dir + "/trajectory_kitti.txt");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify use_pangolin" << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    bool visualization = true;

    auto node = std::make_shared<rclcpp::Node>("orb_slam");

    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, visualization);
    pSLAM_global = &pSLAM; // Atribui à variável global

    std::shared_ptr<StereoInertialNode> slam_ros;
    slam_ros = std::make_shared<StereoInertialNode>(&pSLAM, node.get(), argv[2], argv[3], argv[4]);
    std::cout << "============================" << std::endl;

    // Configura o handler para salvar dados no shutdown
    rclcpp::on_shutdown([&]() {
        SaveStereoInertialData();
        pSLAM.Shutdown();
    });

    rclcpp::spin(slam_ros->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}


StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *pSLAM, rclcpp::Node* node ,const std::string &strSettingsFile, const std::string &strDoRectify, const std::string &strDoEqual) :
    SlamNode(pSLAM, node)
{
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;
    // doRectify_ = 1;

    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    

    subImu_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 100, std::bind(&StereoInertialNode::GrabImu, this, _1));
    subImgLeft_ = this->create_subscription<sensor_msgs::msg::Image>("camera/left", 10, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    subImgRight_ = this->create_subscription<sensor_msgs::msg::Image>("camera/right", 10, std::bind(&StereoInertialNode::GrabImageRight, this, _1));
    
    // tf_publisher = this->create_publisher<geometry_msgs::msg::TransformStamped>("transform", 10);
    // pclpublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

    syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);

}

StereoInertialNode::~StereoInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoInertialNode::GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "IMU received");
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void StereoInertialNode::GrabImageLeft(const sensor_msgs::msg::Image::SharedPtr msgLeft)
{
    // RCLCPP_INFO(this->get_logger(), "Left image received");
    bufMutexLeft_.lock();

    if (!imgLeftBuf_.empty())
        imgLeftBuf_.pop();
    imgLeftBuf_.push(msgLeft);

    bufMutexLeft_.unlock();
}

void StereoInertialNode::GrabImageRight(const sensor_msgs::msg::Image::SharedPtr msgRight)
{
    // RCLCPP_INFO(this->get_logger(), "Right image received");
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
                // std::cout << "big time difference" << std::endl;
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
                // Load imu measurements from buffer
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
            current_frame_time_ = now();
            SE3 = m_SLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
            Update();
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }


    }
}
