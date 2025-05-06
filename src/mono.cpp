#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sys/stat.h> // Para mkdir

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"

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

// Função para salvamento dos dados
void SaveMonoData() {
    if(!pSLAM_global) return;
    
    const std::string output_dir = std::string(getenv("HOME")) + "/ros2_ws/src/orbslam3_ros2/results/mono";
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
        std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings use_pangolin" << std::endl;
        return 1;
    }
    bool visualization = true;

    auto node = std::make_shared<rclcpp::Node>("run_slam");

    // Create SLAM system
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, visualization);
    pSLAM_global = &pSLAM; // Atribui à variável global

    auto slam_node = std::make_shared<MonocularSlamNode>(&pSLAM, node.get());
    std::cout << "============================ " << std::endl;

    // Configura o handler para salvar dados no shutdown
    rclcpp::on_shutdown([&]() {
        SaveMonoData();
        pSLAM.Shutdown();
    });

    rclcpp::spin(slam_node);
    rclcpp::shutdown();

    return 0;
}



MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM, rclcpp::Node* node)
:   SlamNode(pSLAM, node)
{
    this->declare_parameter<bool>("rescale", false);
    this->get_parameter("rescale", rescale);
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{

}

void MonocularSlamNode::GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        img_cam = cv_bridge::toCvShare(msg, msg->encoding)->image;
        cv::resize(img_cam, img_cam, cv::Size(800, 600), cv::INTER_LINEAR);
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
}
