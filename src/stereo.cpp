#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sys/stat.h> // Para mkdir
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "stereo-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include "System.h"

using std::placeholders::_1;
using std::placeholders::_2;

// Variável global para acesso ao SLAM
ORB_SLAM3::System* pSLAM_global = nullptr;

// Função para criar diretório se não existir
void CreateDirectoryIfNotExists(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
        mkdir(path.c_str(), 0777);
    }
}

// Função para salvamento dos dados estéreo
void SaveStereoData() {
    if(!pSLAM_global) return;
    
    const std::string output_dir = std::string(getenv("HOME")) + "/ros2_ws/src/orbslam3_ros2/results/stereo";
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
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify" << std::endl;
        return 1;
    }
    bool visualization = true;

    auto node = std::make_shared<rclcpp::Node>("run_slam");

    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, visualization);
    pSLAM_global = &pSLAM; // Atribui à variável global

    std::shared_ptr<StereoSlamNode> slam_ros;
    slam_ros = std::make_shared<StereoSlamNode>(&pSLAM, node.get(), argv[2], argv[3]);
    std::cout << "============================ " << std::endl;

    // Configura o handler para salvar dados no shutdown
    rclcpp::on_shutdown([&]() {
        SaveStereoData();
        pSLAM.Shutdown();
    });

    rclcpp::spin(slam_ros->node_->get_node_base_interface());
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}






StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, rclcpp::Node* node, const std::string &strSettingsFile, const std::string &strDoRectify)
: SlamNode(pSLAM, node)
{
    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;
    this->declare_parameter<bool>("rescale", false);
    this->get_parameter("rescale", rescale);


    // Cria os subscritores usando o nó passado
    left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node, "camera/left");
    right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node, "camera/right");

    // Sincroniza os subscritores
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
}

StereoSlamNode::~StereoSlamNode() 
{
}

void StereoSlamNode::GrabStereo(const sensor_msgs::msg::Image::SharedPtr msgLeft, const sensor_msgs::msg::Image::SharedPtr msgRight) {
    // Copia a imagem RGB da mensagem ROS para cv::Mat
    try {
        imLeft = cv_bridge::toCvShare(msgLeft, msgLeft->encoding)->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copia a imagem de profundidade da mensagem ROS para cv::Mat
    try {
        imRight = cv_bridge::toCvShare(msgRight, msgLeft->encoding)->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    current_frame_time_ = now();
    if (rescale){

        cv::resize(imLeft, imLeft, cv::Size(800,600), cv::INTER_LINEAR);
        cv::resize(imRight, imRight, cv::Size(800,600), cv::INTER_LINEAR);
    }


    SE3 = m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));
    Update();
}


