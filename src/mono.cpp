#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"

#include "System.h"

using std::placeholders::_1;



int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("run_slam");

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    bool visualization = true;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, visualization);

    auto slam_node = std::make_shared<MonocularSlamNode>(&SLAM, node.get());
    std::cout << "============================ " << std::endl;

    rclcpp::spin(slam_node);
    rclcpp::shutdown();

    return 0;
}

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM, rclcpp::Node* node)
:   SlamNode(pSLAM, node)
{
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{

}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        img_cam = cv_bridge::toCvShare(msg, msg->encoding)->image;
        cv::resize(img_cam, img_cam, cv::Size(1600, 1200), cv::INTER_LINEAR);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout<<"one frame has been sent"<<std::endl;
    SE3 = m_SLAM->TrackMonocular(img_cam, Utility::StampToSec(msg->header.stamp));
    // Update();
}
