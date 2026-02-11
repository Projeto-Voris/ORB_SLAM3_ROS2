#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sys/stat.h> // Para mkdir

#include "rclcpp/rclcpp.hpp"
#include "monocular.hpp"

#include "System.h"

using std::placeholders::_1;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify [--with-saver]\n"
                  << "  add --with-saver to run an image-saver node in the same process (intra-process zero-copy)\n";
        return 1;
    }
    bool visualization = false;

    bool run_saver_in_process = false;
    for (int i = 4; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--with-saver" || a == "--inproc-saver") {
            run_saver_in_process = true;
        }
    }

    auto node = std::make_shared<rclcpp::Node>("run_slam");
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(run_saver_in_process); // Enable intra-process communication if saver is in the same process
    // Create SLAM system
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, visualization);

    auto slam_node = std::make_shared<MonocularSlamNode>(&pSLAM, node.get(), options);
    std::cout << "============================ " << std::endl;

    // Configura o handler para salvar dados no shutdown
    rclcpp::on_shutdown([&]() {
        pSLAM.Shutdown();
    });

    rclcpp::spin(slam_node);
    rclcpp::shutdown();

    return 0;
}



MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM, rclcpp::Node* node, rclcpp::NodeOptions options)
:   SlamNode(pSLAM, node, options)
{
    this->declare_parameter<bool>("rescale", false);
    this->get_parameter("rescale", rescale);
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
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
        // cv::resize(img_cam, img_cam, cv::Size(960, 540), cv::INTER_LINEAR);
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
    TrackedImage(img_cam);
}
