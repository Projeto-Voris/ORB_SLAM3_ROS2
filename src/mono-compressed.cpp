#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sys/stat.h> // Para mkdir

#include "rclcpp/rclcpp.hpp"
#include "monocular_compressed.hpp"

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

    auto node = std::make_shared<rclcpp::Node>("run_slam_compressed");
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(run_saver_in_process); // Enable intra-process communication if saver is in the same process
    // Create SLAM system
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, visualization);
    auto slam_node = std::make_shared<MonocularCompressedSlamNode>(&pSLAM, node.get(), options);
    std::cout << "============================ " << std::endl;
    
    if(run_saver_in_process) {
        // Run both in a multi-threaded executor
        rclcpp::executors::MultiThreadedExecutor exec;
        exec.add_node(slam_node);

        RCLCPP_INFO(slam_node->get_logger(), "Running SLAM and Saver in the same process (MultiThreadedExecutor) for zero-copy intra-process transport.");
        exec.spin();
    } else {
        rclcpp::spin(slam_node);
    }


    rclcpp::shutdown();

    return 0;
}



MonocularCompressedSlamNode::MonocularCompressedSlamNode(ORB_SLAM3::System* pSLAM, rclcpp::Node* node, rclcpp::NodeOptions options)
:   SlamNode(pSLAM, node, options)
{
    this->declare_parameter<bool>("rescale", false);
    this->get_parameter("rescale", rescale);
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "camera/compressed",
        10,
        std::bind(&MonocularCompressedSlamNode::GrabImage, this, std::placeholders::_1));
    }

MonocularCompressedSlamNode::~MonocularCompressedSlamNode()
{

}

void MonocularCompressedSlamNode::GrabImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // Copy the ros compressed image message to cv::Mat.
    try
    {
        img_cam = cv_bridge::toCvCopy(msg, "bgr8")->image;
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