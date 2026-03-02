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
    auto slam_node = std::make_shared<MonocularCompressedSlamNode>(options);
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



MonocularCompressedSlamNode::MonocularCompressedSlamNode(const rclcpp::NodeOptions & options)
:   SlamNode(nullptr, options)
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
    m_SLAM = new ORB_SLAM3::System(strVocFile, strSettingsFile, ORB_SLAM3::System::MONOCULAR, false);

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
    rescale = this->get_parameter("rescale").as_bool();
    cv::Mat img;
    try
    {
        img_cam = cv_bridge::toCvCopy(msg, "bgr8")->image;
        if(rescale){

            cv::resize(img_cam, img, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
        } else {
            img = img_cam;
        }
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