#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sys/stat.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "stereo-slam-node.hpp"
#include "stereo_image_saver_node.hpp" // header we added

#include <opencv2/core/core.hpp>
#include "System.h"

using std::placeholders::_1;
using std::placeholders::_2;

// Global variable for SLAM access
ORB_SLAM3::System* pSLAM_global = nullptr;

// Function to create directory if it does not exist
void CreateDirectoryIfNotExists(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
        mkdir(path.c_str(), 0777);
    }
}

// Function for stereo data saving
void SaveStereoData() {
    if(!pSLAM_global) return;
    
    const std::string output_dir = std::string(getenv("HOME")) + "/ros2_ws/src/orbslam3_ros2/results/stereo";
    CreateDirectoryIfNotExists(output_dir);
    
    pSLAM_global->SaveMapPoints(output_dir + "/map_points.ply");
    pSLAM_global->SaveKeyFrameTrajectory(output_dir + "/keyframe_trajectory.txt");
    pSLAM_global->SaveTrajectoryKITTI(output_dir + "/trajectory_kitti.txt");
}

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

    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, visualization);
    pSLAM_global = &pSLAM;

    auto slam_node = std::make_shared<StereoSlamNode>(&pSLAM, node.get(), argv[2], argv[3]);
    std::cout << "============================ " << std::endl;

    rclcpp::on_shutdown([&]() {
        SaveStereoData();
        pSLAM.Shutdown();
    });

    if (run_saver_in_process) {
        // instantiate saver node with intra-process subscriptions enabled
        // StereoImageSaverImpl is the concrete implementation (defined in the cpp)
        // we create it with use_intra_process = true so its subscriptions use intra-process option
        // auto saver_node = std::make_shared<StereoImageSaverImpl>(true); // zero-copy inside process

        // Run both in a multi-threaded executor
        rclcpp::executors::MultiThreadedExecutor exec;
        exec.add_node(slam_node);
        // exec.add_node(saver_node);

        RCLCPP_INFO(slam_node->get_logger(), "Running SLAM and Saver in the same process (MultiThreadedExecutor) for zero-copy intra-process transport.");
        exec.spin();
    } else {
        // Run SLAM node alone in default single-threaded spin
        rclcpp::spin(slam_node);
    }

    rclcpp::shutdown();
    return 0;
}


StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, rclcpp::Node* node, const std::string &strSettingsFile, const std::string &strDoRectify)
: SlamNode(pSLAM, node)
{
    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;
    this->declare_parameter<bool>("rescale", false);
    this->get_parameter("rescale", rescale);

    left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "camera/left");
    right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "camera/right");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
}

StereoSlamNode::~StereoSlamNode() 
{
}

void StereoSlamNode::GrabStereo(const sensor_msgs::msg::Image::SharedPtr msgLeft, const sensor_msgs::msg::Image::SharedPtr msgRight) {
    try {
        imLeft = cv_bridge::toCvShare(msgLeft, msgLeft->encoding)->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    try {
        imRight = cv_bridge::toCvShare(msgRight, msgLeft->encoding)->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    if (rescale){
        cv::resize(imLeft, imLeft, cv::Size(800,600), cv::INTER_LINEAR);
        cv::resize(imRight, imRight, cv::Size(800,600), cv::INTER_LINEAR);
    }
    
    SE3 = m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));
    current_frame_time_ = now();
    Update();
    TrackedImage(imLeft);
}


