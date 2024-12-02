#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-slam-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify" << std::endl;
        return 1;
    }
    auto node = std::make_shared<rclcpp::Node>("run_slam");


    bool visualization = true;
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, visualization);

    std::shared_ptr<StereoSlamNode> slam_ros;
    slam_ros = std::make_shared<StereoSlamNode>(&pSLAM, node.get(), argv[2], argv[3]);
    std::cout << "============================ " << std::endl;

    rclcpp::spin(slam_ros->node_->get_node_base_interface());
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
