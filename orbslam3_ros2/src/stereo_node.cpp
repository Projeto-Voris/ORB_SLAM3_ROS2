#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sys/stat.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "stereo.hpp"
#include "stereo_image_saver.hpp"
#include <opencv2/core/core.hpp>
#include "System.h"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    bool run_saver_in_process = false;
    for (int i = 4; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--with-saver" || a == "--inproc-saver") {
            run_saver_in_process = true;
        }
    }
   
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(run_saver_in_process); // Enable intra-process communication if saver is in the same process

    auto slam_node = std::make_shared<orbslam3_ros2::StereoSlamNode>(options);
    std::cout << "============================ " << std::endl;
    
    
    if (run_saver_in_process) {
        auto saver_node = std::make_shared<orbslam3_ros2::StereoImageSaverNode>(options);
        // instantiate saver node with intra-process subscriptions enabled
        // StereoImageSaverImpl is the concrete implementation (defined in the cpp)
        // we create it with use_intra_process = true so its subscriptions use intra-process option
        // auto saver_node = std::make_shared<StereoImageSaverImpl>(true); // zero-copy inside process

        // Run both in a multi-threaded executor
        rclcpp::executors::MultiThreadedExecutor exec;
        exec.add_node(slam_node);
        exec.add_node(saver_node);

        RCLCPP_INFO(slam_node->get_logger(), "Running SLAM and Saver in the same process (MultiThreadedExecutor) for zero-copy intra-process transport.");
        exec.spin();

    } else {
        // Run SLAM node alone in default single-threaded spin
        rclcpp::spin(slam_node);
    }

    rclcpp::shutdown();
    return 0;
}

