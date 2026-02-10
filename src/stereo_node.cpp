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

    auto slam_node = std::make_shared<StereoSlamNode>(&pSLAM, node.get(), argv[2], argv[3]);
    auto saver_node = std::make_shared<StereoImageSaverNode>();
    std::cout << "============================ " << std::endl;

    rclcpp::on_shutdown([&]() {
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

