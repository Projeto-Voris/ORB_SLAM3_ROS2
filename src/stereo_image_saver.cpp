#include "stereo_image_saver.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <memory>
#include <iomanip>
#include <sstream>

#include <unistd.h>
#define GETPID getpid

StereoImageSaverNode::StereoImageSaverNode(rclcpp::NodeOptions options)
: Node("stereo_image_saver_node", options), frame_count_(0)
    {
        this->declare_parameter<std::string>("save_directory", "/home/daniel/Documents/stereo_images");
        this->get_parameter("save_directory", save_directory_);

        if (!std::filesystem::exists(save_directory_))
        {
            std::filesystem::create_directories(save_directory_);
            std::filesystem::create_directories(save_directory_ + "/left");
            std::filesystem::create_directories(save_directory_ + "/right");
        }

        // Create subscriptions with intra-process enabled so same-process nodes can use zero-copy

        left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, "camera/left");
        right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, "camera/right");
        state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "state", rclcpp::QoS(10), [this](const std_msgs::msg::String::SharedPtr msg) {
                // RCLCPP_INFO(this->get_logger(), "Received state message: %s", msg->data.c_str());
                // slam_state = msg->data;
            });

        sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(
            ApproximateSyncPolicy(10), *left_sub_, *right_sub_);
        sync_->registerCallback(std::bind(&StereoImageSaverNode::imageCallback, this, 
                                          std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "StereoImageSaverNode started. Saving to: %s", save_directory_.c_str());

    
}

void StereoImageSaverNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msgLeft, const sensor_msgs::msg::Image::ConstSharedPtr &msgRight)
{
    // if (slam_state != "OK") {
    //     // RCLCPP_INFO(this->get_logger(), "Skipping frame %zu due to state: %s", frame_count_, slam_state.c_str());
    //     return;
    // }
    try
    {
        auto cv_left = cv_bridge::toCvShare(msgLeft, msgLeft->encoding)->image;
        auto cv_right = cv_bridge::toCvShare(msgRight, msgRight->encoding)->image;

        std::stringstream left_ss, right_ss;
        left_ss << save_directory_ << "/left/L" << std::setfill('0') << std::setw(3) << frame_count_ << ".png";
        right_ss << save_directory_ << "/right/R" << std::setfill('0') << std::setw(3) << frame_count_ << ".png";
        std::string left_file = left_ss.str();
        std::string right_file = right_ss.str();

        // cv::imwrite(left_file, cv_left->image);
        // cv::imwrite(right_file, cv_right->image);
        // Put this process's id and the msg's pointer address on the image.
        std::stringstream left, right;
        left << "pid: " << GETPID() << ", ptr: " << msgLeft.get();
        // Put this process's id and the msg's pointer address on the image.
        right << "pid: " << GETPID() << ", ptr: " << msgRight.get();
        RCLCPP_WARN(this->get_logger(), "Save l: %s, r: %s", left.str().c_str(), right.str().c_str());
        // RCLCPP_INFO(this->get_logger(), "Saved frame %zu: %s, %s", frame_count_, left_file.c_str(), right_file.c_str());
        frame_count_++;
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}


