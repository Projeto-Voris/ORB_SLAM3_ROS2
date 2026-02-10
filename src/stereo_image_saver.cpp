#include "stereo_image_saver.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <memory>

StereoImageSaverNode::StereoImageSaverNode()
: Node("stereo_image_saver_node", rclcpp::NodeOptions().use_intra_process_comms(true)), frame_count_(0)
    {
        this->declare_parameter<std::string>("save_directory", "/home/jetson/Documents/stereo_images");
        this->get_parameter("save_directory", save_directory_);

        if (!std::filesystem::exists(save_directory_))
        {
            std::filesystem::create_directories(save_directory_);
        }

        // Create subscriptions with intra-process enabled so same-process nodes can use zero-copy
        rclcpp::SubscriptionOptions sub_options;
        sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

        left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, "camera/left");
        right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, "camera/right");

        sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(
            ApproximateSyncPolicy(10), *left_sub_, *right_sub_);
        sync_->registerCallback(std::bind(&StereoImageSaverNode::imageCallback, this, 
                                          std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "StereoImageSaverNode started. Saving to: %s", save_directory_.c_str());

    
}

void StereoImageSaverNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msgLeft, const sensor_msgs::msg::Image::ConstSharedPtr &msgRight)
{
    try
    {
        auto cv_left = cv_bridge::toCvCopy(msgLeft, msgLeft->encoding);
        auto cv_right = cv_bridge::toCvCopy(msgRight, msgRight->encoding);

        std::string left_file = save_directory_ + "/left_" + std::to_string(frame_count_) + ".png";
        std::string right_file = save_directory_ + "/right_" + std::to_string(frame_count_) + ".png";

        cv::imwrite(left_file, cv_left->image);
        cv::imwrite(right_file, cv_right->image);

        RCLCPP_INFO(this->get_logger(), "Saved frame %zu: %s, %s", frame_count_, left_file.c_str(), right_file.c_str());
        frame_count_++;
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}


