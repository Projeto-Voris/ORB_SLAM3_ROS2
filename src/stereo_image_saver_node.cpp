#include "stereo_image_saver_node.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <memory>

using ImageMsg = sensor_msgs::msg::Image;
using ApproximateSyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

class StereoImageSaverImpl : public StereoImageSaverNode
{
public:
StereoImageSaverImpl(bool use_intra)
: StereoImageSaverNode(use_intra) , Node("stereo_image_saver_node"), frame_count_(0)
    {
        this->declare_parameter<std::string>("save_directory", "/tmp/stereo_images");
        this->get_parameter("save_directory", save_directory_);

        if (!std::filesystem::exists(save_directory_))
        {
            std::filesystem::create_directories(save_directory_);
        }

        if (use_intra) {
            // Create subscriptions with intra-process enabled so same-process nodes can use zero-copy
            rclcpp::SubscriptionOptions sub_options;
            sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

        left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, "camera/left", rmw_qos_profile_sensor_data, sub_options);
        right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, "camera/right", rmw_qos_profile_sensor_data, sub_options);

        sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(
            ApproximateSyncPolicy(10), *left_sub_, *right_sub_);
        sync_->registerCallback(std::bind(&StereoImageSaverImpl::imageCallback, this, 
                                          std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "StereoImageSaverNode started. Saving to: %s", save_directory_.c_str());
        if (use_intra) {
            RCLCPP_INFO(this->get_logger(), "Subscriptions created with intra-process communication enabled.");
        }
    }
}

    private:
        void imageCallback(const ImageMsg::ConstSharedPtr &msgLeft, const ImageMsg::ConstSharedPtr &msgRight)
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

    std::string save_directory_;
    size_t frame_count_;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> sync_;
};

// Bridge constructor to behave like declared header type
StereoImageSaverNode::StereoImageSaverNode(bool /*use_intra_process*/) : rclcpp::Node("stereo_image_saver_node") {}
// Note: actual usable class is StereoImageSaverImpl which exposes the requested constructor behavior.

int main_saver_for_test(int argc, char *argv[])
{
    // This file still can be built as a separate executable, but when used in-process
    // we will instantiate StereoImageSaverImpl directly from stereo.cpp.
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoImageSaverImpl>(false);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
