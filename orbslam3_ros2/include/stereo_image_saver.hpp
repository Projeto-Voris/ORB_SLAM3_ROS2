#ifndef STEREO_IMG_SAVE_HPP
#define STEREO_IMG_SAVE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <vector>
#include <string>
#include <fstream>
#include <std_srvs/srv/trigger.hpp>

namespace slam
{
    class ImageSaver : public rclcpp::Node {
        public:
            ImageSaver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

            ~ImageSaver();

        private:
            bool verify_path(const std::string& path_string, int counter);

            void images_cb(const sensor_msgs::msg::Image::ConstSharedPtr &msgLeft,
                        const sensor_msgs::msg::Image::ConstSharedPtr &msgRight);

            void odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);

            bool save_image_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> &req,
                                const std::shared_ptr<std_srvs::srv::Trigger::Response> &res);

            cv::Mat appyCLAHEtoColor(const cv::Mat& input_bgr);


            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
            std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub;
            std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub;

            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_image_srv;

            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;

            std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;


            sensor_msgs::msg::Image::ConstSharedPtr msgLeft_, msgRight_;
            nav_msgs::msg::Odometry::ConstSharedPtr lastOdom_;

            cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE();
            bool apply_clahe_{true};
            std::string path_;
            std::ofstream odom_file_;
            int counter_;
            bool save_images;

    };
}
#endif // STEREO_IMG_SAVE_HPP
