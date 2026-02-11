#ifndef __STEREO_IMAGE_SAVER_NODE_HPP__
#define __STEREO_IMAGE_SAVER_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "std_msgs/msg/string.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <string> 
#include <opencv2/opencv.hpp>

class StereoImageSaverNode : public rclcpp::Node
{
    public:
        // Simple node wrapper used as a base for the implementation in the .cpp.
        // Provide an inline default constructor so translation units that include
        // this header don't require a separate definition.
        StereoImageSaverNode(rclcpp::NodeOptions options);
        ~StereoImageSaverNode() = default;
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msgLeft, 
                            const sensor_msgs::msg::Image::ConstSharedPtr &msgRight);
        
    protected:
        std::string save_directory_;
        size_t frame_count_;
        std::string slam_state;
        using ApproximateSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
        std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> sync_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;

};

#endif
