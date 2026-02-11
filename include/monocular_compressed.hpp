#ifndef __MONOCULAR_COMPRESSED_SLAM_NODE_HPP__
#define __MONOCULAR_COMPRESSED_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include <cv_bridge/cv_bridge.hpp>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

#include "slam_node.hpp"

class MonocularCompressedSlamNode : public SlamNode
{
public:
    MonocularCompressedSlamNode(ORB_SLAM3::System* pSLAM, rclcpp::Node* node, rclcpp::NodeOptions options);

    ~MonocularCompressedSlamNode();

private:
    using CompressedImageMsg = sensor_msgs::msg::CompressedImage;

    void GrabImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    cv::Mat img_cam;

    bool rescale;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr m_image_subscriber;
};

#endif