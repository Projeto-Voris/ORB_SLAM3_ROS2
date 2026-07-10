#ifndef __STEREO_INERTIAL_NODE_HPP__
#define __STEREO_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"
#include "slam_node.hpp"
#include <queue>

namespace orbslam3_ros2
{
class StereoInertialNode : public SlamNode
{
public:
    StereoInertialNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~StereoInertialNode() override;

private:
    void GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg);
    void GrabImageLeft(const sensor_msgs::msg::Image::SharedPtr msgLeft);
    void GrabImageRight(const sensor_msgs::msg::Image::SharedPtr msgRight);
    cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void SyncWithImu();

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   subImu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImgLeft_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImgRight_;


    std::thread *syncThread_;

    // IMU
    queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Image
    queue<sensor_msgs::msg::Image::SharedPtr> imgLeftBuf_, imgRightBuf_;
    std::mutex bufMutexLeft_, bufMutexRight_;

    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE();
    bool apply_clahe {false};

};
}

#endif
