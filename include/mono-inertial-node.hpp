#ifndef __STEREO_INERTIAL_NODE_HPP__
#define __STEREO_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.h>

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

// using ImageMsg = sensor_msgs::msg::Image;

class MonoInertialNode : public SlamNode
{
public:
    MonoInertialNode(ORB_SLAM3::System* pSLAM, rclcpp::Node* node);
    ~MonoInertialNode();

private:
    // using ImuMsg = sensor_msgs::msg::Imu;
    void GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg);
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void SyncWithImu();

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   subImu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImg_;

    ORB_SLAM3::System *SLAM_;
    std::thread *syncThread_;

    // IMU
    queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Image
    queue<sensor_msgs::msg::Image::SharedPtr> imgBuf_;
    std::mutex bufMutexImg_;
};

#endif
