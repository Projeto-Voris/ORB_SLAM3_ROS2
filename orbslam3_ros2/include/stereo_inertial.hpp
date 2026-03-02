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
    // void PublishPointCloud(std::vector<ORB_SLAM3::MapPoint*> points);
    // void Transform_orbslam2cam(const Eigen::Vector3f translation, const Eigen::Quaternionf rotation);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   subImu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImgLeft_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImgRight_;

    // Publisher for transform and PCL2
    // rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_publisher;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclpublisher;

    std::thread *syncThread_;

    // IMU
    queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Image
    queue<sensor_msgs::msg::Image::SharedPtr> imgLeftBuf_, imgRightBuf_;
    std::mutex bufMutexLeft_, bufMutexRight_;

    bool doRectify;
    bool rescale;
};
}

#endif
