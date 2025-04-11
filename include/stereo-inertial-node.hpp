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

class StereoInertialNode : public SlamNode
{
public:
    StereoInertialNode(ORB_SLAM3::System* pSLAM, rclcpp::Node* node, const std::string &strSettingsFile, const std::string &strDoRectify , const std::string &strDoEqual);
    ~StereoInertialNode();

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

    ORB_SLAM3::System *SLAM_;
    std::thread *syncThread_;

    // IMU
    queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Image
    queue<sensor_msgs::msg::Image::SharedPtr> imgLeftBuf_, imgRightBuf_;
    std::mutex bufMutexLeft_, bufMutexRight_;

    bool doRectify_;
    bool doEqual_;
    cv::Mat M1l_, M2l_, M1r_, M2r_;

    bool bClahe_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
};

#endif
