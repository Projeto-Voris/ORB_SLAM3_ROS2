#ifndef __SLAM_NODE_HPP__
#define __SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"

#include "nav_msgs/msg/path.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"


#include <cv_bridge/cv_bridge.h>
#include <string> 
#include <opencv2/opencv.hpp>
#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"
#include "utility.hpp"

class SlamNode : public rclcpp::Node
{
public:
    SlamNode(ORB_SLAM3::System* pSLAM, rclcpp::Node* node);

    ~SlamNode();
    void Update();
    void PublishCurrentPointCloud();
    void PublishTrackedPointCloud();
    void PublishPath();
    void PublishPose();
    void PublishTransform();
    tf2::Transform TransformFromSophus(Sophus::SE3f &pose);

    rclcpp::Node* node_;

protected:
    // using ImageMsg = sensor_msgs::msg::Image;

    ORB_SLAM3::System* m_SLAM;
    std::vector<ORB_SLAM3::KeyFrame*> trajectory;
    std::vector<ORB_SLAM3::MapPoint*> map_points;
    Sophus::SE3f SE3;
    rclcpp::Time current_frame_time_;
    // rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_publisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    tf2::Transform initial_map_base_offset_;
    bool initial_offset_set_ = false;

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posepublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclpublisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathpublisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statepublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flagpublisher;

};

#endif