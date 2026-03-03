#ifndef __SLAM_HPP__
#define __SLAM_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/image.hpp"

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

#include <orbslam3_msgs/msg/slam_status.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <string> 
#include <opencv2/opencv.hpp>
#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"
#include "utility.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sys/stat.h>
#include <string>

#include "System.h"

namespace orbslam3_ros2
{
class SlamNode : public rclcpp::Node
{
public:
    SlamNode(ORB_SLAM3::System* pSLAM, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~SlamNode();
    void Update();
    void PublishCurrentPointCloud();
    void PublishTrackedPointCloud();
    void PublishPath();
    void PublishPose();
    void PublishTransform();
    void TrackedImage(const cv::Mat image);
    void handleReset( const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response); 
    void CreateDirectoryIfNotExists(const std::string& directory);
    void SaveData();
    tf2::Transform TransformFromSophus(Sophus::SE3f &pose);

    static const tf2::Matrix3x3 tf_orb_to_ros_enu;
    static const tf2::Matrix3x3 tf_orb_to_ros_default;

protected:
    ORB_SLAM3::System* m_SLAM;
    std::vector<ORB_SLAM3::KeyFrame*> trajectory;
    std::vector<ORB_SLAM3::MapPoint*> map_points;
    Sophus::SE3f SE3;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    bool initial_offset_set_ = false;
    bool tf_static_cached_{false};
    rclcpp::Time current_frame_time_;
    tf2::Transform initial_map_base_offset_; 
    tf2::Transform T_base_cam_;

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posepublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclpublisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathpublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr trackedpublisher;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetservice;
    
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statepublisher;
    // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flagpublisher;
    
    rclcpp::Publisher<orbslam3_msgs::msg::SlamStatus>::SharedPtr status_publisher;


};
}
#endif