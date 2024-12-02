#ifndef __STEREO_SLAM_NODE_HPP__
#define __STEREO_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>
#include <string> 

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"
#include "utility.hpp"

class StereoSlamNode : public rclcpp::Node
{
public:
    StereoSlamNode(ORB_SLAM3::System* pSLAM, rclcpp::Node* node, const std::string &strSettingsFile, const std::string &strDoRectify);

    ~StereoSlamNode();

    rclcpp::Node* node_;

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using approximate_sync_policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    void GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight);

    ORB_SLAM3::System* m_SLAM;

    bool doRectify;
    cv::Mat M1l, M2l, M1r, M2r;

    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;

    std::shared_ptr<message_filters::Subscriber<ImageMsg>> left_sub;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> right_sub;

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;
};

#endif

