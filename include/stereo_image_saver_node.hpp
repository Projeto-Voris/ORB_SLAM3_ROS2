#ifndef __STEREO_IMAGE_SAVER_NODE_HPP__
#define __STEREO_IMAGE_SAVER_NODE_HPP__

#include <rclcpp/rclcpp.hpp>

class StereoImageSaverNode : public rclcpp::Node
{
public:
    // If use_intra_process is true, subscriptions will be created with
    // SubscriptionOptions.use_intra_process_comm = Enable so in-process zero-copy is possible.
    explicit StereoImageSaverNode(bool use_intra_process = false);

    ~StereoImageSaverNode() = default;
};

#endif
