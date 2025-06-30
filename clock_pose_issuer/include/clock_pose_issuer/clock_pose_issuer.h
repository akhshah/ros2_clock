#pragma once

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace clock_pose {

/**
 * Publishes poses for the robot to follow based on the position of the minute
 * hand.
 *
 */
class ClockPoseIssuer : public rclcpp::Node {
public:
    ClockPoseIssuer();

private:
    void Callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

} // namespace
