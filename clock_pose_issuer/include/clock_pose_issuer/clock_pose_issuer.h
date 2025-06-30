#pragma once

#include "rclcpp/rclcpp.h"
#include "geometry_msg/PoseStamped.h"

#include "clock_pose_issuer/clock_pose_calculator.h"

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

    ClockPoseCalculator calculator_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msg::PoseStamped>::SharedPtr publisher_;
};

} // namespace
