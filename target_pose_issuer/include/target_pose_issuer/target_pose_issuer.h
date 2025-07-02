#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace target_pose {

class TargetPoseIssuer : public rclcpp::Node {
public:
    TargetPoseIssuer();

private:
    void HandleClockPoseMessage(const geometry_msgs::msg::PoseStamped& msg);
    void HandleGuiPoseMessage(const geometry_msgs::msg::PoseStamped& msg);

    std::unique_ptr<geometry_msgs::msg::PoseStamped> last_clock_pose_msg_ptr_ = nullptr;
    std::unique_ptr<geometry_msgs::msg::PoseStamped> last_gui_pose_msg_ptr_ = nullptr;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr clock_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gui_pose_subscriber_;
};

} // namespace
