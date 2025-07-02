#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include  "target_pose_issuer/target_pose_issuer.h"

namespace target_pose {

namespace {

    const std::string kNodeName = "target_pose_issuer";
    const std::string kTopicName = "target_pose";
    const std::string kClockPoseTopicName = "/clock_pose";
    const std::string kGuiPoseTopicName = "/gui_pose";

} // namespace

TargetPoseIssuer::TargetPoseIssuer() : Node(kNodeName) {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(kTopicName, 10);

    clock_pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            kClockPoseTopicName, 10,
            std::bind(
                &TargetPoseIssuer::HandleClockPoseMessage,
                this, std::placeholders::_1
                )
            );
    
    gui_pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            kGuiPoseTopicName, 10,
            std::bind(
                &TargetPoseIssuer::HandleGuiPoseMessage,
                this, std::placeholders::_1
                )
            );
}

/**
 * Callback handler for clock pose messages.
 *
 * @param msg A clock pose message to be handled.
 */
void TargetPoseIssuer::HandleClockPoseMessage(const geometry_msgs::msg::PoseStamped& msg) {
    last_clock_pose_msg_ptr_ = std::make_unique<geometry_msgs::msg::PoseStamped>(msg);

    if (last_gui_pose_msg_ptr_ == nullptr) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing clock based pose as target.");
        publisher_->publish(msg);
    }
}

/**
 * Callback handler for clock pose messages.
 *
 * @param msg A clock pose message to be handled.
 */
void TargetPoseIssuer::HandleGuiPoseMessage(const geometry_msgs::msg::PoseStamped& msg) {
    // If the publisehd gui pose has a blank timestamp, it means the target pose
    // is no longer valid, and publish the last clock pose.
    if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) {
        last_gui_pose_msg_ptr_ = nullptr;
        // Prevent race condition at start up.
        if (last_clock_pose_msg_ptr_ != nullptr) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Publishing clock based pose as target, since gui based expired.");
            publisher_->publish(*last_clock_pose_msg_ptr_);
        }

        // Early return, to not execute the rest of the code;
        return;
    }

    last_gui_pose_msg_ptr_ = std::make_unique<geometry_msgs::msg::PoseStamped>(msg);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing gui based pose as target.");
    publisher_->publish(msg);
}

} // namespace target_pose

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<target_pose::TargetPoseIssuer>());
    rclcpp::shutdown();

    return 0;
}

