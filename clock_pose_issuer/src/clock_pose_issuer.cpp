#include <chrono>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "clock_pose_issuer/clock_pose_utilities.h"

#include "clock_pose_issuer/clock_pose_issuer.h"


// TODO(akhil): Try to avoid using namespaces.
using namespace std::chrono_literals;

namespace clock_pose {

namespace {

    const std::string kNodeName = "clock_pose_issuer";
    const std::string kTopicName = "clock_pose_issuer_topic";
    constexpr std::chrono::milliseconds kPublishFrequency = std::chrono::milliseconds(500ms); // Units [ms]

} // namespace

ClockPoseIssuer::ClockPoseIssuer() : Node(kNodeName) {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(kTopicName, 10);
    timer_ = this->create_wall_timer(kPublishFrequency, std::bind(&ClockPoseIssuer::Callback, this));
}

void ClockPoseIssuer::Callback() {
    geometry_msgs::msg::PoseStamped pose_stamped = CalculatePoseFromCurrentTime();
    pose_stamped.header.stamp = this->get_clock()->now();
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing clock based pose.");

    publisher_->publish(pose_stamped);
}

} // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<clock_pose::ClockPoseIssuer>());
    rclcpp::shutdown();

    return 0;
}
