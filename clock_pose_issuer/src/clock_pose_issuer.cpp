#include <chrono>

#include "clock_pose_issuer/clock_pose_issuer.h"
#include "geometry_msg/PoseStamped.h"


// TODO(akhil): Try to avoid using namespaces.
using namespace std::chrono_literals;

namespace clock_pose {

namespace {

    constexpr int kQueueSize = 10;
    constexpr std::string kNodeName = "clock_pose_issuer";
    constexpr std::string kTopicName = "clock_pose_issuer_topic";
    constexpr std::chrono::milliseconds kPublishFrequency = std::chrono::milliseconds(500ms); // Units [ms]
                                                                                              //
} // namespace

ClockPoseIssuer::ClockPoseIssuer() : Node(kNodeName) {
    publisher_ = this->create_publisher<geometry_msgs::PoseStamped>(kTopicName, kQueueSize);
    timer_ = this->create_wall_timer(kPublishFrequency, std::bind(&ClockPoseIssuer::Callback, this));
}

void Callback() {
    geometry_msg::PoseStamped pose_stamped = CalculatePoseFromCurrentTime();
    pose_stamped.header.stamp = this->getclock()->now();
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing clock based pose");

    publisher_->publish(pose_stamped);
}

} // namespace
