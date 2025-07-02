#include <chrono>
#include <cmath>
#include <ctime>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "clock_pose_issuer/clock_pose_utilities.h"

namespace clock_pose {

namespace {

    constexpr double kHourInMinutes = 1. / 60.; // Units: hour / minutes

} // namespace

/**
 * From the current time, determines the pose based on the minute hand.
 * The pose will contain an x and y position, and a heading (yaw) that takes the robot
 * clockwise to follow the minute hand.
 *
 * The pose will be provided in a fixed inertial frame.
 *
 * @param radius The radius of the clock face. Units [m]
 *
 * @return A stamped pose containing the time that the pose was created.
 */
geometry_msgs::msg::PoseStamped CalculatePoseFromCurrentTime(const int radius) {
    // Convert current time to extract minutes.
    const auto now = std::chrono::system_clock::now();
    std::time_t t_c = std::chrono::system_clock::to_time_t(now);
    const std::tm tm_c = *std::localtime(&t_c);

    // Caluulate the fundamental pose parameters.
    int num_minutes = tm_c.tm_min;
    const SimplePose simple_pose = CalculatePoseFromMinutes(num_minutes, radius);

    geometry_msgs::msg::PoseStamped stamped_pose;
    stamped_pose.header.frame_id = "inertial";
    stamped_pose.pose.position.x = simple_pose.x_pos;
    stamped_pose.pose.position.y = simple_pose.y_pos;
    stamped_pose.pose.position.z = 0.;

    // TODO(akhil): Either figure out how to use thet tf2 library or write a
    // custom one for this.
    const double yaw = simple_pose.yaw;
    
    double cy = std::cos(0.5 * yaw);
    double sy = std::sin(0.5 * yaw);

    stamped_pose.pose.orientation.x = 0.;
    stamped_pose.pose.orientation.y = 0.;
    stamped_pose.pose.orientation.z = sy;
    stamped_pose.pose.orientation.w = cy;
    
    return stamped_pose;
}

/**
 * Generates the pose represented by the minute hand on a clock.
 * The pose will contain an x and y position, and a heading (yaw) that takes the robot
 * clockwise to follow the minute hand.
 *
 * The pose will be provided in a fixed inertial frame.
 *
 * @param num_minutes The number of minutes since the hour began.
 * @param radius The radius of the clock face. Units [m]
 *
 * @return A simple pose, containing x and y positions, and yaw.
 */
SimplePose CalculatePoseFromMinutes(const int num_minutes, const double radius) {
    // Convert minutes into radiuns from the zero minute mark as the reference.
    const double radians_from_zero_minutes = M_PI / 2 - static_cast<double>(num_minutes)
        * kHourInMinutes * 2 * M_PI;

    // Determines the x and y position of the pose.
    const double x_pos = radius * std::cos(radians_from_zero_minutes);
    const double y_pos = radius * std::sin(radians_from_zero_minutes);

    // Find the heading of the normal vector.
    const double yaw = std::atan2(-x_pos, y_pos);

    SimplePose pose;
    pose.x_pos = x_pos;
    pose.y_pos = y_pos;
    pose.yaw = yaw;

    return pose;
}

} // namespace clock_pose
