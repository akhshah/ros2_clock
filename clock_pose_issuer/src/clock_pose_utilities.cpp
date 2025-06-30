#include <chrono>
#include <cmath>
#include <ctime>

#include "clock_pose_issuer/clock_pose_utilities.h"

namespace clock_pose {

namespace {

    constexpr double kHourInMinutes = 1. / 60.; // Units: hour / minutes

} // namespace

geometry_msgs::PoseStamped CalculatePoseFromCurrentTime(const int radius) {
    // Convert current time to extract minutes.
    const auto now = std::chrono::system_clock::now();
    std::time_t t_c = std::chrono::system_clock_to_time_t(now);
    const std::tm tm_c = *std::localtime(&t_c);

    // Caluulate the fundamental pose parameters.
    int num_minutes = tm_c.tm_min;
    const SimplePose simple_pose = CalculatePoseFromMinutes(num_minutes, radius);

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "inertial";
    pose.point.x = simple_pos.x_pos;
    pose.point.y = simple_pose.y_pos;
    pose.point.z = 0.;

    // TODO(akhil): Either figure out how to use thet tf2 library or write a
    // custom one for this.
    const double yaw = simple_pose.yaw;
    
    double cy = std::cos(0.5 * yaw);
    double sy = std::sin(0.5 * yaw);

    pose.orientation.x = 0.;
    pose.orientation.y = 0.;
    pose.orientation.z = sy;
    pose.orientation.w = cy;
    
    return pose;
}

SimplePose CalculatePoseFromMinutes(const int num_minutes, const double radius) {
    // Convert minutes into radiuns from the zero minute mark as the reference.
    const double radians_from_zero_minutes = M_PI / 4 - static_cast<double>(num_minutes)
        * kHourInMinutes * 2 * M_PI;

    // Determines the x and y position of the pose.
    const double x_pos = radius * std::sin(radians_from_zero_minutes);
    const double y_pos = radius * std::cos(radians_from_zero_minutes);

    // Find the heading of the normal vector.
    const double yaw = std::atan2(-y_pos, x_pos);

    SimplePose pose;
    pose.x_pos = x_pos;
    pose.y_pos = y_pos;
    pose.yaw = yaw;

    return pose;
}

} // namespace clock_pose
