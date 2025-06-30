#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace clock_pose {

/** 
 * Human readable form for the x and y positions, and the yaw.
 *
 */
struct SimplePose {
    double x_pos = 0.;
    double y_pos = 0.;
    double yaw = 0.;
};

/**
 * Utility functions to calculate the pose from time.
 *
 */

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
geometry_msgs::msg::PoseStamped CalculatePoseFromCurrentTime(const int radius = 1.);

/**
 * Generates the pose represented by the minute hand on a clock.
 * The pose will contain an x and y position, and a heading (yaw) that takes the robot
 * clockwise to follow the minute hand.
 *
 * The pose will be provided in a fixed inertial frame.
 *
 * NOTE(akhil): This function is broken out to provide unit testing coverage.
 *
 * @param num_minutes The number of minutes since the hour began.
 * @param radius The radius of the clock face. Units [m]
 *
 * @return A simple pose, containing x and y positions, and yaw.
 */
SimplePose CalculatePoseFromMinutes(const int num_minutes, const double raidus = 1.);

} // namespace clock_pose
