#include <cmath>
#include <gtest/gtest.h>

#include "clock_pose_issuer/clock_pose_utilities.h"

TEST(ClockPoseIssuer, CaculatePoseFromMinutesTest) {
    const int num_minutes = 15;
    clock_pose::SimplePose expected_pose;
    expected_pose.x_pos = 1.;
    expected_pose.y_pos = 0.;
    expected_pose.yaw = -M_PI / 2;

    const clock_pose::SimplePose actual_pose = 
        clock_pose::CalculatePoseFromMinutes(num_minutes);

    EXPECT_EQ(expected_pose.x_pos, actual_pose.x_pos);
    EXPECT_EQ(expected_pose.y_pos, actual_pose.y_pos);
    EXPECT_EQ(expected_pose.yaw, actual_pose.yaw);
}
