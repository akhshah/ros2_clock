# ros2_clock

Provides functionality to publish poses based on the minute hand of a clock.

There are two main publishers:
- Clock Pose Issuer
- GUI Pose Issuer

There are a few ways to ingest this information, either by directly subscribing
to the topics themselves, or by interfacing with a thin client that subscribes
to both, and publishes the target pose.

The thin client is called Target Pose Issuer and subscribes to the two above
publishers and publishes a target pose.

The code is expected to be built on Jazzy, and has been tested on Ubuntu 24.04.

For full convenience, this functionality should be containerized in Docker for
use on other platforms, but has not yet been done.

## Clock Pose Issuer
Automatically issues a pose based on the minute hand of the current local time.
The poses assume that the yaw will follow in the clockwise direction.

Publishes a `StampedPose` on the `clock_pose` topic.

The testing for the utility functions relies on `GTest`, but that is provided by
`ament`.

### Tasks Remaining
- Allow for parameter changes to update the radius.
- Allow for parameter changes to update the yaw, e.g., go backwards.

## GUI Pose Issuer
Displays a Qt window with a rudimentary clock face. This publisher will publish
posed based on where the user clicks on the window. The user can also clear the
selected pose by pressing space bar on their keyboard.

After 30 seconds, the pose should clear.

Clearing the pose, means broadcasting an empty `StampedPose` and end users
should understand that the pose is not to be used by checking the stamp in the
header.

### Tasks Remaining
- Allow for parameter changes to update the radius, and duration that the target
    pose is valid.
- Allow for parameter changes to update the yaw.
- Fix the displayed point to be constrained on the radius of the clock face.
- Fix the displayed point to update when resizing the window.
- Instead of using the timestamp in the header, should broadcast a validity flag
    alongside the `StampedPose`.

## Target Pose Issuer
A convenience client that allows for the motion controller to subscribe to a
target pose without having to reimplement the logic to change target poses.

Will continuously publish clock poses, if there isn't a valid GUI pose.

### Tasks Remaining
- Confirm that the clock pose is different from the last one, and then publish.

## Running the issuers
1. Ensure that your ROS2 workspace is instantiated by running `source
/opt/ros/<ros-version>/setup.<preferred shell>`.

2. `colcon build` to build all the packages.

3. Run
```
source install/setup.<preferred shell>
```
4. Assuming you are in the parent directory, e.g., `ros2_clock/`, run:
```
ros2 launch launch/pose_issuer.launch.py
```
for headless mode.

Otherwise, run:
```
ros2 launch launch/pose_issuer.launch.py headless:=False
```
to run with the GUI enabled.
