# ros2_clock

Provides functionality to publish poses based on the minute hand of a clock.

There are two main publishers:
- Clock Pose Issuer
- GUI Pose Issuer

There are a few ways to ingest this information, either by directly subscribing
to the topics themselves, or by interfacing with a thin client that subscribes
to both, and publishes the target pose.

## Clock Pose Issuer

### Tasks Remaining
- Allow for parameter changes to update the radius.

## GUI Pose Issuer

### Tasks Remaining
- Allow for parameter changes to update the radius, and duration that the target
    pose is valid.

## Target Pose Issuer
A convenience client that allows for the motion controller to subscribe to a
target pose without having to reimplement the logic to change target poses.
