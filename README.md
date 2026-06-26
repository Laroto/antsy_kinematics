# antsy_kinematics

Kinematics library for ANTSY. This repo is primarily a reusable library consumed by `antsy_control`; it does not provide a configurable production node.

## What is in here

- `antsy_kinematics::Kinematics`: helper class that builds KDL chains from `robot_description`, then exposes:
  - inverse kinematics via `cartToJnt(...)`
  - forward kinematics via `jntToCart(...)`
  - joint-limit clamping via `foldAndClampJointAnglesToLimits(...)`
- `example`: a small executable showing how to subscribe to `robot_description` and call the library

## Nodes

### `example`

Run it with:

```bash
ros2 run antsy_kinematics example
```

Parameters:

- none

Topics:

- subscribes: `robot_description`

## Library configuration

The production behavior of this repo is controlled by constructor arguments, not ROS parameters:

- `feet_links`: ordered list of foot link names for each leg
- `base_link`: base frame used for the KDL chains
- `position_weight`: IK Cartesian position weight
- `orientation_weight`: IK Cartesian orientation weight

Those values are set by the caller, typically `antsy_control/follow_velocity_rectangle`.
