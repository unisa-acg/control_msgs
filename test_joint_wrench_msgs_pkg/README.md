# test_joint_wrench_msgs

This package introduces two nodes: an action client and an action server. The goal is to test the following message definitions:

1. `JointWrenchTrajectory.msg`
2. `JointWrenchTrajectoryPoint.msg`

And the corresponding action definition, `FollowJointWrenchTrajectory.action`.

## How to test

First of all, from the workspace folder build and source the code:

```bash
colcon build --packages-up-to test_joint_wrench_msgs_pkg
source install/setup.bash
```

Then run:

```bash
ros2 run test_joint_wrench_msgs_pkg test_joint_wrench_msgs_action_server_node
```

I a new shell, run:

```bash
ros2 run test_joint_wrench_msgs_pkg test_joint_wrench_msgs_action_client_node
```