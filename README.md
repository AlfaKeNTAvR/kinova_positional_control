# ROS kinova_positional_control

## Description

- The `kinova_positional_control` ROS package is designed to provide positional control for Kinova Gen 3 7DoF robot arm using `relaxed_ik`(collision_ik) and `kinova_pid` packages.

- `relaxed_ik` package is a ROS package that provides an inverse kinematics solver for manipulators. `relaxed_ik` uses a relaxation strategy to generate solutions that are not necessarily exact, but that satisfy the constraints of the robot arm. The relaxation strategy allows the solver to find feasible solutions even in situations where the problem is ill-posed or under-constrained.

- `kinova_pid` is a ROS package that provides a PID controller for joint-level control of robot arms. It allows users to set desired joint positions and velocities and provides feedback on the current joint positions and velocities.

- `kinova_positional_control` ROS package uses these two packages to provide positional control for Kinova Gen 3 7DoF robot arm. It takes as input the desired end-effector position and orientation, and uses the `relaxed_ik` package to compute the desired joint positions that achieve this position and orientation. It then uses the `kinova_pid` package to control the robot arm and move it to the desired joint positions.