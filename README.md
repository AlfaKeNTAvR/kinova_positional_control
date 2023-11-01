# ROS kinova_positional_control

## Description

- The `kinova_positional_control` ROS package is designed to provide positional control for Kinova Gen 3 7DoF robot arm using `relaxed_ik`(collision_ik) and `kinova_pid` packages.

- `relaxed_ik` package is a ROS package that provides an inverse kinematics solver for manipulators. `relaxed_ik` uses a relaxation strategy to generate solutions that are not necessarily exact, but that satisfy the constraints of the robot arm. The relaxation strategy allows the solver to find feasible solutions even in situations where the problem is ill-posed or under-constrained.

- `kinova_pid` is a ROS package that provides a PID controller for joint-level control of robot arms. It allows users to set desired joint positions and velocities and provides feedback on the current joint positions and velocities.

- `kinova_positional_control` ROS package uses these two packages to provide positional control for Kinova Gen 3 7DoF robot arm. It takes as input the desired end-effector position and orientation, and uses the `relaxed_ik` package to compute the desired joint positions that achieve this position and orientation. It then uses the `kinova_pid` package to control the robot arm and move it to the desired joint positions.

## Usage

Currently, `user-study` branch should be used. The system requires an Oculus Quest 2 as input for the end-effector position and orientation control. To install and run Kinova Gen 3 positional control with different motorized chest control modes follow the instructions below.

## Requirements

1. [relaxed_ik_ros1](https://github.com/GOPHER-System-Intergration/relaxed_ik_ros1)

2. [kinova_pid](https://github.com/GOPHER-System-Intergration/kinova_pid)

3. [gopher_ros_clearcore](https://github.com/GOPHER-System-Intergration/gopher_ros_clearcore)

## Installation

To install the package, clone the repository into your Catkin workspace:

1. Navigate to your Catkin workspace's `src` directory:

   ```
   cd ~/catkin_ws/src
   ```

2. Clone the repository:

   ```
   git clone -b user-study https://github.com/GOPHER-System-Intergration/kinova_positional_control.git
   ```


3. Build the package:

   ```
   cd ~/catkin_ws
   catkin_make
   ```

4. Source the workspace:

   ```
   source ~/catkin_ws/devel/setup.bash
   ```

## Running the Package

To run the package run the following python file:

1. Manual chest control mode:

```
python oculus_chest_control.py --mode=0
```

2. Proximity chest control mode:

```
python oculus_chest_control.py --mode=1
```

3. Scaling chest control mode:

```
python oculus_chest_control.py --mode=2
```

## Work-in-progress

The code from the `user-study` is currently being refactored on a `refactor` branch. We pay extra attention to better modularity and flexibility of this new codebase.