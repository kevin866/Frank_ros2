# Frank ROS 2 Workspace

This repository contains the ROS 2 workspace for `Frank`, a mobile manipulator with:

- a mecanum base
- a 6-DOF arm
- custom `ros2_control` hardware interfaces
- custom chained controllers for arm and whole-body control
- coordination nodes for arm/base behaviors
- launch files for bringup, experiments, logging, and calibration

The code is primarily set up for real robot bringup and controller experiments on ROS 2 Humble.

## Metadata

- Author: Kevin Wang
- License: Apache-2.0

## What Is In This Repo

The most important packages are:

- `src/ombot_bringup`: launch files and controller configuration
- `src/ombot_controller`: custom C++ controller plugins and helper nodes
- `src/ombot_hardware`: `ros2_control` hardware interfaces for the arm and base
- `src/ombot_description`: URDF/xacro and RViz configuration
- `src/ombot_coordination`: Python nodes for high-level command generation and experiments
- `src/ombot_msgs`: custom messages, including `WholeBodyCmd`
- `src/ombot_base_kinematics`: mecanum base kinematics

Also present in the workspace:

- `src/zed-ros2-wrapper`: vendored ZED ROS 2 driver
- `src/osqp-eigen`: vendored QP dependency used by the whole-body QP controller
- `src/teleop_core`, `src/locomotion_core`, `src/rover_launch`: rover-related code that appears separate from the main Ombot stack

## Control Architecture

The core pattern in this repo is chained control:

1. A high-level node or controller generates task-space commands.
2. A whole-body or resolved-rate controller converts those commands into joint references.
3. The joint impedance controller executes those references as effort commands.
4. The hardware interface sends commands to the real arm and base.

In short:

`task command -> outer controller -> joint impedance -> hardware`

This chaining is intentional and is reflected in both the controller implementations and the launch files. The note in [controller_structure.txt](/home/frank/frank_ws/src/ombot_controller/controller_structure.txt) describes the same pattern.

## Main Packages

### `ombot_controller`

This is the core control package. It exports multiple `ros2_control` controller plugins.

Controllers defined in [ombot_controller.yaml](/home/frank/frank_ws/src/ombot_bringup/config/ombot_controller.yaml):

- `joint_impedance_controller`: inner-loop arm controller; consumes joint position/velocity references and sends effort commands
- `gravity_cancel_controller`: gravity compensation only
- `cartesian_impedance_controller`: Cartesian arm impedance controller with pose/twist/wrench inputs
- `resolved_rate_controller`: arm resolved-rate controller; writes references into the joint impedance controller
- `wb_resolved_rate_controller`: whole-body resolved-rate controller; coordinates arm references and base velocity
- `wb_qp_controller`: whole-body QP controller; consumes `WholeBodyCmd` and publishes base motion while feeding the inner arm controller
- `ee_twist_velocity_controller`: converts end-effector twist commands into joint velocity commands
- `effort_controller`: simple direct effort command path
- `joint_excitation_controller`: torque excitation for system ID
- `joint_velocity_excitation_controller`: velocity excitation for system ID
- `joint_friction_calib_controller`: friction identification/calibration workflow

Helper executables in this package:

- `ee_to_base`: converts `/ee_pose` into base velocity commands on `/mecanum_controller/reference`
- `mecanum_forward_cmd`: publishes a forward command for the mecanum base

### `ombot_coordination`

This package provides the high-level experiment nodes that feed the controllers. The most important ones referenced by launch files are:

- `arm_base_coordinator`
- `whole_body_task_commander`
- `whole_body_cmd_publisher`
- `goal_commander`
- `goal_from_base_offset_latched`
- `split_commander`
- `resolved_rate_tester`
- `optitrack_tf_pub`
- `moveaway_goto_cmd`
- `depth_move_away_cmd_publisher`
- trajectory publishers such as `ee_trajectory_generator` and `dual_traj_generator`

### `ombot_hardware`

This package contains the robot hardware interfaces:

- arm hardware interface
- base hardware interface
- Roboteq interface support

### `ombot_description`

This package contains:

- the robot xacro/URDF files
- base and arm mesh assets
- RViz configuration
- a description-only launch file for visualization

## Controller Relationships

If you are new to the repo, the controllers below matter most:

### Inner controller

- `joint_impedance_controller`

This is the controller most other arm controllers feed into. It exports per-joint position and velocity reference interfaces, and outer controllers write into those references.

### Outer arm controller

- `resolved_rate_controller`

This subscribes to `~/ee_twist` and computes joint references for the impedance controller. Use this for arm-only task-space control.

### Whole-body controllers

- `wb_resolved_rate_controller`
- `wb_qp_controller`

These combine arm control with base motion. They use the arm as a chained inner loop and publish base commands to `/mecanum_controller/reference`.

### Calibration / test controllers

- `gravity_cancel_controller`
- `joint_excitation_controller`
- `joint_velocity_excitation_controller`
- `joint_friction_calib_controller`
- `cartesian_impedance_controller`
- `ee_twist_velocity_controller`

These are mainly for isolated testing, tuning, and data collection rather than full robot operation.

## Launch Files That Matter Most

There are many launch files in `src/ombot_bringup/launch`. Most are experiment-specific. Start with the ones below.

### 1. Basic robot bringup

File: [ombot.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/ombot.launch.py)

What it launches:

- `ros2_control_node`
- `robot_state_publisher`
- `joint_state_broadcaster`
- `mecanum_controller`
- `joint_impedance_controller`
- `ee_to_base`
- optional RViz

Use this when:

- you want the robot online with the base and arm stack loaded
- you want the simplest real robot bringup path

Example:

```bash
ros2 launch ombot_bringup ombot.launch.py
```

Useful launch args:

- `gui:=true|false`
- `launch_zed:=true|false`

### 2. Resolved-rate coordination with bagging

File: [arm_base_coordination.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/arm_base_coordination.launch.py)

What it launches:

- `joint_state_broadcaster`
- `joint_impedance_controller`
- `resolved_rate_controller`
- `mecanum_controller`
- `arm_base_coordinator`
- rosbag recording

Use this when:

- you want coordinated base and arm experiments
- you want automatic bagging of experiment topics

Example:

```bash
ros2 launch ombot_bringup arm_base_coordination.launch.py
```

### 3. Resolved-rate bringup with OptiTrack and a task commander

File: [bringup_rr_with_bag.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/bringup_rr_with_bag.launch.py)

What it launches:

- `optitrack_tf_pub`
- `goal_commander`
- `joint_state_broadcaster`
- `joint_impedance_controller`
- `resolved_rate_controller`
- `mecanum_controller`
- `whole_body_task_commander`
- rosbag recording

Use this when:

- you want world-frame goal experiments
- you are using OptiTrack / VRPN feedback
- you want the resolved-rate pipeline with logging

Example:

```bash
ros2 launch ombot_bringup bringup_rr_with_bag.launch.py bag_prefix:=ombot_run1
```

### 4. Whole-body resolved-rate bringup

File: [bringup_wb_with_bag.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/bringup_wb_with_bag.launch.py)

What it launches:

- `optitrack_tf_pub`
- `goal_commander`
- `joint_state_broadcaster`
- `joint_impedance_controller`
- `wb_resolved_rate_controller`
- `mecanum_controller`
- `whole_body_task_commander`
- rosbag recording

Use this when:

- you want the whole-body resolved-rate controller instead of the arm-only resolved-rate controller

### 5. Whole-body QP bringup

File: [bringup_qp.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/bringup_qp.launch.py)

What it launches:

- `optitrack_tf_pub`
- `joint_state_broadcaster`
- `joint_impedance_controller`
- `wb_qp_controller`
- `mecanum_controller`
- `whole_body_cmd_publisher`
- rosbag recording

Use this when:

- you want to test the QP-based whole-body controller
- you are publishing `WholeBodyCmd` style commands

### 6. Description-only visualization

File: [view_ombot.launch.py](/home/frank/frank_ws/src/ombot_description/launch/view_ombot.launch.py)

What it launches:

- `robot_state_publisher`
- `joint_state_publisher` or `joint_state_publisher_gui`
- optional RViz

Use this when:

- you only want to inspect the robot model
- you do not want to connect hardware or controllers

Example:

```bash
ros2 launch ombot_description view_ombot.launch.py start_rviz:=true
```

## Other Useful Launch Files

These are narrower tools for testing specific controllers or behaviors:

- [joint_impedance_controller.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/joint_impedance_controller.launch.py): isolate the inner impedance controller
- [gravity_controller.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/gravity_controller.launch.py): gravity compensation only
- [cartesian_impedance_controller.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/cartesian_impedance_controller.launch.py): Cartesian impedance tests
- [ombot_effort_controller.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/ombot_effort_controller.launch.py): direct effort controller tests
- [ombot_joint_excitation.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/ombot_joint_excitation.launch.py): torque excitation runs
- [velocity_controller.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/velocity_controller.launch.py): joint velocity excitation runs
- [joint_friction_calib.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/joint_friction_calib.launch.py): friction fitting workflow
- [rr_tester.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/rr_tester.launch.py): resolved-rate test setup with `resolved_rate_tester`
- [split_commander.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/split_commander.launch.py): split base/arm command generation
- [split_cmd_vel.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/split_cmd_vel.launch.py): `ee_twist_velocity_controller` plus split commander
- [wb_with_traj.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/wb_with_traj.launch.py): whole-body resolved-rate with trajectory generation
- [bringup_traj_bag.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/bringup_traj_bag.launch.py): QP bringup with dual trajectory generation and bagging
- [moveaway_goto.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/moveaway_goto.launch.py): move-away / go-to whole-body experiments
- [qp_moveaway.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/qp_moveaway.launch.py): move-away experiment using the QP controller
- [ombot_base_controller.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/ombot_base_controller.launch.py): base-only mecanum controller bringup
- [minimum.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/minimum.launch.py): minimal base controller path

## Common Topics And Interfaces

These are the topics that appear repeatedly across controllers and launch files:

- `/joint_states`: arm state feedback
- `/ee_pose`: end-effector pose, used by several coordinators and helper nodes
- `/goal_pose`: target pose for experiments
- `/mecanum_controller/reference`: base velocity command topic
- `/vrpn_mocap/RigidBody_1/pose`: base pose from OptiTrack / VRPN
- `/vrpn_mocap/RigidBody_2/pose`: second tracked rigid body, typically used in experiments
- `/wb_cmd`: whole-body command input for the QP / whole-body stacks

Important controller-local topics:

- `resolved_rate_controller/ee_twist`
- `joint_impedance_controller/command`
- `cartesian_impedance_controller/command_pose`
- `cartesian_impedance_controller/command_twist`
- `joint_excitation_controller/tau_cmd`
- `joint_velocity_excitation_controller/qdot_cmd`

## Building The Workspace

Typical build flow:

```bash
source /opt/ros/humble/setup.bash
cd /home/frank/frank_ws
colcon build --symlink-install
source install/setup.bash
```

If you only need the main robot stack, these packages are the ones to care about first:

- `ombot_description`
- `ombot_msgs`
- `ombot_base_kinematics`
- `ombot_hardware`
- `ombot_controller`
- `ombot_coordination`
- `ombot_bringup`

## Typical First-Time Workflow

### Visualize the robot model

```bash
ros2 launch ombot_description view_ombot.launch.py start_rviz:=true
```

### Bring up the robot with base and inner arm controller

```bash
ros2 launch ombot_bringup ombot.launch.py
```

### Run a resolved-rate experiment with logging

```bash
ros2 launch ombot_bringup bringup_rr_with_bag.launch.py bag_prefix:=test_run
```

### Run a whole-body QP experiment

```bash
ros2 launch ombot_bringup bringup_qp.launch.py bag_prefix:=qp_run
```

## Notes For Contributors

- This repo is organized around real hardware, not a complete simulation stack.
- Many launch files are experiment-specific and encode assumptions about topics, frames, gains, and OptiTrack setup.
- The controller YAML in [ombot_controller.yaml](/home/frank/frank_ws/src/ombot_bringup/config/ombot_controller.yaml) is the best single place to inspect active controller names and default gains.
- If you are adding a new high-level controller, follow the existing chained-controller pattern instead of bypassing the inner impedance loop unless you have a specific reason to do so.

## Where To Start Reading

If you want to understand the repo quickly, read files in this order:

1. [ombot.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/ombot.launch.py)
2. [ombot_controller.yaml](/home/frank/frank_ws/src/ombot_bringup/config/ombot_controller.yaml)
3. [joint_impedance_controller.cpp](/home/frank/frank_ws/src/ombot_controller/src/joint_impedance_controller.cpp)
4. [resolved_rate_controller.cpp](/home/frank/frank_ws/src/ombot_controller/src/resolved_rate_controller.cpp)
5. [wb_resolved_rate_controller.cpp](/home/frank/frank_ws/src/ombot_controller/src/wb_resolved_rate_controller.cpp)
6. [wb_qp_controller.cpp](/home/frank/frank_ws/src/ombot_controller/src/wb_qp_controller.cpp)
7. [arm_base_coordination.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/arm_base_coordination.launch.py)
8. [bringup_rr_with_bag.launch.py](/home/frank/frank_ws/src/ombot_bringup/launch/bringup_rr_with_bag.launch.py)
