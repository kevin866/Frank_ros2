# OMBot — Omnidirectional Mobile Manipulator

A ROS 2 (Humble) workspace for a mecanum-wheeled mobile base with a 6-DOF arm, built on **ros2_control**. The system supports standalone arm control, standalone base control, and coordinated whole-body control.

## Repository Structure

```
frank_ws/
├── src/
│   ├── ombot_description/     # URDF/Xacro, meshes, RViz configs
│   ├── ombot_hardware/        # ros2_control SystemInterface plugins (arm + base)
│   ├── ombot_controller/      # ros2_control controller plugins (C++)
│   ├── ombot_bringup/         # Launch files & controller YAML config
│   ├── ombot_coordination/    # Python task-level nodes (commanders, trajectory generators)
│   ├── ombot_msgs/            # Custom messages (WholeBodyCmd, Obstacle, ObstacleArray)
│   ├── ombot_base_kinematics/ # Mecanum forward kinematics / odometry node (C++)
│   ├── teleop_core/           # Joystick teleoperation node
│   ├── locomotion_core/       # Low-level Roboteq motor driver nodes (legacy/rover)
│   ├── rover_launch/          # Standalone rover launch (no arm)
│   ├── ombot_sysid/           # System identification data
│   └── zed-ros2-wrapper/      # ZED2 stereo camera driver (submodule)
```

## Hardware

| Component | Details |
|-----------|---------|
| **Arm** | 6-DOF, Dynamixel servos via `/dev/ttyUSB0` @ 1 Mbaud. Effort (torque) + position command interfaces. |
| **Base** | 4 mecanum wheels driven by two Roboteq controllers (`/dev/ttyACM0`, `/dev/ttyACM1`). Velocity command interface. |
| **Camera** | ZED2 stereo camera (optional, for depth-based obstacle avoidance). |
| **Mocap** | OptiTrack via VRPN (optional, for world-frame localization). |

Hardware plugins registered in `ombot_hardware.xml`:
- `ombot_hardware/OMBotArmSystem` — Dynamixel arm interface
- `ombot_hardware/OMBotBaseSystem` — Roboteq mecanum base interface

## Controllers

All controllers are ros2_control plugins defined in `ombot_controller`. Configuration lives in `ombot_bringup/config/ombot_controller.yaml`. The controller manager runs at **250 Hz**.

### Controller Hierarchy

```
[ Task Planner / Teleop / Commander Node ]
        ↓  (task-space targets)
[ Outer Controller ]                        ← chainable
  e.g. ResolvedRate / WholeBody / CartesianImpedance
        ↓  (writes reference interfaces: q_d, dq_d)
[ Inner Controller ]                        ← chainable
  e.g. JointImpedanceController (effort/torque)
        ↓  (effort commands)
[ Hardware Interface ]
```

Outer controllers export reference interfaces that inner controllers claim, using the ros2_control chaining mechanism.

### Arm Controllers (standalone)

| Controller | Plugin Name | Type | Description |
|-----------|------------|------|-------------|
| **GravityCancelController** | `ombot_controller/GravityCancelController` | `ControllerInterface` | Feedforward gravity compensation using KDL inverse dynamics. Reads joint positions, commands effort. |
| **EffortController** | `ombot_controller/EffortController` | `ControllerInterface` | Sends a fixed or topic-driven effort command to each joint. Useful for testing. |
| **JointImpedanceController** | `ombot_controller/JointImpedanceController` | `ChainableControllerInterface` | Joint-space PD + gravity compensation + friction feedforward. Accepts `q_d`, `dq_d`, `tau_ff` via topic or reference interfaces. This is the primary inner (torque-level) controller. |
| **CartesianImpedanceController** | `ombot_controller/CartesianImpedanceController` | `ChainableControllerInterface` | Cartesian-space impedance (stiffness + damping in XYZ/RPY) + gravity comp. Subscribes to desired pose, twist, and optional feedforward wrench. |
| **EeTwistVelocityController** | `ombot_controller/EeTwistVelocityController` | `ControllerInterface` | Task-space velocity controller. Takes a `TwistStamped` command, computes joint velocities via damped-least-squares (DLS) Jacobian pseudoinverse. Supports nullspace posture bias and soft joint limits. |
| **ResolvedRateController** | `ombot_controller/ResolvedRateController` | `ChainableControllerInterface` | Velocity-level resolved-rate (Jacobian) controller for the arm. Integrates desired EE twist into joint references and writes them to the inner `JointImpedanceController` via chaining. Includes DLS damping, nullspace posture control, and joint limit avoidance. |

### Whole-Body Controllers (arm + base)

| Controller | Plugin Name | Description |
|-----------|------------|-------------|
| **WholeBodyResolvedRateController** | `ombot_controller/WholeBodyResolvedRateController` | Extends resolved-rate to 9 DOF (6 arm + 3 base: vx, vy, wz). Distributes task-space velocity across arm joints and base twist using weighted pseudoinverse. Publishes base commands to `/cmd_vel`. |
| **WholeBodyQPController** | `ombot_controller/WholeBodyQPController` | QP-based whole-body controller using OSQP. Optimizes arm + base velocities subject to joint limits. Supports obstacle avoidance via Control Barrier Functions (CBF) from an `ObstacleArray` topic. Publishes base commands to `/mecanum_controller/reference`. |

### Calibration & System ID Controllers

| Controller | Plugin Name | Description |
|-----------|------------|-------------|
| **JointExcitationController** | `ombot_controller/JointExcitationController` | Applies PRBS / sine / chirp torque excitation signals for system identification. |
| **JointVelocityExcitationController** | `ombot_controller/JointVelocityExcitationController` | Velocity-domain excitation for sysid. |
| **JointFrictionCalibController** | `ombot_controller/JointFrictionCalibController` | Automated friction calibration: sweeps each joint through velocities, measures torque, and fits Coulomb + viscous friction parameters. |

### Base Controller

| Controller | Type | Description |
|-----------|------|-------------|
| **mecanum_controller** | `mecanum_drive_controller/MecanumDriveController` | Standard ROS 2 mecanum drive controller. Accepts `TwistStamped` on `/cmd_vel`, publishes odometry. |
| **wheel_velocity_controller** | `velocity_controllers/JointGroupVelocityController` | Direct wheel velocity control (alternative to mecanum controller). |

## Custom Messages (`ombot_msgs`)

| Message | Fields | Usage |
|---------|--------|-------|
| `WholeBodyCmd` | `ee` (Twist), `bvx`, `bvy`, `bwz`, `valid` | Combined EE + base velocity command for whole-body controllers. |
| `Obstacle` | `id`, `center`, `radius`, `velocity` | Single obstacle for CBF avoidance. |
| `ObstacleArray` | `header`, `obstacles[]` | Array of obstacles for the QP controller. |

## Launch Files (`ombot_bringup`)

### Primary Launch Files

| Launch File | What It Starts | Use Case |
|-------------|---------------|----------|
| `ombot.launch.py` | Full robot: RSP + controller_manager + JSB + mecanum + joint_impedance + RViz + ee2base node | **Main bringup** for arm+base with impedance control and EE-to-base tracking. |
| `minimum.launch.py` | RSP + controller_manager + JSB + wheel_velocity_controller | Minimal bringup for base-only testing. |
| `gravity_controller.launch.py` | RSP + controller_manager + JSB + gravity_cancel_controller + RViz | Arm-only gravity compensation (arm hangs limp with gravity cancelled). |
| `joint_impedance_controller.launch.py` | RSP + controller_manager + JSB + joint_impedance_controller | Arm-only joint impedance control (holds position with compliance). |
| `cartesian_impedance_controller.launch.py` | RSP + controller_manager + JSB + cartesian_impedance_controller | Arm-only Cartesian impedance control. |

### Chained / Resolved-Rate Launch Files

| Launch File | Controller Chain | Use Case |
|-------------|-----------------|----------|
| `chainable_controllers.launch.py` | JSB → JointImpedance → ResolvedRateController (+ optional RViz) | Arm-only resolved-rate. Send `TwistStamped` to `/resolved_rate_controller/ee_twist` to move the end-effector. Supports `chained:=true/false` arg. |
| `ee_twist_vel_ctrl_with_pub.launch.py` | JSB → JointImpedance → EeTwistVelocityController + twist publisher | Arm-only EE velocity control with a built-in Python twist publisher (mode: const/circle/yaw/zero). |

### Whole-Body Launch Files

| Launch File | Controller Chain | Use Case |
|-------------|-----------------|----------|
| `bringup_wb_with_bag.launch.py` | JSB → JointImpedance → WholeBodyResolvedRate + mecanum + OptiTrack TF + goal commander + whole_body_task_commander + rosbag | **Whole-body resolved-rate** with goal tracking and data recording. |
| `bringup_qp.launch.py` | JSB → JointImpedance → WholeBodyQPController + mecanum + OptiTrack TF + wb_cmd_publisher + rosbag | **Whole-body QP** controller bringup with data recording. |
| `qp_moveaway.launch.py` | JSB → JointImpedance → WholeBodyQPController + mecanum + ZED camera + depth-based moveaway node + rosbag | QP whole-body with **depth-based obstacle avoidance** (ZED camera). |
| `moveaway_goto.launch.py` | Same as above + goal-seeking behavior | QP whole-body with obstacle avoidance **and** autonomous go-to-goal navigation. |

### Other Launch Files

| Launch File | Description |
|-------------|-------------|
| `ombot_base_controller.launch.py` | Base-only with mecanum controller (no arm). |
| `robot_camera.launch.py` | Full robot + ZED camera. |
| `split_commander.launch.py` | Whole-body with split base/arm commander. |
| `joint_friction_calib.launch.py` | Friction calibration controller. |
| `ombot_joint_excitation.launch.py` | Joint excitation for system ID. |
| `velocity_controller.launch.py` | Direct velocity control. |
| `rr_tester.launch.py` | Resolved-rate tester. |

## Coordination Nodes (`ombot_coordination`)

Python nodes for task-level planning and command generation:

| Executable | Description |
|-----------|-------------|
| `whole_body_task_commander` | Computes EE twist from goal pose error (PD control in task space) and publishes to the whole-body controller. |
| `whole_body_cmd_publisher` | Publishes `WholeBodyCmd` messages with constant or scripted base + EE velocities. |
| `depth_move_away_cmd_publisher` | Uses ZED depth to detect obstacles; commands backward base motion when an obstacle is too close. |
| `moveaway_goto_cmd` | Combines depth-based obstacle avoidance with proportional go-to-goal navigation. |
| `split_commander` | Decomposes a world-frame goal into separate base and arm commands. |
| `ee_twist_cmd_publisher` | Publishes `TwistStamped` for EE velocity control (modes: const, circle, yaw, zero). |
| `ee_trajectory_generator` | Generates smooth EE trajectories (e.g., for resolved-rate following). |
| `optitrack_tf_pub` | Subscribes to VRPN mocap poses and publishes `world→base_link` TF. |
| `goal_commander` | Publishes a static goal pose at a fixed rate. |
| `arm_base_coordinator` | P-controller that drives the base to keep the arm's EE centered. |
| `resolved_rate_tester` | Test node for the resolved-rate controller. |

## Quick Start

### Prerequisites
- ROS 2 Humble
- `ros2_control`, `ros2_controllers`
- KDL (`kdl_parser`, `orocos_kdl`)
- Eigen3
- OSQP + OsqpEigen (for QP controller)
- Dynamixel SDK (for arm hardware)
- ZED SDK + `zed-ros2-wrapper` (optional, for camera)

### Build

```bash
cd ~/frank_ws
colcon build --symlink-install
source install/setup.bash
```

### Run: Arm-Only (Gravity Compensation)

```bash
ros2 launch ombot_bringup gravity_controller.launch.py
```

### Run: Arm-Only (Joint Impedance)

```bash
ros2 launch ombot_bringup joint_impedance_controller.launch.py
```

### Run: Arm-Only (Resolved Rate)

```bash
ros2 launch ombot_bringup chainable_controllers.launch.py chained:=true
# Then send twist commands:
ros2 topic pub /resolved_rate_controller/ee_twist geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.05}}}"
```

### Run: Full Robot (Arm + Base with Impedance)

```bash
ros2 launch ombot_bringup ombot.launch.py
```

### Run: Whole-Body QP Controller

```bash
ros2 launch ombot_bringup bringup_qp.launch.py
```

### Run: Whole-Body with Obstacle Avoidance

```bash
ros2 launch ombot_bringup qp_moveaway.launch.py launch_zed:=true
```

### Drive the Base Manually

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.3, y: 0.0}, angular: {z: 0.0}}}"
```

## Configuration

All controller parameters are in `src/ombot_bringup/config/ombot_controller.yaml`. Key tuning parameters:

- **Joint Impedance**: `Kp`, `Kd` (per-joint stiffness/damping), friction parameters (`Fc`, `B`)
- **Resolved Rate**: `lambda` (DLS damping), `null_kp`/`null_kd` (nullspace posture), `q_home`, `qdot_limit`
- **Whole-Body**: `base_weight`/`arm_weight` (cost allocation), `base_cmd_scale`
- **Cartesian Impedance**: `Kp_xyz`, `Kd_xyz`, `Kr_rpy`, `Dr_rpy`, wrench limits
- **Mecanum**: `wheel_separation_x/y`, `wheels_radius`, `sum_of_robot_center_projection_on_X_Y_axis`

## Author

Kevin Wang
