#!/usr/bin/env python3
# Copyright (c) 2025
#
# Arm+Base coordinator for "walk-and-retract" behavior:
#  - Computes a single 6D task-space error (EE to goal) in base frame
#  - Builds desired twist v_task
#  - Blends commands between base and arm with a smooth factor alpha in [0,1]
#  - Near the goal, increases alpha->1 and (optionally) raises the arm controller's null_kp
#
# Assumptions:
#  - All poses (EE, goal, base) are in the SAME frame (usually base or map)
#  - EE and goal are geometry_msgs/PoseStamped
#  - Base pose is geometry_msgs/PoseStamped (position.xy + yaw used)
#  - Arm resolved-rate controller consumes geometry_msgs/TwistStamped at
#      /resolved_rate_controller/ee_twist
#  - Base consumes geometry_msgs/Twist at /cmd_vel
#  - (Optional) Publish null_kp as std_msgs/Float64 to a topic if provided
#
# Parameters are exposed to tune behavior without code edits.

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from std_msgs.msg import Float64
import numpy as np


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def sigmoid(x: float) -> float:
    return 1.0 / (1.0 + math.exp(-x))


def quat_to_rotvec(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """Quaternion -> rotation vector (axis * angle), returns 3-vector.
    For small angles, result ~ 2*vector_part (since sin(theta/2) ~ theta/2).
    """
    # Normalize to be safe
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n == 0.0:
        return (0.0, 0.0, 0.0)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    angle = 2.0 * math.acos(clamp(qw, -1.0, 1.0))
    s = math.sqrt(max(1.0 - qw*qw, 0.0))
    if s < 1e-8:
        # If s ~ 0, axis not well-defined; use vector part directly
        return (qx*angle, qy*angle, qz*angle)
    ax, ay, az = qx/s, qy/s, qz/s
    return (ax*angle, ay*angle, az*angle)


def quat_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    # Assuming standard XY-plane yaw
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny_cosp, cosy_cosp)


def rotmat_from_quat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Quaternion -> 3x3 rotation matrix."""
    # normalize
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = qx/n, qy/n, qz/n, qw/n
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([[1-2*(yy+zz),   2*(xy-wz),     2*(xz+wy)],
                     [  2*(xy+wz), 1-2*(xx+zz),    2*(yz-wx)],
                     [  2*(xz-wy),   2*(yz+wx),  1-2*(xx+yy)]])


@dataclass
class LPF:
    alpha: float
    y: float = 0.0
    def step(self, x: float) -> float:
        # y_k = alpha*x_k + (1-alpha)*y_{k-1}
        self.y = self.alpha * x + (1.0 - self.alpha) * self.y
        return self.y


@dataclass
class SlewLimiter:
    max_rate: float  # units per second
    last: float = 0.0
    def step(self, x: float, dt: float) -> float:
        dl = x - self.last
        max_dl = self.max_rate * dt
        if dl > max_dl:
            x = self.last + max_dl
        elif dl < -max_dl:
            x = self.last - max_dl
        self.last = x
        return x


def rotmat_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0],[sy, cy, 0],[0,0,1]])
    Ry = np.array([[cp, 0, sp],[0,1,0],[-sp,0,cp]])
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    return (Rz @ Ry @ Rx)

class ArmBaseCoordinator(Node):
    def __init__(self):
        super().__init__('arm_base_coordinator')

        # --- Parameters ---
        # Topics
        self.ee_pose_topic = self.declare_parameter('ee_pose_topic', '/ee_pose').get_parameter_value().string_value
        self.goal_pose_topic = self.declare_parameter('goal_pose_topic', '/goal_pose').get_parameter_value().string_value
        self.base_pose_topic = self.declare_parameter('base_pose_topic', '/base_pose').get_parameter_value().string_value
        self.inputs_in_world = self.declare_parameter('inputs_in_world', True).get_parameter_value().bool_value  # set True if OptiTrack/world frame
        self.world_frame_name = self.declare_parameter('world_frame_name', 'world').get_parameter_value().string_value
        # If OptiTrack rigid body frame != base_link, set marker->base static offset (meters, radians)
        self.base_marker_offset_xyz = self.declare_parameter('base_marker_offset_xyz', [0.0, 0.0, 0.0]).get_parameter_value().double_array_value
        self.base_marker_offset_rpy = self.declare_parameter('base_marker_offset_rpy', [0.0, 0.0, 0.0]).get_parameter_value().double_array_value
        self.ee_twist_topic = self.declare_parameter('ee_twist_topic', '/resolved_rate_controller/ee_twist').get_parameter_value().string_value
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').get_parameter_value().string_value
        self.null_kp_topic = self.declare_parameter('null_kp_topic', '').get_parameter_value().string_value  # optional

        # Task gains
        self.kp_lin = self.declare_parameter('kp_lin', 1.2).get_parameter_value().double_value
        self.kd_lin = self.declare_parameter('kd_lin', 0.3).get_parameter_value().double_value
        self.kp_ang = self.declare_parameter('kp_ang', 1.0).get_parameter_value().double_value
        self.kd_ang = self.declare_parameter('kd_ang', 0.25).get_parameter_value().double_value
        self.k_ori_w = self.declare_parameter('k_ori_weight', 0.5).get_parameter_value().double_value  # scale orientation error -> meters-equivalent

        # Blending
        self.k_d = self.declare_parameter('blend_slope', 6.0).get_parameter_value().double_value
        self.d_mid = self.declare_parameter('blend_mid_distance', 0.8).get_parameter_value().double_value
        self.max_reach = self.declare_parameter('max_reach', 0.8).get_parameter_value().double_value

        # Retract hysteresis
        self.d_retract_enter = self.declare_parameter('d_retract_enter', 0.25).get_parameter_value().double_value
        self.d_retract_exit = self.declare_parameter('d_retract_exit', 0.35).get_parameter_value().double_value
        self.null_kp_normal = self.declare_parameter('null_kp_normal', 0.05).get_parameter_value().double_value
        self.null_kp_retract = self.declare_parameter('null_kp_retract', 0.3).get_parameter_value().double_value

        # Limits
        self.ee_lin_lim = self.declare_parameter('ee_lin_limit', 0.15).get_parameter_value().double_value  # m/s
        self.ee_ang_lim = self.declare_parameter('ee_ang_limit', 0.6).get_parameter_value().double_value   # rad/s
        self.base_lin_lim = self.declare_parameter('base_lin_limit', 0.30).get_parameter_value().double_value
        self.base_ang_lim = self.declare_parameter('base_ang_limit', 0.80).get_parameter_value().double_value

        # Base kinematics
        self.base_is_holonomic = self.declare_parameter('base_is_holonomic', False).get_parameter_value().bool_value
        self.k_heading = self.declare_parameter('k_heading', 1.5).get_parameter_value().double_value

        # Filters
        alpha_v = self.declare_parameter('vel_lpf_alpha', 0.6).get_parameter_value().double_value
        slew_base = self.declare_parameter('slew_base', 1.0).get_parameter_value().double_value  # units/s
        slew_arm_lin = self.declare_parameter('slew_arm_lin', 0.5).get_parameter_value().double_value
        slew_arm_ang = self.declare_parameter('slew_arm_ang', 1.5).get_parameter_value().double_value

        # State
        self.last_e6 = [0.0]*6
        self.have_ee = False
        self.have_goal = False
        self.have_base = False
        self.retract_mode = False
        self.last_time = self.get_clock().now()

        # IO
        self.sub_ee = self.create_subscription(PoseStamped, self.ee_pose_topic, self.on_ee, 10)
        self.sub_goal = self.create_subscription(PoseStamped, self.goal_pose_topic, self.on_goal, 10)
        self.sub_base = self.create_subscription(PoseStamped, self.base_pose_topic, self.on_base, 10)
        self.pub_ee_twist = self.create_publisher(TwistStamped, self.ee_twist_topic, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_null_kp = self.create_publisher(Float64, self.null_kp_topic, 10) if self.null_kp_topic else None

        # Filters & slew limiters
        self.lpf_base_vx = LPF(alpha_v)
        self.lpf_base_vy = LPF(alpha_v)
        self.lpf_base_wz = LPF(alpha_v)
        self.lpf_arm_vx = LPF(alpha_v)
        self.lpf_arm_vy = LPF(alpha_v)
        self.lpf_arm_vz = LPF(alpha_v)
        self.lpf_arm_wx = LPF(alpha_v)
        self.lpf_arm_wy = LPF(alpha_v)
        self.lpf_arm_wz = LPF(alpha_v)

        self.slew_base_vx = SlewLimiter(slew_base)
        self.slew_base_vy = SlewLimiter(slew_base)
        self.slew_base_wz = SlewLimiter(slew_base)
        self.slew_arm_lin = SlewLimiter(slew_arm_lin)
        self.slew_arm_ang = SlewLimiter(slew_arm_ang)

        self.timer = self.create_timer(0.01, self.spin)  # 100 Hz

        self.get_logger().info('ArmBaseCoordinator ready.')

    # --- Callbacks ---
    def on_ee(self, msg: PoseStamped):
        self.ee = msg
        self.have_ee = True

    def on_goal(self, msg: PoseStamped):
        self.goal = msg
        self.have_goal = True

    def on_base(self, msg: PoseStamped):
        self.base = msg
        self.have_base = True

    # --- Main loop ---
    def spin(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-3
        self.last_time = now

        if not (self.have_ee and self.have_goal and self.have_base):
            return

        # 1) Build 6D task error e6 in base frame
        if self.inputs_in_world:
            # Transform world poses into base frame: T_be = T_bw * T_we, T_bg = T_bw * T_wg
            # Positions
            pb = np.array([self.base.pose.position.x, self.base.pose.position.y, self.base.pose.position.z])
            pe = np.array([self.ee.pose.position.x, self.ee.pose.position.y, self.ee.pose.position.z])
            pg = np.array([self.goal.pose.position.x, self.goal.pose.position.y, self.goal.pose.position.z])
            # Rotations
            qb = (self.base.pose.orientation.x, self.base.pose.orientation.y,
                  self.base.pose.orientation.z, self.base.pose.orientation.w)
            qe = (self.ee.pose.orientation.x, self.ee.pose.orientation.y,
                  self.ee.pose.orientation.z, self.ee.pose.orientation.w)
            qg = (self.goal.pose.orientation.x, self.goal.pose.orientation.y,
                  self.goal.pose.orientation.z, self.goal.pose.orientation.w)
            # Compose world->base from world->marker (VRPN) and marker->base (static offset)
            R_wm = rotmat_from_quat(*qb)
            t_mb = np.array(self.base_marker_offset_xyz, dtype=float)
            # R_mb = rotmat_from_rpy(*self.base_marker_offset_rpy)
            R_mb = rotmat_from_rpy(*self.base_marker_offset_rpy)
            R_wb = R_wm @ R_mb
            pb = pb + R_wm @ t_mb
            R_bw = R_wb.T
            # Positions in base frame
            p_be = R_bw @ (pe - pb)
            p_bg = R_bw @ (pg - pb)
            ex, ey, ez = (p_bg - p_be).tolist()
            # Orientations in base frame via quaternion multiplication
            q_bw = quat_conj(qb)
            q_be = quat_mul(q_bw, qe)
            q_bg = quat_mul(q_bw, qg)
            q_err = quat_mul(q_bg, quat_conj(q_be))
            rx, ry, rz = quat_to_rotvec(*q_err)
        else:
            # Inputs are already in base frame
            ex = self.goal.pose.position.x - self.ee.pose.position.x
            ey = self.goal.pose.position.y - self.ee.pose.position.y
            ez = self.goal.pose.position.z - self.ee.pose.position.z
            qe = (self.ee.pose.orientation.x, self.ee.pose.orientation.y,
                  self.ee.pose.orientation.z, self.ee.pose.orientation.w)
            qg = (self.goal.pose.orientation.x, self.goal.pose.orientation.y,
                  self.goal.pose.orientation.z, self.goal.pose.orientation.w)
            q_err = quat_mul(qg, quat_conj(qe))
            rx, ry, rz = quat_to_rotvec(*q_err)

        # weight orientation to meter-equivalent
        rx *= self.k_ori_w
        ry *= self.k_ori_w
        rz *= self.k_ori_w
        e6 = [ex, ey, ez, rx, ry, rz]

        # Approximate e_dot with backward difference
        edot = [(e6[i] - self.last_e6[i]) / dt for i in range(6)]
        self.last_e6 = e6[:]

        # 2) Desired task-space twist (PD on error)
        vx =  self.kp_lin * e6[0] - self.kd_lin * edot[0]
        vy =  self.kp_lin * e6[1] - self.kd_lin * edot[1]
        vz =  self.kp_lin * e6[2] - self.kd_lin * edot[2]
        wx =  self.kp_ang * e6[3] - self.kd_ang * edot[3]
        wy =  self.kp_ang * e6[4] - self.kd_ang * edot[4]
        wz =  self.kp_ang * e6[5] - self.kd_ang * edot[5]

        # Clamp EE twist
        def clamp_vec3(x, y, z, lim):
            n = math.sqrt(x*x + y*y + z*z)
            if n > lim and n > 1e-9:
                s = lim / n
                return x*s, y*s, z*s
            return x, y, z
        vx, vy, vz = clamp_vec3(vx, vy, vz, self.ee_lin_lim)
        wx, wy, wz = clamp_vec3(wx, wy, wz, self.ee_ang_lim)

        # 3) Compute allocation alpha
        # base distance to EE goal position (XY) and yaw error to face goal
        dx = self.goal.pose.position.x - self.base.pose.position.x
        dy = self.goal.pose.position.y - self.base.pose.position.y
        d_base_xy = math.hypot(dx, dy)
        # Reach ratio r_arm = ||p_ee - p_base|| / max_reach
        dx_arm = self.ee.pose.position.x - self.base.pose.position.x
        dy_arm = self.ee.pose.position.y - self.base.pose.position.y
        dz_arm = self.ee.pose.position.z - self.base.pose.position.z
        r_arm = min(math.sqrt(dx_arm*dx_arm + dy_arm*dy_arm + dz_arm*dz_arm) / max(self.max_reach, 1e-6), 1.0)

        alpha_base = sigmoid(self.k_d * (d_base_xy - self.d_mid))  # rises as we get close
        alpha_reach = 0.5 + 0.5 * (1.0 - r_arm)                    # downweight when stretched
        alpha = clamp(alpha_base * alpha_reach, 0.0, 1.0)

        # 4) Retract hysteresis
        if d_base_xy < self.d_retract_enter:
            self.retract_mode = True
        elif d_base_xy > self.d_retract_exit:
            self.retract_mode = False

        # Publish null_kp if requested
        if self.pub_null_kp is not None:
            nkp = self.null_kp_retract if self.retract_mode else self.null_kp_normal
            self.pub_null_kp.publish(Float64(data=float(nkp)))

        # 5) Split commands
        # Arm gets (1-alpha) of the full 6D twist
        arm_vx, arm_vy, arm_vz = (1.0 - alpha) * vx, (1.0 - alpha) * vy, (1.0 - alpha) * vz
        arm_wx, arm_wy, arm_wz = (1.0 - alpha) * wx, (1.0 - alpha) * wy, (1.0 - alpha) * wz

        # Base gets alpha of the planar components derived from task twist
        # Start from the same desired planar components (vx, vy, wz)
        b_vx, b_vy, b_wz = alpha * vx, alpha * vy, alpha * wz

        # For non-holonomic base, convert (vx,vy) into forward v and heading correction
        if not self.base_is_holonomic:
            # Desired planar heading of the velocity
            speed = math.hypot(b_vx, b_vy)
            # Base yaw
            qb = (self.base.pose.orientation.x, self.base.pose.orientation.y,
                  self.base.pose.orientation.z, self.base.pose.orientation.w)
            yaw_b = yaw_from_quat(*qb)
            desired_heading = math.atan2(b_vy, b_vx) if speed > 1e-6 else yaw_b
            heading_err = (desired_heading - yaw_b + math.pi) % (2.0*math.pi) - math.pi
            v_forward = speed * math.cos(heading_err)
            w_heading = self.k_heading * heading_err
            b_vx, b_vy = v_forward, 0.0   # no lateral for differential drive
            b_wz = clamp(b_wz + w_heading, -self.base_ang_lim, self.base_ang_lim)

        # 6) LPF + slew + limit
        # Base
        b_vx = self.lpf_base_vx.step(b_vx)
        b_vy = self.lpf_base_vy.step(b_vy)
        b_wz = self.lpf_base_wz.step(b_wz)
        b_vx = self.slew_base_vx.step(b_vx, dt)
        b_vy = self.slew_base_vy.step(b_vy, dt)
        b_wz = self.slew_base_wz.step(b_wz, dt)
        b_lin = math.hypot(b_vx, b_vy)
        if b_lin > self.base_lin_lim and b_lin > 1e-9:
            s = self.base_lin_lim / b_lin
            b_vx *= s
            b_vy *= s
        b_wz = clamp(b_wz, -self.base_ang_lim, self.base_ang_lim)

        # Arm
        arm_vx = self.lpf_arm_vx.step(self.slew_arm_lin.step(arm_vx, dt))
        arm_vy = self.lpf_arm_vy.step(self.slew_arm_lin.step(arm_vy, dt))
        arm_vz = self.lpf_arm_vz.step(self.slew_arm_lin.step(arm_vz, dt))
        arm_wx = self.lpf_arm_wx.step(self.slew_arm_ang.step(arm_wx, dt))
        arm_wy = self.lpf_arm_wy.step(self.slew_arm_ang.step(arm_wy, dt))
        arm_wz = self.lpf_arm_wz.step(self.slew_arm_ang.step(arm_wz, dt))

        # 7) Publish
        # Base
        twb = Twist()
        twb.linear.x = float(b_vx)
        twb.linear.y = float(b_vy)
        twb.angular.z = float(b_wz)
        self.pub_cmd_vel.publish(twb)

        # Arm
        twa = TwistStamped()
        twa.header.stamp = now.to_msg()
        # Use EE/goal header frame_id to be explicit
        twa.header.frame_id = self.ee.header.frame_id if self.ee.header.frame_id else 'base'
        twa.twist.linear.x = float(arm_vx)
        twa.twist.linear.y = float(arm_vy)
        twa.twist.linear.z = float(arm_vz)
        twa.twist.angular.x = float(arm_wx)
        twa.twist.angular.y = float(arm_wy)
        twa.twist.angular.z = float(arm_wz)
        self.pub_ee_twist.publish(twa)

        # Occasional debug
        if int(now.nanoseconds * 1e-6) % 200 == 0:  # ~5 Hz-ish
            self.get_logger().info(
                f"d_base={d_base_xy:.3f} alpha={alpha:.2f} retract={int(self.retract_mode)} "
                f"b_v=({b_vx:.2f},{b_vy:.2f},{b_wz:.2f}) arm_v=({arm_vx:.2f},{arm_vy:.2f},{arm_vz:.2f})")


def main():
    rclpy.init()
    node = ArmBaseCoordinator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
