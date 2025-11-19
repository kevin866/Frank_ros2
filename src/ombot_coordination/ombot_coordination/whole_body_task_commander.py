#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
import numpy as np
def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def quat_to_rotmat(qx: float, qy: float, qz: float, qw: float):
    """Return 3x3 rotation matrix from quaternion (world -> this frame)."""
    # normalize
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-9:
        return [[1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]]
    qx /= n; qy /= n; qz /= n; qw /= n

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return [
        [1.0 - 2.0*(yy + zz), 2.0*(xy - wz),       2.0*(xz + wy)],
        [2.0*(xy + wz),       1.0 - 2.0*(xx + zz), 2.0*(yz - wx)],
        [2.0*(xz - wy),       2.0*(yz + wx),       1.0 - 2.0*(xx + yy)],
    ]


# drop this helper somewhere in your node file
class ThrottledLogger:
    def __init__(self, node):
        self._node = node
        self._last_ns = {}  # key -> last log time in ns

    def _allow(self, key: str, period_s: float) -> bool:
        now_ns = self._node.get_clock().now().nanoseconds
        last = self._last_ns.get(key, 0)
        if now_ns - last >= int(period_s * 1e9):
            self._last_ns[key] = now_ns
            return True
        return False

    def info(self, period_s: float, msg: str, key: str = None):
        k = key or "info"
        if self._allow(k, period_s):
            self._node.get_logger().info(msg)

    def warn(self, period_s: float, msg: str, key: str = None):
        k = key or "warn"
        if self._allow(k, period_s):
            self._node.get_logger().warn(msg)

    def error(self, period_s: float, msg: str, key: str = None):
        k = key or "error"
        if self._allow(k, period_s):
            self._node.get_logger().error(msg)

def rotmat_mul(Ra, Rb):
    """Ra * Rb for 3x3 matrices."""
    out = [[0.0]*3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = Ra[i][0]*Rb[0][j] + Ra[i][1]*Rb[1][j] + Ra[i][2]*Rb[2][j]
    return out


def rotmat_transpose(R):
    return [
        [R[0][0], R[1][0], R[2][0]],
        [R[0][1], R[1][1], R[2][1]],
        [R[0][2], R[1][2], R[2][2]],
    ]


def rotmat_to_rotvec(R):
    """
    Log map so(3): rotation matrix -> rotation vector (axis * angle).
    Returns 3D vector in the same frame as R.
    """
    # Trace-based angle
    tr = R[0][0] + R[1][1] + R[2][2]
    cos_theta = (tr - 1.0) * 0.5
    cos_theta = max(-1.0, min(1.0, cos_theta))
    theta = math.acos(cos_theta)

    if abs(theta) < 1e-6:
        return (0.0, 0.0, 0.0)

    # axis = (1/(2 sinθ)) * [R32 - R23, R13 - R31, R21 - R12]
    s = 2.0 * math.sin(theta)
    if abs(s) < 1e-6:
        return (0.0, 0.0, 0.0)

    rx = (R[2][1] - R[1][2]) / s
    ry = (R[0][2] - R[2][0]) / s
    rz = (R[1][0] - R[0][1]) / s

    return (rx * theta, ry * theta, rz * theta)


@dataclass
class Pose3D:
    p: list[float]   # [x, y, z]
    q: list[float]   # [x, y, z, w]


class WholeBodyTaskCommander(Node):
    """
    High-level node that:
      - Subscribes to base, EE, and goal poses (usually in world frame)
      - Computes 6D error in base frame
      - Outputs desired EE twist in base_link frame to the whole-body controller
    """

    def __init__(self):
        super().__init__("whole_body_task_commander")

        self.log = ThrottledLogger(self)


        # Parameters (tune as you like)
        self.declare_parameter("base_pose_topic", "/vrpn_mocap/RigidBody_1/pose")
        self.declare_parameter("ee_pose_topic", "/ee_pose")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        # Controller's private "~ee_twist" topic will expand to "<controller_name>/ee_twist"
        self.declare_parameter("ee_twist_topic", "/wb_resolved_rate_controller/ee_twist")

        # Simple PD gains in base frame
        self.declare_parameter("kp_pos", 1.0)
        self.declare_parameter("kp_rot", 1.0)
        self.declare_parameter("kd_pos", 0.0)
        self.declare_parameter("kd_rot", 0.0)

        # Velocity limits
        self.declare_parameter("max_lin", 0.5)    # m/s
        self.declare_parameter("max_ang", 1.0)    # rad/s

        base_pose_topic = self.get_parameter("base_pose_topic").get_parameter_value().string_value
        ee_pose_topic   = self.get_parameter("ee_pose_topic").get_parameter_value().string_value
        goal_pose_topic = self.get_parameter("goal_pose_topic").get_parameter_value().string_value
        ee_twist_topic  = self.get_parameter("ee_twist_topic").get_parameter_value().string_value

        self.kp_pos = self.get_parameter("kp_pos").value
        self.kp_rot = self.get_parameter("kp_rot").value
        self.kd_pos = self.get_parameter("kd_pos").value
        self.kd_rot = self.get_parameter("kd_rot").value
        self.max_lin = self.get_parameter("max_lin").value
        self.max_ang = self.get_parameter("max_ang").value

        self.base: Optional[Pose3D] = None
        self.ee:   Optional[Pose3D] = None
        self.goal: Optional[Pose3D] = None

        # Best-effort for mocap / ee pose
        sensor_qos = qos_profile_sensor_data  # depth=10, BEST_EFFORT, VOLATILE

        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscriptions
        self.sub_base = self.create_subscription(
            PoseStamped, base_pose_topic, self.base_cb, sensor_qos)

        self.sub_ee = self.create_subscription(
            PoseStamped, ee_pose_topic, self.ee_cb, sensor_qos)

        self.sub_goal = self.create_subscription(
            PoseStamped, goal_pose_topic, self.goal_cb, reliable_qos)
        
        # Publisher to the controller; reliable is fine here
        self.pub_twist = self.create_publisher(
            TwistStamped, ee_twist_topic, reliable_qos)

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.01, self.spin)  # 100 Hz

        self.get_logger().info(
            f"WholeBodyTaskCommander: base={base_pose_topic}, ee={ee_pose_topic}, "
            f"goal={goal_pose_topic}, twist_out={ee_twist_topic}"
        )

    # --- Callbacks for poses ---

    def base_cb(self, msg: PoseStamped):
        self.base = Pose3D(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w],
        )

    def ee_cb(self, msg: PoseStamped):
        self.ee = Pose3D(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w],
        )

    def goal_cb(self, msg: PoseStamped):
        self.goal = Pose3D(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w],
        )

    # --- Main control loop ---

    def spin(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-3
        dt = min(dt, 0.05)
        self.last_time = now

        if self.base is None or self.ee is None or self.goal is None:
            return
        
        # 1) Extract poses in BASE frame (rename to make this explicit)
        pe_b = self.ee.p     # base -> ee position
        qe_b = self.ee.q     # base -> ee orientation (quaternion)

        pg_b = self.goal.p   # base -> goal position
        qg_b = self.goal.q   # base -> goal orientation (quaternion)

        pb_w = np.array(self.base.p)   # base in world (you may not even need this)

        R_wb = np.array([
            [1.0,  0.0,  0.0],
            [0.0,  0.0, 1.0],
            [0.0,  1.0,  0.0],
        ])

        pb_b = R_wb.T @ pb_w   # base position in base frame (should be close to zero)
        self.log.info(1.0, f"pb_b = {pb_b}", key="pb_b")
        # 2) Rotation matrices in BASE frame
        R_be = quat_to_rotmat(*qe_b)   # base -> ee
        R_bg = quat_to_rotmat(*qg_b)   # base -> goal

        self.log.info(1.0, f"R_be (base->ee) = {R_be}", key="R_be")
        self.log.info(1.0, f"R_bg (base->goal) = {R_bg}", key="R_bg")

        # 3) Relative rotation ee -> goal, expressed in base
        # R_eb = R_be^T, then R_eg = R_eb * R_bg
        R_be_T = rotmat_transpose(R_be)      # ee -> base
        R_err  = rotmat_mul(R_be_T, R_bg)    # ee -> goal, expressed in base

        e_rot_b = rotmat_to_rotvec(R_err)    # rotational error (axis-angle) in base
        self.log.info(1.0, f"e_rot_b = {e_rot_b}", key="e_rot_b")

        # 4) Position error in base frame: e_pos_b = p_g^b - p_e^b
        e_pos_b = [
            pg_b[0] - pe_b[0] - pb_b[0],
            pg_b[1] - pe_b[1] - pb_b[1],
            pg_b[2] - pe_b[2] - pb_b[2],
        ]

        self.log.info(1.0, f"e_pos_b = {e_pos_b}", key="e_pos_b")




        # 4) Orientation error in base frame:
        #   R_be = R_bw * R_we
        #   R_bg = R_bw * R_wg
        #   R_err = R_be^T * R_bg
        # R_be = rotmat_mul(R_bw, R_we)
        # R_bg = rotmat_mul(R_bw, R_wg)
        # R_be_T = rotmat_transpose(R_be)
        # R_err = rotmat_mul(R_be_T, R_bg)   # rotation from ee frame to goal, expressed in base

        # e_rot_b = rotmat_to_rotvec(R_err)  # (rx, ry, rz) in base frame
        # self.log.info(1.0, f"e_rot_b = {e_rot_b}", key="e_rot_b")

        # 5) Simple PD in base frame
        # (here we ignore derivative of error for now; you can add it using stored last e_pos_b / e_rot_b)
        vx = self.kp_pos * e_pos_b[0]
        vy = self.kp_pos * e_pos_b[1]
        vz = self.kp_pos * e_pos_b[2]

        wx = self.kp_rot * e_rot_b[0]
        wy = self.kp_rot * e_rot_b[1]
        wz = self.kp_rot * e_rot_b[2]

        # 6) Clamp
        v_xy_mag = math.hypot(vx, vy)
        if v_xy_mag > self.max_lin:
            scale = self.max_lin / (v_xy_mag + 1e-9)
            vx *= scale
            vy *= scale

        vz = clamp(vz, -self.max_lin, self.max_lin)
        wx = clamp(wx, -self.max_ang, self.max_ang)
        wy = clamp(wy, -self.max_ang, self.max_ang)
        wz = clamp(wz, -self.max_ang, self.max_ang)

        # 7) Publish command in base_link frame for whole-body controller
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "base_link"   # should match base_link used in your C++ controller

        msg.twist.linear.x  = float(vx)
        msg.twist.linear.y  = float(vy)
        msg.twist.linear.z  = float(vz)
        msg.twist.angular.x = float(wx)
        msg.twist.angular.y = float(wy)
        msg.twist.angular.z = float(wz)

        # self.pub_twist.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WholeBodyTaskCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
