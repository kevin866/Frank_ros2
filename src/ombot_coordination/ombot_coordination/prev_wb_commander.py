#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional
from tf2_ros import Buffer, TransformListener

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
def vel_deadband(v, db=0.001):
    return 0.0 if abs(v) < db else v

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

def smooth_deadband(x, db):
    if abs(x) < db:
        return 0.0
    return math.copysign(abs(x) - db, x)

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
        self.declare_parameter("ee_pose_topic", "/ee_pose")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        # Controller's private "~ee_twist" topic will expand to "<controller_name>/ee_twist"
        self.declare_parameter("ee_twist_topic", "/wb_resolved_rate_controller/ee_twist")

        # Simple PD gains in base frame
        self.declare_parameter("kp_pos", 1.0)
        self.declare_parameter("kp_rot", 1.0)
        self.declare_parameter("kd_pos", 0.2)
        self.declare_parameter("kd_rot", 0.0)

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame  = self.get_parameter("base_frame").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # Velocity limits
        self.declare_parameter("max_lin", 3.5)    # m/s
        self.declare_parameter("max_ang", 1.0)    # rad/s

        ee_pose_topic   = self.get_parameter("ee_pose_topic").get_parameter_value().string_value
        goal_pose_topic = self.get_parameter("goal_pose_topic").get_parameter_value().string_value
        ee_twist_topic  = self.get_parameter("ee_twist_topic").get_parameter_value().string_value

        self.kp_pos = self.get_parameter("kp_pos").value
        self.kp_rot = self.get_parameter("kp_rot").value
        self.kd_pos = self.get_parameter("kd_pos").value
        self.kd_rot = self.get_parameter("kd_rot").value
        self.max_lin = self.get_parameter("max_lin").value
        self.max_ang = self.get_parameter("max_ang").value

        self.ee:   Optional[Pose3D] = None
        self.goal: Optional[Pose3D] = None

        self.deadband = 0.001

        self.declare_parameter("use_traj", True)
        self.use_traj = bool(self.get_parameter("use_traj").value)

        self.des_pose = None
        self.des_twist = None



        # Best-effort for mocap / ee pose
        sensor_qos = qos_profile_sensor_data  # depth=10, BEST_EFFORT, VOLATILE

        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscriptions

        self.sub_des_pose = self.create_subscription(
            PoseStamped, "/ee_desired_pose", self.des_pose_cb, 10)

        self.sub_des_twist = self.create_subscription(
            TwistStamped, "/ee_desired_twist", self.des_twist_cb, 10)

   
        self.sub_ee = self.create_subscription(
            PoseStamped, ee_pose_topic, self.ee_cb, sensor_qos)

        self.sub_goal = self.create_subscription(
            PoseStamped, goal_pose_topic, self.goal_cb, reliable_qos)
        
        # Publisher to the controller; reliable is fine here
        self.pub_twist = self.create_publisher(
            TwistStamped, ee_twist_topic, reliable_qos)

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.01, self.spin)  # 100 Hz

        # self.get_logger().info(
        #     f"WholeBodyTaskCommander: base={base_pose_topic}, ee={ee_pose_topic}, "
        #     f"goal={goal_pose_topic}, twist_out={ee_twist_topic}"
        # )

        self.last_e_pos = [0.0, 0.0, 0.0]
        self.last_e_rot = [0.0, 0.0, 0.0]
        self.have_last_error = False


    # --- Callbacks for poses ---
    def des_pose_cb(self, msg): self.des_pose = msg

    def des_twist_cb(self, msg): self.des_twist = msg

    def lookup_world_T_base(self):
        tf = self.tf_buffer.lookup_transform(
            self.world_frame,   # target
            self.base_frame,    # source
            rclpy.time.Time(),  # latest
        )
        t = tf.transform.translation
        q = tf.transform.rotation

        p_b_w = np.array([t.x, t.y, t.z], dtype=float)  # base in world
        R_wb  = np.array(quat_to_rotmat(q.x, q.y, q.z, q.w), dtype=float)  # base orientation in world
        return R_wb, p_b_w


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
        # --- 1) Measure loop time once ---
        now = self.get_clock().now()
        dt_raw = (now - self.last_time).nanoseconds * 1e-9
        if dt_raw <= 0.0:
            dt_raw = 1e-3
        dt_used = min(dt_raw, 0.05)   # only clamp huge spikes if you want
        self.last_time = now

        # Optional: log the commander loop rate
        # self.get_logger().info(
        #     f"Commander loop: dt_raw={dt_raw:.6f} s, dt_used={dt_used:.6f} s"
        # )

        if self.ee is None or self.goal is None:
            return

        # EE in base
        pe_b = self.ee.p
        qe_b = self.ee.q
        R_be = quat_to_rotmat(*qe_b)

        # TF: world -> base
        try:
            R_wb, p_b_w = self.lookup_world_T_base()
        except Exception as e:
            self.log.warn(1.0, f"TF lookup {self.world_frame}->{self.base_frame} failed: {e}", key="tf_fail")
            return
        R_bw = R_wb.T

        # Goal in world
        pg_w = np.array(self.goal.p, dtype=float)
        qg_w = self.goal.q
        R_wg = np.array(quat_to_rotmat(*qg_w), dtype=float)

        # Default: goal from /goal_pose (world) transformed into base
        pg_b_np = R_bw @ (pg_w - p_b_w)
        pg_b = [float(pg_b_np[0]), float(pg_b_np[1]), float(pg_b_np[2])]
        R_bg = (R_bw @ R_wg).tolist()
        vff = [0.0, 0.0, 0.0]

        # If using trajectory refs in base frame, override consistently
        if self.use_traj and self.des_pose is not None:
            if self.des_pose.header.frame_id == self.base_frame:
                pg_b = [self.des_pose.pose.position.x,
                        self.des_pose.pose.position.y,
                        self.des_pose.pose.position.z]
                vff = [self.des_twist.twist.linear.x,
                    self.des_twist.twist.linear.y,
                    self.des_twist.twist.linear.z] if self.des_twist else [0.0, 0.0, 0.0]
                qg_b = [self.des_pose.pose.orientation.x,
                        self.des_pose.pose.orientation.y,
                        self.des_pose.pose.orientation.z,
                        self.des_pose.pose.orientation.w]
                R_bg = quat_to_rotmat(*qg_b)  # base -> desired
            else:
                self.log.warn(1.0, f"des_pose in '{self.des_pose.header.frame_id}', expected '{self.base_frame}'", key="des_pose_frame")



        R_be_T = rotmat_transpose(R_be)   # ee -> base
        R_err  = rotmat_mul(R_be_T, R_bg) # ee -> goal (expressed in base)
        e_rot_b = rotmat_to_rotvec(R_err)

        e_pos_b = [
            pg_b[0] - pe_b[0],
            pg_b[1] - pe_b[1],
            pg_b[2] - pe_b[2],
        ]
        # e_pos_b = np.array([e_pos_b[0], 0.0, 0.0])  # only X

        self.log.info(1.0, f"e_pos_b = {e_pos_b}", key="e_pos_b")

        # e_pos_b and e_rot_b already computed

        if self.have_last_error:
            de_pos = [
                (e_pos_b[0] - self.last_e_pos[0]) / dt_used,
                (e_pos_b[1] - self.last_e_pos[1]) / dt_used,
                (e_pos_b[2] - self.last_e_pos[2]) / dt_used,
            ]
            de_rot = [
                (e_rot_b[0] - self.last_e_rot[0]) / dt_used,
                (e_rot_b[1] - self.last_e_rot[1]) / dt_used,
                (e_rot_b[2] - self.last_e_rot[2]) / dt_used,
            ]
        else:
            de_pos = [0.0, 0.0, 0.0]
            de_rot = [0.0, 0.0, 0.0]
            self.have_last_error = True

        self.last_e_pos = e_pos_b.copy()
        self.last_e_rot = list(e_rot_b)

        vx = vff[0] + self.kp_pos * e_pos_b[0]
        vy = vff[1] + self.kp_pos * e_pos_b[1]
        vz = vff[2] + self.kp_pos * e_pos_b[2]


        wx = self.kp_rot * e_rot_b[0] + self.kd_rot * de_rot[0]
        wy = self.kp_rot * e_rot_b[1] + self.kd_rot * de_rot[1]
        wz = self.kp_rot * e_rot_b[2] + self.kd_rot * de_rot[2]


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

        self.pub_twist.publish(msg)
        # self.log.info(
        #     1.0,
        #     f"twist = lin({vx:.3f}, {vy:.3f}, {vz:.3f}), "
        #     f"ang({wx:.3f}, {wy:.3f}, {wz:.3f})",
        #     key="twist_cmd"
        # )



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
