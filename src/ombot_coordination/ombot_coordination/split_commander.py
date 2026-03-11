#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Vector3Stamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
import numpy as np

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

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))
def smoothstep(x):
    return 3*x*x - 2*x*x*x  # 0..1, smooth slope at ends

class SoftBoxLimiter:
    def __init__(self):
        self.in_limit_x = False
        self.in_limit_y = False
        self.in_limit_z = False

    def _limit_axis(self, p, v, pmin, pmax, m_enter, m_exit, dt, in_limit_flag):
        # predict next position (this kills the "hits then backs off" tap)
        p_next = p + v * dt

        # distance to bounds (use next position for decision)
        dist_lo = p_next - pmin
        dist_hi = pmax - p_next

        # hysteresis state update
        if not in_limit_flag:
            if dist_lo < m_enter or dist_hi < m_enter:
                in_limit_flag = True
        else:
            if dist_lo > m_exit and dist_hi > m_exit:
                in_limit_flag = False

        # only apply scaling when we're in limiting mode
        if in_limit_flag:
            # scale only the inward direction
            if v < 0.0 and dist_lo < m_exit:
                x = max(0.0, min(1.0, dist_lo / m_exit))
                v *= smoothstep(x)
            if v > 0.0 and dist_hi < m_exit:
                x = max(0.0, min(1.0, dist_hi / m_exit))
                v *= smoothstep(x)

        return v, in_limit_flag

def apply_bound_with_latch(p_next: float,
                           v_cmd: float,
                           pmin: float,
                           pmax: float,
                           latched: bool,
                           eps: float = 0.03):
    """
    If you hit a bound while moving outward, latch and force v=0.
    Stay latched (v=0) until motion is inward or you're back inside.
    Returns: (v_limited, latched)
    """
    # Hit lower bound moving outward (negative direction)
    if p_next <= pmin + eps and v_cmd < 0.0:
        return 0.0, True

    # Hit upper bound moving outward (positive direction)
    if p_next >= pmax - eps and v_cmd > 0.0:
        return 0.0, True

    if latched:
        # Release latch if:
        # 1) we're safely inside, OR
        # 2) at lower bound but commanding inward, OR
        # 3) at upper bound but commanding inward
        # if ((pmin + eps) < p_next < (pmax - eps)) or \
        #    (p_next <= pmin + eps and v_cmd > 0.0) or \
        #    (p_next >= pmax - eps and v_cmd < 0.0) or False:
        #     latched = False
        # else:
        return 0.0, True

    return v_cmd, latched


def quat_to_rotmat(qx: float, qy: float, qz: float, qw: float):
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


def vel_deadband(v, db=0.01):
    return 0.0 if abs(v) < db else v

def smooth_limit(p, v, pmin, pmax, margin):
    
    # lower bound
    if v < 0.0:
        dist = p - pmin
        if dist < margin:
            scale = max(0.0, dist / margin)
            v *= scale

    # upper bound
    if v > 0.0:
        dist = pmax - p
        if dist < margin:
            scale = max(0.0, dist / margin)
            v *= scale

    return v

def rotmat_mul(Ra, Rb):
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
    tr = R[0][0] + R[1][1] + R[2][2]
    cos_theta = (tr - 1.0) * 0.5
    cos_theta = max(-1.0, min(1.0, cos_theta))
    theta = math.acos(cos_theta)

    if abs(theta) < 1e-6:
        return (0.0, 0.0, 0.0)

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


class SplitCommander(Node):
    """
    Option B:
      Arm twist = K1*(goal-ee) + K2*(stow-ee)  (implemented via effective goal)
      Base cmd  = K3*(goal - stow) in XY
    """

    def __init__(self):
        super().__init__("split_commander")

        # Topics
        self.declare_parameter("base_pose_topic", "/vrpn_mocap/RigidBody_1/pose")
        self.declare_parameter("arm_pose_topic", "/vrpn_mocap/RigidBody_2/pose")

        self.declare_parameter("ee_pose_topic", "/ee_pose")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("ee_twist_topic", "/resolved_rate_controller/ee_twist")
        self.declare_parameter("base_cmd_topic", "/cmd_vel")  # change to your base controller cmd

        # Option B gains
        self.declare_parameter("k1", 1.0)
        self.declare_parameter("k2", 0.2)
        self.declare_parameter("k3", 0.8)
        self.declare_parameter("k1d", 0.0)
        self.declare_parameter("k2d", 0.0)
        self.declare_parameter("k3d", 0.0)


        # Stow point (in base frame)
        self.declare_parameter("stow_point_b", [0.25, 0.0, 0.4])  # <-- set this

        # Existing PD gains for twist tracking (keep simple)
        self.declare_parameter("kp_pos", 1.0)
        self.declare_parameter("kp_rot", 0.0)   # start w/ 0 for rot for simplicity
        self.declare_parameter("kd_pos", 0.0)
        self.declare_parameter("kd_rot", 0.0)

        self.e1_pub = self.create_publisher(Vector3Stamped, "/debug/e1", 10)
        self.e2_pub = self.create_publisher(Vector3Stamped, "/debug/e2", 10)
        self.e3_pub = self.create_publisher(Vector3Stamped, "/debug/e3", 10)

        # Limits
        self.declare_parameter("max_lin", 0.5)
        self.declare_parameter("max_ang", 1.0)
        self.declare_parameter("max_base_lin", 0.4)
        self.declare_parameter("max_base_ang", 1.0)

        self.log = ThrottledLogger(self)


        base_pose_topic = self.get_parameter("base_pose_topic").value
        arm_pose_topic  = self.get_parameter("arm_pose_topic").value
        ee_pose_topic   = self.get_parameter("ee_pose_topic").value
        goal_pose_topic = self.get_parameter("goal_pose_topic").value
        ee_twist_topic  = self.get_parameter("ee_twist_topic").value
        base_cmd_topic  = self.get_parameter("base_cmd_topic").value

        self.k1 = float(self.get_parameter("k1").value)
        self.k2 = float(self.get_parameter("k2").value)
        self.k3 = float(self.get_parameter("k3").value)
        self.k1d = float(self.get_parameter("k1d").value)
        self.k2d = float(self.get_parameter("k2d").value)
        self.k3d = float(self.get_parameter("k3d").value)

        self.stow_b = list(self.get_parameter("stow_point_b").value)

        self.kp_pos = float(self.get_parameter("kp_pos").value)
        self.kp_rot = float(self.get_parameter("kp_rot").value)
        self.kd_pos = float(self.get_parameter("kd_pos").value)
        self.kd_rot = float(self.get_parameter("kd_rot").value)

        self.max_lin = float(self.get_parameter("max_lin").value)
        self.max_ang = float(self.get_parameter("max_ang").value)
        self.max_base_lin = float(self.get_parameter("max_base_lin").value)
        self.max_base_ang = float(self.get_parameter("max_base_ang").value)

        self.ws_min = np.array([-0.2, -0.5, 0.5], dtype=float)
        self.ws_max = np.array([ 0.6,  0.5, 0.8], dtype=float)
        self.ws_margin = 0.02   # 2 cm “soft” zone near the walls
        
        # persistent state somewhere
        self.x_latched = False
        self.y_latched = False
        self.z_latched = False


        self.base: Optional[Pose3D] = None
        self.ee:   Optional[Pose3D] = None
        self.goal: Optional[Pose3D] = None

        self.deadband = 0.05

        sensor_qos = qos_profile_sensor_data
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub_base = self.create_subscription(PoseStamped, base_pose_topic, self.base_cb, sensor_qos)
        self.sub_arm  = self.create_subscription(PoseStamped, arm_pose_topic,  self.arm_cb,  sensor_qos)
        self.sub_ee   = self.create_subscription(PoseStamped, ee_pose_topic,   self.ee_cb,   sensor_qos)
        self.sub_goal = self.create_subscription(PoseStamped, goal_pose_topic, self.goal_cb, reliable_qos)

        self.pub_twist = self.create_publisher(TwistStamped, ee_twist_topic, reliable_qos)
        self.pub_base  = self.create_publisher(TwistStamped, base_cmd_topic, reliable_qos)

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.01, self.spin)

        self.last_e_pos = [0.0, 0.0, 0.0]
        self.last_e_rot = [0.0, 0.0, 0.0]
        self.have_last_error = False

        self.have_last_error = False
        self.last_e1 = [0.0, 0.0, 0.0]
        self.last_e2 = [0.0, 0.0, 0.0]
        self.limiter = SoftBoxLimiter()



    def publish_vector(self, pub, vec, frame="base_link"):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.vector.x = float(vec[0])
        msg.vector.y = float(vec[1])
        msg.vector.z = float(vec[2])
        pub.publish(msg)

    def project_velocity_sphere(p, v, c, R, margin=0.02):
        d = p - c
        dist = np.linalg.norm(d)
        if dist < 1e-9:
            return v
        n = d / dist  # outward normal

        # only constrain when near/outside boundary
        if dist >= (R - margin):
            v_out = np.dot(v, n)
            if v_out > 0.0:          # trying to go outward
                v = v - v_out * n    # remove outward component
        return v
    
    def project_velocity_box(p, v, pmin, pmax, margin=0.02):
        v2 = v.copy()
        for i in range(3):
            if p[i] <= pmin[i] + margin and v2[i] < 0: v2[i] = 0.0
            if p[i] >= pmax[i] - margin and v2[i] > 0: v2[i] = 0.0
        return v2

    def arm_cb(self, msg: PoseStamped):
        self.arm =  Pose3D(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w],
        )

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



    def spin(self):
        now = self.get_clock().now()
        dt_raw = (now - self.last_time).nanoseconds * 1e-9
        if dt_raw <= 0.0:
            dt_raw = 1e-3
        dt = min(dt_raw, 0.05)
        self.last_time = now

        if self.base is None or self.ee is None or self.goal is None:
            return

        pb_w = np.array(self.base.p)   # base in world (you may not even need this)
        pa_w = np.array(self.arm.p)    # arm in world (you may not even need this)

        R_wb = np.array([
            [1.0,  0.0,  0.0],
            [0.0,  0.0, 1.0],
            [0.0,  -1.0,  0.0],
        ])
        t = np.array([0, -0.33, 0])   # translation applied after rotation

        pb_b = R_wb.T @ pb_w  
        pa_b = R_wb.T @ pa_w 
        pe_b = self.ee.p

        # t_base_to_arm = np.array([0.0, 0.0, 0.358])   # base_link -> arm_base_link, expressed in base_link

        # pe_b = self.ee.p + t_base_to_arm
        # Assume ee_pose and goal_pose are already expressed in base frame
        qe_b = self.ee.q
        pg_b = self.goal.p
        qg_b = self.goal.q

        # self.log.info(
        #     1.0,  # log once per second
        #     (
        #         f"Poses (base frame) | "
        #         f"EE p={pe_b} | "
        #         f"Goal p={pg_b}, q={qg_b}"
        #         f"arm p={pa_b} | "
        #     ),
        #     key="poses_all"
        # )

        # --- Explicit errors ---
        # e1: goal - ee

        # e1 = [pg_b[0] - pe_b[0],
        #     pg_b[1] - pe_b[1],
        #     pg_b[2] - pe_b[2]]

        e1 = [
            pg_b[0] - pe_b[0] - pb_b[0],
            # pg_b[1] - pe_b[1] - pb_b[1],
            0.0,
            0.0,
        ]
        e1 = [pg_b[0] - pa_b[0],
              pg_b[1] - pa_b[1],
              pg_b[2] - pa_b[2]]

        # e2: stow - ee
        e2 = [self.stow_b[0] - pe_b[0],
            self.stow_b[1] - pe_b[1],
            self.stow_b[2] - pe_b[2]]

        # e3: goal - stow (for base)
        e3 = [pg_b[0] - self.stow_b[0] - pb_b[0],
            pg_b[1] - self.stow_b[1] - pb_b[1],
            pg_b[2] - self.stow_b[2] - pb_b[2]]

        if self.have_last_error:
            de1 = [(e1[i] - self.last_e1[i]) / dt for i in range(3)]
            de2 = [(e2[i] - self.last_e2[i]) / dt for i in range(3)]
        else:
            de1 = [0.0, 0.0, 0.0]
            de2 = [0.0, 0.0, 0.0]
            self.have_last_error = True

        self.last_e1 = e1.copy()
        self.last_e2 = e2.copy()



        # --- Arm "force" in task space (Option B literal) ---
        # Think of this as the *position-like error* you want to drive down
        # e_sum = [e1[0] + e2[0],
        #         e1[1] + e2[1],
        #         e1[2] + e2[2]]
        # e_sum = [self.k1 * e1[0] + self.k2 * e2[0] + self.k1d * de1[0] + self.k2d * de2[0],
        #     self.k1 * e1[1] + self.k2 * e2[1] + self.k1d * de1[1] + self.k2d * de2[1],
        #     self.k1 * e1[2] + self.k2 * e2[2] + self.k1d * de1[2] + self.k2d * de2[2]]
        e_sum = [self.k1 * e1[0] + self.k1d * de1[0],
            self.k1 * e1[1] + self.k1d * de1[1] ,
            self.k1 * e1[2] + self.k1d * de1[2]]
        # self.get_logger().info(
        #     f"e_sum: [{e_sum[0]:.4f}, {e_sum[1]:.4f}, {e_sum[2]:.4f}]"
        # )
        # self.get_logger().info(
        #     f"e1: [{e1[0]:.4f}, {e1[1]:.4f}, {e1[2]:.4f}]  "
        #     f"de1: [{de1[0]:.4f}, {de1[1]:.4f}, {de1[2]:.4f}]"
        # )

        # Deadband on the combined error
        e_sum_db = [smooth_deadband(e_sum[i], self.deadband) for i in range(3)]

 
        # --- Orientation error (optional) ---
        R_be = quat_to_rotmat(*qe_b)
        R_bg = quat_to_rotmat(*qg_b)
        R_err = rotmat_mul(rotmat_transpose(R_be), R_bg)
        e_rot_b = rotmat_to_rotvec(R_err)

        # --- Derivatives (use e_sum, not old e_pos_b) ---
        # if self.have_last_error:
        #     de_pos = [(e_sum[i] - self.last_e_pos[i]) / dt for i in range(3)]
        #     de_rot = [(e_rot_b[i] - self.last_e_rot[i]) / dt for i in range(3)]
        # else:
        #     de_pos = [0.0, 0.0, 0.0]
        #     de_rot = [0.0, 0.0, 0.0]
        #     self.have_last_error = True

        # self.last_e_pos = e_sum.copy()
        # self.last_e_rot = list(e_rot_b)

        # --- Commanded EE twist (base frame) ---
        # If you want "pure Option B", you can set kd_pos=0 and treat kp_pos as a scale.
        vx = self.kp_pos * e_sum_db[0] 
        vy = self.kp_pos * e_sum_db[1] 
        vz = self.kp_pos * e_sum_db[2] 

        wx = self.kp_rot * e_rot_b[0] 
        wy = self.kp_rot * e_rot_b[1] 
        wz = self.kp_rot * e_rot_b[2] 

        # Clamp arm twist
        v_xy_mag = math.hypot(vx, vy)
        if v_xy_mag > self.max_lin:
            s = self.max_lin / (v_xy_mag + 1e-9)
            vx *= s
            vy *= s

        vz = clamp(vz, -self.max_lin, self.max_lin)
        wx = clamp(wx, -self.max_ang, self.max_ang)
        wy = clamp(wy, -self.max_ang, self.max_ang)
        wz = clamp(wz, -self.max_ang, self.max_ang)

        vx = vel_deadband(vx)
        vy = vel_deadband(vy)
        vz = vel_deadband(vz)

        # --- Workspace box limits (base frame) ---
        pmin = np.array([-0.1, -0.3, 0.1], dtype=float)
        pmax = np.array([ 0.5,  0.3, 0.5], dtype=float)
        m = 0.02  # 2cm margin

        # current EE position in base frame (you must already have this)
        px, py, pz = pe_b[0], pe_b[1], pe_b[2]
        # px_next = px + vx * dt
        # py_next = py + vy * dt
        # pz_next = pz + vz * dt
        # project velocity so it cannot push outward at/near limits
        if px <= pmin[0] + m and vx < 0.0: vx = 0.0
        if px >= pmax[0] - m and vx > 0.0: vx = 0.0

        if py <= pmin[1] + m and vy < 0.0: vy = 0.0
        if py >= pmax[1] - m and vy > 0.0: vy = 0.0

        if pz <= pmin[2] + m and vz < 0.0: vz = 0.0
        if pz >= pmax[2] - m and vz > 0.0: vz = 0.0

        # vx = smooth_limit(px_next, vx, pmin[0], pmax[0], m)
        # vy = smooth_limit(py_next, vy, pmin[1], pmax[1], m)
        # vz = smooth_limit(pz_next, vz, pmin[2], pmax[2], m)
        
      

        # in your control loop:
        # vx = smooth_limit(px_next, vx, pmin[0], pmax[0], m)
        # vy = smooth_limit(py_next, vy, pmin[1], pmax[1], m)
        # vz = smooth_limit(pz_next, vz, pmin[2], pmax[2], m)

        # vx, x_latched = apply_bound_with_latch(px_next, vx, pmin[0], pmax[0], self.x_latched)
        # vy, y_latched = apply_bound_with_latch(py_next, vy, pmin[1], pmax[1], self.y_latched)
        # vz, z_latched = apply_bound_with_latch(pz_next, vz, pmin[2], pmax[2], self.z_latched)
        # self.x_latched = x_latched
        # self.y_latched = y_latched
        # self.z_latched = z_latched

        # self.get_logger().info(f"x_latched: {x_latched}")
        # self.get_logger().info(f"px_next: {px_next}")


       

        # choose hysteresis band
        # m_enter = m          # start limiting
        # m_exit  = 1.3 * m    # stop limiting (a bit larger)

        # vx, self.limiter.in_limit_x = self.limiter._limit_axis(px, vx, pmin[0], pmax[0], m_enter, m_exit, dt, self.limiter.in_limit_x)
        # vy, self.limiter.in_limit_y = self.limiter._limit_axis(py, vy, pmin[1], pmax[1], m_enter, m_exit, dt, self.limiter.in_limit_y)
        # vz, self.limiter.in_limit_z = self.limiter._limit_axis(pz, vz, pmin[2], pmax[2], m_enter, m_exit, dt, self.limiter.in_limit_z)


        # Publish arm twist
        tmsg = TwistStamped()
        tmsg.header.stamp = now.to_msg()
        tmsg.header.frame_id = "base_link"
        tmsg.twist.linear.x = float(vx)
        tmsg.twist.linear.y = float(vy)
        tmsg.twist.linear.z = float(vz)
        tmsg.twist.angular.x = float(wx)
        tmsg.twist.angular.y = float(wy)
        tmsg.twist.angular.z = float(wz)
        self.pub_twist.publish(tmsg)
        # self.get_logger().info(
        #     f"cmd linear: vx={tmsg.twist.linear.x:.3f}, "
        #     f"vy={tmsg.twist.linear.y:.3f}, "
        #     f"vz={tmsg.twist.linear.z:.3f}"
        # )

        # --- Base command from F3: move stow toward goal (XY only) ---
        vbx = -self.k3 * e2[0]
        vby = self.k3 * e2[1]

        vmag = math.hypot(vbx, vby)
        if vmag > self.max_base_lin:
            s = self.max_base_lin / (vmag + 1e-9)
            vbx *= s
            vby *= s

        bmsg = TwistStamped()
        bmsg.header.stamp = now.to_msg()
        bmsg.header.frame_id = "base_link"
        bmsg.twist.linear.x = -float(vel_deadband(vbx, 0.02))
        bmsg.twist.linear.y = -float(vel_deadband(vby, 0.02))
        bmsg.twist.linear.z = 0.0
        bmsg.twist.angular.x = 0.0
        bmsg.twist.angular.y = 0.0
        bmsg.twist.angular.z = 0.0
        self.pub_base.publish(bmsg)

        # --- Logging ---
        self.log.info(
            1.0,
            (
                f"cmd arm lin=[{vx:+.2f},{vy:+.2f},{vz:+.2f}] "
                f"ang=[{wx:+.2f},{wy:+.2f},{wz:+.2f}] | "
                f"base lin=[{bmsg.twist.linear.x:+.2f},{bmsg.twist.linear.y:+.2f}]"
            ),
            key="cmds"
        )

        # self.log.info(
        #     1.0,
        #     (
        #         "errors (base frame)\n"
        #         f"  e1 (goal-ee):  "
        #         f"[x={e1[0]:+.3f}, y={e1[1]:+.3f}, z={e1[2]:+.3f}]\n"
        #         f"  e2 (stow-ee):  "
        #         f"[x={e2[0]:+.3f}, y={e2[1]:+.3f}, z={e2[2]:+.3f}]\n"
        #         f"  e3 (goal-stow):"
        #         f"[x={e3[0]:+.3f}, y={e3[1]:+.3f}]"
        #     ),
        #     key="errs"
        # )

        self.publish_vector(self.e1_pub, e1)
        self.publish_vector(self.e2_pub, e2)
        self.publish_vector(self.e3_pub, e3)


def main(args=None):
    rclpy.init(args=args)
    node = SplitCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()