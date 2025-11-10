#!/usr/bin/env python3
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Tuple
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np

def clamp(x: float, lo: float, hi: float) -> float: return max(lo, min(hi, x))
def sigmoid(x: float) -> float: return 1.0 / (1.0 + math.exp(-x))

def quat_to_rotvec(qx, qy, qz, qw) -> Tuple[float,float,float]:
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n == 0.0: return (0.0,0.0,0.0)
    qx,qy,qz,qw = qx/n,qy/n,qz/n,qw/n
    ang = 2.0*math.acos(clamp(qw,-1.0,1.0))
    s = math.sqrt(max(1.0-qw*qw,0.0))
    if s < 1e-8: return (qx*ang, qy*ang, qz*ang)
    ax,ay,az = qx/s,qy/s,qz/s
    return (ax*ang, ay*ang, az*ang)

def quat_conj(q): x,y,z,w=q; return (-x,-y,-z,w)
def quat_mul(a,b):
    ax,ay,az,aw=a; bx,by,bz,bw=b
    return (aw*bx+ax*bw+ay*bz-az*by,
            aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw,
            aw*bw-ax*bx-ay*by-az*bz)

def yaw_from_quat(qx,qy,qz,qw):
    siny = 2.0*(qw*qz + qx*qy)
    cosy = 1.0-2.0*(qy*qy+qz*qz)
    return math.atan2(siny, cosy)

def rotmat_from_quat(qx,qy,qz,qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n == 0.0: return np.eye(3)
    x,y,z,w = qx/n,qy/n,qz/n,qw/n
    xx,yy,zz = x*x,y*y,z*z
    xy,xz,yz = x*y,x*z,y*z
    wx,wy,wz = w*x,w*y,w*z
    return np.array([[1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)],
                     [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)],
                     [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)]])

def rotmat_from_rpy(roll, pitch, yaw):
    cr,sr = math.cos(roll), math.sin(roll)
    cp,sp = math.cos(pitch), math.sin(pitch)
    cy,sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    return Rz @ Ry @ Rx

def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr, sr = math.cos(roll/2.0), math.sin(roll/2.0)
    cp, sp = math.cos(pitch/2.0), math.sin(pitch/2.0)
    cy, sy = math.cos(yaw/2.0), math.sin(yaw/2.0)
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    qw = cr*cp*cy + sr*sp*sy
    return (qx, qy, qz, qw)


@dataclass
class LPF:
    alpha: float
    y: float = 0.0
    def step(self, x: float) -> float:
        self.y = self.alpha * x + (1.0 - self.alpha) * self.y
        return self.y

@dataclass
class SlewLimiter:
    max_rate: float
    last: float = 0.0
    def step(self, x: float, dt: float) -> float:
        dl = x - self.last
        md = self.max_rate * dt
        if dl > md: x = self.last + md
        elif dl < -md: x = self.last - md
        self.last = x
        return x

class ArmBaseCoordinator(Node):
    def __init__(self):
        super().__init__('arm_base_coordinator')

        # --- Parameters ---
        self.ee_pose_topic  = self.declare_parameter('ee_pose_topic',  '/ee_pose').get_parameter_value().string_value
        self.goal_pose_topic= self.declare_parameter('goal_pose_topic','/goal_pose').get_parameter_value().string_value
        self.base_pose_topic= self.declare_parameter('base_pose_topic','/base_pose').get_parameter_value().string_value

        # NEW: use internal XYZ offset goal?
        self.use_offset_goal = self.declare_parameter('use_offset_goal', True).get_parameter_value().bool_value
        self.offset_frame    = self.declare_parameter('offset_frame', 'base').get_parameter_value().string_value  # 'base' or 'world'
        self.offset_xyz      = list(self.declare_parameter('offset_xyz', [0.5, 0.0, 0.0]).get_parameter_value().double_array_value)  # meters

        self.inputs_in_world = self.declare_parameter('inputs_in_world', True).get_parameter_value().bool_value
        self.ignore_marker_orientation = self.declare_parameter(
            'ignore_marker_orientation', False).get_parameter_value().bool_value
        self.fixed_marker_to_base_rpy = list(
            self.declare_parameter('fixed_marker_to_base_rpy',
                                [0.0, 0.0, 0.0]).get_parameter_value().double_array_value)

        self.ignore_pose_orientations = self.declare_parameter(
            'ignore_pose_orientations', False).get_parameter_value().bool_value

        self.base_marker_offset_xyz = list(self.declare_parameter('base_marker_offset_xyz', [0.0,0.0,0.0]).get_parameter_value().double_array_value)
        self.base_marker_offset_rpy = list(self.declare_parameter('base_marker_offset_rpy', [0.0,0.0,0.0]).get_parameter_value().double_array_value)
        self.marker_yaw_flip = self.declare_parameter(
            'marker_yaw_flip', True).get_parameter_value().bool_value  # apply +π yaw if mocap marker faces backward
        self.ee_twist_topic  = self.declare_parameter('ee_twist_topic','/resolved_rate_controller/ee_twist').get_parameter_value().string_value
        # self.cmd_vel_topic   = self.declare_parameter('cmd_vel_topic', '/cmd_vel').get_parameter_value().string_value
        self.cmd_vel_topic   = self.declare_parameter('cmd_vel_topic', '/mecanum_controller/reference').get_parameter_value().string_value

        self.kp_lin = self.declare_parameter('kp_lin', 1.2).get_parameter_value().double_value
        self.kd_lin = self.declare_parameter('kd_lin', 0.3).get_parameter_value().double_value
        self.kp_ang = self.declare_parameter('kp_ang', 1.0).get_parameter_value().double_value
        self.kd_ang = self.declare_parameter('kd_ang', 0.25).get_parameter_value().double_value
        self.k_ori_w = self.declare_parameter('k_ori_weight', 0.5).get_parameter_value().double_value

        self.k_d   = self.declare_parameter('blend_slope', 6.0).get_parameter_value().double_value
        self.d_mid = self.declare_parameter('blend_mid_distance', 0.8).get_parameter_value().double_value
        self.max_reach = self.declare_parameter('max_reach', 0.8).get_parameter_value().double_value

        self.d_retract_enter  = self.declare_parameter('d_retract_enter', 0.25).get_parameter_value().double_value
        self.d_retract_exit   = self.declare_parameter('d_retract_exit', 0.35).get_parameter_value().double_value

        self.ee_lin_lim  = self.declare_parameter('ee_lin_limit', 0.15).get_parameter_value().double_value
        self.ee_ang_lim  = self.declare_parameter('ee_ang_limit', 0.6).get_parameter_value().double_value
        self.base_lin_lim= self.declare_parameter('base_lin_limit', 5.0).get_parameter_value().double_value
        self.base_ang_lim= self.declare_parameter('base_ang_limit', 0.80).get_parameter_value().double_value

        self.base_is_holonomic = self.declare_parameter('base_is_holonomic', True).get_parameter_value().bool_value
        self.k_heading = self.declare_parameter('k_heading', 1.5).get_parameter_value().double_value
        self.base_cmd_scale = self.declare_parameter('base_cmd_scale', 500.0).get_parameter_value().double_value
        self.base_cmd_sat_distance = self.declare_parameter('base_cmd_sat_distance', 0.5).get_parameter_value().double_value

        alpha_v = self.declare_parameter('vel_lpf_alpha', 0.6).get_parameter_value().double_value
        slew_base   = self.declare_parameter('slew_base',    1.0).get_parameter_value().double_value
        slew_arm_lin= self.declare_parameter('slew_arm_lin', 0.5).get_parameter_value().double_value
        slew_arm_ang= self.declare_parameter('slew_arm_ang', 1.5).get_parameter_value().double_value

        # --- State ---
        self.last_e6 = [0.0]*6
        self.have_ee = False
        self.have_goal = False
        self.have_base = False
        self.retract_mode = False
        self.last_time = self.get_clock().now()
        self._goal_latched = False  # NEW

        # --- IO ---
        mocap_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                               reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        self.sub_ee   = self.create_subscription(PoseStamped, self.ee_pose_topic,   self.on_ee,   mocap_qos)
        # Keep goal sub for compatibility; ignored when use_offset_goal=True
        self.sub_goal = self.create_subscription(PoseStamped, self.goal_pose_topic, self.on_goal, 10)
        self.sub_base = self.create_subscription(PoseStamped, self.base_pose_topic, self.on_base, mocap_qos)

        self.pub_ee_twist = self.create_publisher(TwistStamped, self.ee_twist_topic, 10)
        # self.pub_cmd_vel  = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_cmd_vel  = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)

        # Filters & slew
        self.lpf_base_vx = LPF(alpha_v); self.lpf_base_vy = LPF(alpha_v); self.lpf_base_wz = LPF(alpha_v)
        self.lpf_arm_vx  = LPF(alpha_v); self.lpf_arm_vy  = LPF(alpha_v); self.lpf_arm_vz  = LPF(alpha_v)
        self.lpf_arm_wx  = LPF(alpha_v); self.lpf_arm_wy  = LPF(alpha_v); self.lpf_arm_wz  = LPF(alpha_v)

        self.slew_base_vx = SlewLimiter(slew_base)
        self.slew_base_vy = SlewLimiter(slew_base)
        self.slew_base_wz = SlewLimiter(slew_base)

        self.slew_arm_lin_x = SlewLimiter(slew_arm_lin)
        self.slew_arm_lin_y = SlewLimiter(slew_arm_lin)
        self.slew_arm_lin_z = SlewLimiter(slew_arm_lin)
        self.slew_arm_ang_x = SlewLimiter(slew_arm_ang)
        self.slew_arm_ang_y = SlewLimiter(slew_arm_ang)
        self.slew_arm_ang_z = SlewLimiter(slew_arm_ang)

        self.timer = self.create_timer(0.01, self.spin)  # 100 Hz
        self._last_R_wb = np.eye(3)
        self.get_logger().info('ArmBaseCoordinator (offset-goal capable) ready.')

    # --- Callbacks ---
    def on_ee(self, msg: PoseStamped):
        self.ee = msg; self.have_ee = True

    def on_goal(self, msg: PoseStamped):
        if not self.use_offset_goal:  # only use external goal if offset-goal disabled
            self.goal = msg; self.have_goal = True

    def on_base(self, msg: PoseStamped):
        self.base = msg; self.have_base = True

    # --- Helper: latch goal from offsets ---
    def maybe_latch_offset_goal(self):
        if not (self.use_offset_goal and self.have_base and not self._goal_latched):
            return
        # Build goal pose from base_start ⊕ offset
        pb = np.array([self.base.pose.position.x, self.base.pose.position.y, self.base.pose.position.z])
        qb = (self.base.pose.orientation.x, self.base.pose.orientation.y,
              self.base.pose.orientation.z, self.base.pose.orientation.w)
        R_wb = rotmat_from_quat(*qb)
        t_off = np.array(self.offset_xyz, dtype=float)

        if self.inputs_in_world:
            # base pose is in world; express offset in chosen frame
            if self.offset_frame.lower() == 'base':
                pg = pb + (R_wb @ t_off)    # rotate base-offset into world
            else:
                pg = pb + t_off            # world-offset
            qg = qb                        # keep goal yaw same as base (simple)
            frame_id = 'world'
        else:
            # inputs already in base frame; goal is simply at offset in base frame
            pg = t_off
            qg = (0.0,0.0,0.0,1.0)
            frame_id = 'base'

        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = float(pg[0]), float(pg[1]), float(pg[2])
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = qg
        self.goal = goal
        self.have_goal = True
        self._goal_latched = True
        self.get_logger().info(f"Latched goal from offset ({self.offset_frame}): "
                               f"x={pg[0]:.3f}, y={pg[1]:.3f}, z={pg[2]:.3f}")

    # --- Main loop ---
    def spin(self):
        now = self.get_clock().now()
        dt = (now - getattr(self, 'last_time', now)).nanoseconds * 1e-9
        dt = max(min(dt, 0.05), 1e-3)
        self.last_time = now

        if not (self.have_ee and self.have_base):
            return

        # Latch offset goal once (if enabled)
        self.maybe_latch_offset_goal()
        if not self.have_goal:
            return

        # 1) Build 6D task error e6 in base frame
        if self.inputs_in_world:
            pb = np.array([self.base.pose.position.x, self.base.pose.position.y, self.base.pose.position.z])
            pe = np.array([self.ee.pose.position.x,   self.ee.pose.position.y,   self.ee.pose.position.z])
            pg = np.array([self.goal.pose.position.x, self.goal.pose.position.y, self.goal.pose.position.z])
            qb = (self.base.pose.orientation.x, self.base.pose.orientation.y, self.base.pose.orientation.z, self.base.pose.orientation.w)
            qe = (self.ee.pose.orientation.x,   self.ee.pose.orientation.y,   self.ee.pose.orientation.z,   self.ee.pose.orientation.w)
            qg = (self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z, self.goal.pose.orientation.w)
            
            # R_wm = rotation from World → Marker (the coordinate system OptiTrack reports)
            # Translation offset (in meters) from the mocap marker origin to the robot's base origin,
            # expressed in the marker's local coordinates.
            t_mb = np.array(self.base_marker_offset_xyz, dtype=float)
            # R_mb = rotmat_from_rpy(*self.base_marker_offset_rpy)
            # if self.marker_yaw_flip:
            #     R_mb = rotmat_from_rpy(0.0, 0.0, math.pi) @ R_mb

            if self.ignore_marker_orientation:
                R_wm = np.eye(3)
                qb = (0.0, 0.0, 0.0, 1.0)            # keep downstream math happy
            else:
                R_wm = rotmat_from_quat(*qb)

            R_mb = rotmat_from_rpy(*self.base_marker_offset_rpy)
            if self.ignore_marker_orientation:
                R_mb = rotmat_from_rpy(*self.fixed_marker_to_base_rpy)

            R_wb = R_wm @ R_mb
            # q_mb = quat_from_rpy(*self.base_marker_offset_rpy_with_pi)  # build once
            # q_b = quat_mul(qb, q_mb)  # world→base quaternion
            # q_bw = quat_conj(q_b)
            # yaw_b = yaw_from_quat(*q_b)

            pb = pb + R_wm @ t_mb
            R_bw = R_wb.T
            self._last_R_wb = R_wb

            p_be = R_bw @ (pe - pb)
            p_bg = R_bw @ (pg - pb)
            ex, ey, ez = (p_bg - p_be).tolist()

            if self.ignore_pose_orientations:
                rx = ry = rz = 0.0
            else:
                q_bw = quat_conj(qb)
                q_be = quat_mul(q_bw, qe)
                q_bg = quat_mul(q_bw, qg)
                q_err = quat_mul(q_bg, quat_conj(q_be))
                rx, ry, rz = quat_to_rotvec(*q_err)
        else:
            ex = self.goal.pose.position.x - self.ee.pose.position.x
            ey = self.goal.pose.position.y - self.ee.pose.position.y
            ez = self.goal.pose.position.z - self.ee.pose.position.z
            if self.ignore_pose_orientations:
                rx = ry = rz = 0.0
            else:
                qe = (self.ee.pose.orientation.x, self.ee.pose.orientation.y, self.ee.pose.orientation.z, self.ee.pose.orientation.w)
                qg = (self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z, self.goal.pose.orientation.w)
                q_err = quat_mul(qg, quat_conj(qe))
                rx, ry, rz = quat_to_rotvec(*q_err)

        if not self.ignore_pose_orientations:
            rx *= self.k_ori_w; ry *= self.k_ori_w; rz *= self.k_ori_w
        else:
            rx = ry = rz = 0.0
        e6 = [ex, ey, ez, rx, ry, rz]
        raw_edot = [(e6[i] - self.last_e6[i]) / dt for i in range(6)]
        self.last_e6 = e6[:]
        edot = raw_edot  # (optional: add LPFs if needed)

        # 2) Task-space PD
        vx_task =  self.kp_lin * e6[0] - self.kd_lin * edot[0]
        vy_task =  self.kp_lin * e6[1] - self.kd_lin * edot[1]
        vz_task =  self.kp_lin * e6[2] - self.kd_lin * edot[2]
        wx_task =  self.kp_ang * e6[3] - self.kd_ang * edot[3]
        wy_task =  self.kp_ang * e6[4] - self.kd_ang * edot[4]
        wz_task =  self.kp_ang * e6[5] - self.kd_ang * edot[5]

        def clamp_vec3(x,y,z,lim):
            n = math.sqrt(x*x+y*y+z*z)
            if n > lim and n > 1e-9:
                s = lim/n; return x*s, y*s, z*s
            return x,y,z
        vx_l,vy_l,vz_l = clamp_vec3(vx_task,vy_task,vz_task,self.ee_lin_lim)
        wx_l,wy_l,wz_l = clamp_vec3(wx_task,wy_task,wz_task,self.ee_ang_lim)

        e_xy = math.hypot(e6[0], e6[1])

        # 3) Blend α
        dx = self.goal.pose.position.x - self.base.pose.position.x
        dy = self.goal.pose.position.y - self.base.pose.position.y
        d_base_xy = math.hypot(dx, dy)

        dx_arm = self.ee.pose.position.x - self.base.pose.position.x
        dy_arm = self.ee.pose.position.y - self.base.pose.position.y
        dz_arm = self.ee.pose.position.z - self.base.pose.position.z
        r_arm = min(math.sqrt(dx_arm*dx_arm + dy_arm*dy_arm + dz_arm*dz_arm) / max(self.max_reach,1e-6), 1.0)

        alpha_base = sigmoid(self.k_d * (self.d_mid - d_base_xy))   # α↑ near goal
        alpha_reach = 0.5 + 0.5 * r_arm                             # α↑ when stretched
        alpha = clamp(alpha_base * alpha_reach, 0.0, 1.0)

        # 4) Retract hysteresis
        if d_base_xy < self.d_retract_enter: self.retract_mode = True
        elif d_base_xy > self.d_retract_exit: self.retract_mode = False

        if self.base_cmd_sat_distance > 0.0:
            scale_alpha = clamp(e_xy / max(self.base_cmd_sat_distance, 1e-6), 0.0, 1.0)
        else:
            scale_alpha = 1.0
        base_scale = 1.0 + scale_alpha * (self.base_cmd_scale - 1.0)

        # 5) Base allocation
        b_vx, b_vy, b_wz = alpha * vx_task, alpha * vy_task, alpha * wz_task
        if not self.base_is_holonomic:
            speed = math.hypot(b_vx, b_vy)
            qb = (self.base.pose.orientation.x, self.base.pose.orientation.y,
                  self.base.pose.orientation.z, self.base.pose.orientation.w)
            # yaw_b = yaw_from_quat(*qb)
            if self.ignore_marker_orientation:
                yaw_b = 0.0   # base x aligned with world x under your assumption
            else:
                yaw_b = yaw_from_quat(*qb)
            desired_heading = math.atan2(b_vy, b_vx) if speed > 1e-6 else yaw_b
            heading_err = (desired_heading - yaw_b + math.pi) % (2.0*math.pi) - math.pi
            v_forward = speed * math.cos(heading_err)
            w_heading = self.k_heading * heading_err if speed > 1e-3 else 0.0
            b_vx, b_vy = v_forward, 0.0
            b_wz = clamp(b_wz + w_heading, -self.base_ang_lim, self.base_ang_lim)

        # 5b) Arm allocation
        arm_vx, arm_vy, arm_vz = (1.0 - alpha) * vx_l, (1.0 - alpha) * vy_l, (1.0 - alpha) * vz_l
        arm_wx, arm_wy, arm_wz = (1.0 - alpha) * wx_l, (1.0 - alpha) * wy_l, (1.0 - alpha) * wz_l

        # Retract overrides
        if self.retract_mode:
            arm_vx = arm_vy = 0.0
            arm_vz = -0.05
            arm_wx = arm_wy = arm_wz = 0.0

        # 6) LPF + slew + per-component clamps
        b_vx = self.lpf_base_vx.step(self.slew_base_vx.step(b_vx, dt))
        b_vy = self.lpf_base_vy.step(self.slew_base_vy.step(b_vy, dt))
        b_wz = self.lpf_base_wz.step(self.slew_base_wz.step(b_wz, dt))
        b_vx *= base_scale
        b_vy *= base_scale
        b_wz *= base_scale
        b_lin = math.hypot(b_vx,b_vy)
        if b_lin > self.base_lin_lim and b_lin > 1e-9:
            s = self.base_lin_lim / b_lin; b_vx *= s; b_vy *= s
        b_wz = clamp(b_wz, -self.base_ang_lim, self.base_ang_lim)

        arm_vx = self.lpf_arm_vx.step(self.slew_arm_lin_x.step(arm_vx, dt))
        arm_vy = self.lpf_arm_vy.step(self.slew_arm_lin_y.step(arm_vy, dt))
        arm_vz = self.lpf_arm_vz.step(self.slew_arm_lin_z.step(arm_vz, dt))
        arm_wx = self.lpf_arm_wx.step(self.slew_arm_ang_x.step(arm_wx, dt))
        arm_wy = self.lpf_arm_wy.step(self.slew_arm_ang_y.step(arm_wy, dt))
        arm_wz = self.lpf_arm_wz.step(self.slew_arm_ang_z.step(arm_wz, dt))

        arm_vx = clamp(arm_vx, -self.ee_lin_lim, self.ee_lin_lim)
        arm_vy = clamp(arm_vy, -self.ee_lin_lim, self.ee_lin_lim)
        arm_vz = clamp(arm_vz, -self.ee_lin_lim, self.ee_lin_lim)
        arm_wx = clamp(arm_wx, -self.ee_ang_lim, self.ee_ang_lim)
        arm_wy = clamp(arm_wy, -self.ee_ang_lim, self.ee_ang_lim)
        arm_wz = clamp(arm_wz, -self.ee_ang_lim, self.ee_ang_lim)

        # 6.5) Global stop window
        if e_xy < 0.3 and abs(e6[2]) < 0.3:
            arm_vx = arm_vy = arm_vz = 0.0
            arm_wx = arm_wy = arm_wz = 0.0
            b_vx = b_vy = 0.0
            b_wz = 0.0

        # 7) Publish
        # twb = Twist(); twb.linear.x=float(b_vx); twb.linear.y=float(b_vy); twb.angular.z=float(b_wz)
        # self.pub_cmd_vel.publish(twb)
        # --- new: TwistStamped for mecanum_controller/reference ---
        twb = TwistStamped()
        twb.header.stamp = now.to_msg()
        # Publish in the frame the downstream controller expects.
        if self.inputs_in_world:
            v_world = self._last_R_wb @ np.array([b_vx, b_vy, 0.0])
            cmd_vx, cmd_vy = v_world[0], v_world[1]
            cmd_frame = 'world'
        else:
            cmd_vx, cmd_vy = b_vx, b_vy
            cmd_frame = 'base'
        twb.header.frame_id = cmd_frame
        twb.twist.linear.x  = -float(cmd_vx)
        twb.twist.linear.y  = float(cmd_vy)
        twb.twist.linear.z  = 0.0
        twb.twist.angular.x = 0.0
        twb.twist.angular.y = 0.0
        twb.twist.angular.z = float(b_wz)
        self.pub_cmd_vel.publish(twb)


        twa = TwistStamped(); twa.header.stamp = now.to_msg()
        twa.header.frame_id = 'link_1'
        twa.twist.linear.x,  twa.twist.linear.y,  twa.twist.linear.z  = float(arm_vx), float(arm_vy), float(arm_vz)
        twa.twist.angular.x, twa.twist.angular.y, twa.twist.angular.z = float(arm_wx), float(arm_wy), float(arm_wz)
        # self.pub_ee_twist.publish(twa)

    # end class

def main():
    rclpy.init()
    node = ArmBaseCoordinator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
