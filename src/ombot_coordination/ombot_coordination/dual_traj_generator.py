#!/usr/bin/env python3
from __future__ import annotations
import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf2_ros import Buffer, TransformListener
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
import rclpy.logging
from ombot_msgs.msg import WholeBodyCmd


# ---------------- helpers ----------------
def clamp(x, lo, hi): return max(lo, min(hi, x))



def wrap_pi(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def quat_to_rotmat(qx,qy,qz,qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-9:
        return np.eye(3)
    qx/=n; qy/=n; qz/=n; qw/=n
    xx = qx*qx; yy=qy*qy; zz=qz*qz
    xy = qx*qy; xz=qx*qz; yz=qy*qz
    wx = qw*qx; wy=qw*qy; wz=qw*qz
    return np.array([
        [1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)],
        [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)],
        [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)]
    ], dtype=float)

def yaw_from_quat(q):
    x,y,z,w = q
    # yaw (Z)
    siny = 2*(w*z + x*y)
    cosy = 1 - 2*(y*y + z*z)
    return math.atan2(siny, cosy)

def rotmat_to_quat(R):
    # simple stable conversion
    tr = float(np.trace(R))
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2,1]-R[1,2]) / S
        qy = (R[0,2]-R[2,0]) / S
        qz = (R[1,0]-R[0,1]) / S
    else:
        # pick major diagonal
        if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            S = math.sqrt(1.0 + R[0,0]-R[1,1]-R[2,2]) * 2
            qw = (R[2,1]-R[1,2]) / S
            qx = 0.25 * S
            qy = (R[0,1]+R[1,0]) / S
            qz = (R[0,2]+R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = math.sqrt(1.0 + R[1,1]-R[0,0]-R[2,2]) * 2
            qw = (R[0,2]-R[2,0]) / S
            qx = (R[0,1]+R[1,0]) / S
            qy = 0.25 * S
            qz = (R[1,2]+R[2,1]) / S
        else:
            S = math.sqrt(1.0 + R[2,2]-R[0,0]-R[1,1]) * 2
            qw = (R[1,0]-R[0,1]) / S
            qx = (R[0,2]+R[2,0]) / S
            qy = (R[1,2]+R[2,1]) / S
            qz = 0.25 * S
    return np.array([qx,qy,qz,qw], dtype=float)

def slerp(q0, q1, u):
    # u in [0,1]
    q0 = q0 / (np.linalg.norm(q0)+1e-12)
    q1 = q1 / (np.linalg.norm(q1)+1e-12)
    dot = float(np.dot(q0,q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = clamp(dot, -1.0, 1.0)
    if dot > 0.9995:
        # nearly linear
        q = q0 + u*(q1-q0)
        return q / (np.linalg.norm(q)+1e-12)
    theta = math.acos(dot)
    s0 = math.sin((1-u)*theta) / math.sin(theta)
    s1 = math.sin(u*theta) / math.sin(theta)
    return s0*q0 + s1*q1

def quintic_time_scaling(T, t):
    # s(0)=0 s(T)=1, sdot(0)=sdot(T)=0, sddot(0)=sddot(T)=0
    if T <= 1e-6:
        return 1.0, 0.0, 0.0
    tau = clamp(t / T, 0.0, 1.0)
    s  = 10*tau**3 - 15*tau**4 + 6*tau**5
    sd = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
    sdd= (60*tau - 180*tau**2 + 120*tau**3) / (T*T)
    return s, sd, sdd
def bezier_quad(p0, p1, p2, s):
    om = 1.0 - s
    return (om*om)*p0 + (2.0*om*s)*p1 + (s*s)*p2

def dbezier_quad_ds(p0, p1, p2, s):
    om = 1.0 - s
    return 2.0*om*(p1 - p0) + 2.0*s*(p2 - p1)
def closest_s_on_bezier(p0, p1, p2, x, n=51):
    # sample s in [0,1] and pick nearest point
    best_s = 0.0
    best_d2 = 1e18
    for i in range(n):
        s = i / (n - 1)
        p = bezier_quad(p0, p1, p2, s)
        d2 = float((p[0]-x[0])**2 + (p[1]-x[1])**2)
        if d2 < best_d2:
            best_d2 = d2
            best_s = s
    return best_s
def quat_inv(q):
    # q = [x,y,z,w]
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)

def quat_mul(q1, q2):
    x1,y1,z1,w1 = q1
    x2,y2,z2,w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ], dtype=float)

# --------------- node ----------------
class DualTrajGenerator(Node):
    def __init__(self):
        super().__init__("dual_traj_generator")
        self._log_counter = 0

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("rate_hz", 100.0)

        self.declare_parameter("T_base", 2.0)  # seconds
        self.declare_parameter("T_ee",   2.0)

        self.declare_parameter("max_base_v", 2.5)  # m/s
        self.declare_parameter("max_base_w", 2.5)  # rad/s
        self.declare_parameter("max_ee_v",   2.5)
        self.declare_parameter("max_ee_w",   2.5)

        self.declare_parameter("curve_frac", 0.15)  # 0 = straight line, 0.1~0.3 gentle curve
        self.curve_frac = float(self.get_parameter("curve_frac").value)
        self.declare_parameter("lookahead_dist", 0.30)  # meters
        self.declare_parameter("lookahead_gain", 0.15)  # extra lookahead per speed (optional)
        self.lookahead_dist = float(self.get_parameter("lookahead_dist").value)
        self.lookahead_gain = float(self.get_parameter("lookahead_gain").value)

        self.declare_parameter("v_cruise", 0.25)      # m/s
        self.declare_parameter("a_brake", 0.6)        # m/s^2 for slowing near goal
        self.declare_parameter("k_yaw_goal", 1.0)     # yaw P gain
        self.v_cruise = float(self.get_parameter("v_cruise").value)
        self.a_brake  = float(self.get_parameter("a_brake").value)
        self.k_yaw_goal = float(self.get_parameter("k_yaw_goal").value)

        self._prev_base_pose = None
        self._prev_base_time = None


        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame  = self.get_parameter("base_frame").value
        self.rate_hz     = float(self.get_parameter("rate_hz").value)

        self.T_base = float(self.get_parameter("T_base").value)
        self.T_ee   = float(self.get_parameter("T_ee").value)

        self.max_base_v = float(self.get_parameter("max_base_v").value)
        self.max_base_w = float(self.get_parameter("max_base_w").value)
        self.max_ee_v   = float(self.get_parameter("max_ee_v").value)
        self.max_ee_w   = float(self.get_parameter("max_ee_w").value)

        self.declare_parameter("Kp_track", 1.0)   # m/s per m
        self.declare_parameter("Kyaw", 1.0)       # rad/s per rad
        self.declare_parameter("Kd_track", 0.1)   # m/s per (m/s)
        self.declare_parameter("Kd_yaw", 0.1)     # rad/s per (rad/s)

        self.Kp_track = float(self.get_parameter("Kp_track").value)
        self.Kyaw = float(self.get_parameter("Kyaw").value)

        self.Kd_track = float(self.get_parameter("Kd_track").value)
        self.Kd_yaw = float(self.get_parameter("Kd_yaw").value)


        self.base_cmd_b = None   # np.array([bvx,bvy,bwz])
        self.ee_cmd     = None   # np.array([vx,vy,vz,wx,wy,wz])


        self.pub_wb_cmd = self.create_publisher(WholeBodyCmd, "/wb_cmd", 10)


        # --- Latch memory (must exist before callbacks fire) ---
        self.last_base_goal_xyyaw = None
        self.last_ee_goal_pos = None
        self.last_ee_goal_quat = None


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # current EE pose (base frame)
        self.ee_pose = None
        self.create_subscription(PoseStamped, "/ee_pose", self.ee_cb, qos_profile_sensor_data)

        # goals in WORLD
        self.base_goal = None
        self.ee_goal   = None
        self.create_subscription(PoseStamped, "/base_goal_pose", self.base_goal_cb, reliable_qos)
        self.create_subscription(PoseStamped, "/goal_pose",      self.ee_goal_cb,   reliable_qos)

        # outputs (base frame)
        self.pub_ee_des_pose   = self.create_publisher(PoseStamped,  "/ee_desired_pose",  reliable_qos)
        self.pub_ee_des_twist  = self.create_publisher(TwistStamped, "/ee_desired_twist", reliable_qos)
        self.pub_base_des_twist= self.create_publisher(TwistStamped, "/base_desired_twist", reliable_qos)

        # trajectory state
        self.t0_base = None
        self.t0_ee   = None

        self.ee_start_p = None
        self.ee_start_q = None
        self.ee_goal_p  = None
        self.ee_goal_q  = None

        # EE endpoints in WORLD (for world-straight line)
        self.ee_start_p_w = None
        self.ee_goal_p_w  = None
        self.ee_start_q_w = None
        self.ee_goal_q_w  = None


        self.base_start_xyyaw = None
        self.base_goal_xyyaw  = None

        # meters / radians
        self.xy_tol = 0.05          # 5 cm
        self.yaw_tol = 5.0 * math.pi/180.0   # 5 deg
        self.v_tol = 0.03           # m/s (optional)
        self.w_tol = 5.0 * math.pi/180.0     # rad/s (optional)

        self.base_done = False


        self.timer = self.create_timer(1.0/self.rate_hz, self.tick)

    def ee_cb(self, msg: PoseStamped):
        self.ee_pose = msg

    def _pub_ready(self, pub):
        try:
            return pub.get_subscription_count() > 0
        except Exception:
            return True  # be permissive if API differs

    def _hold_t0_until_ready(self, t0, now, pub):
        # If nobody is listening, pause the clock by resetting t0
        if not self._pub_ready(pub):
            return None
        # First moment someone is listening: start timing "for real"
        if t0 is None:
            return now
        return t0

    def lookup_world_T_base(self):
        tf = self.tf_buffer.lookup_transform(self.world_frame, self.base_frame, rclpy.time.Time())
        t = tf.transform.translation
        q = tf.transform.rotation
        p_b_w = np.array([t.x, t.y, t.z], dtype=float)
        R_wb  = quat_to_rotmat(q.x, q.y, q.z, q.w)
        q_wb  = np.array([q.x, q.y, q.z, q.w], dtype=float)
        return p_b_w, R_wb, q_wb


    # def base_goal_cb(self, msg: PoseStamped):
    #     # self.get_logger().info("Got /base_goal_pose")

    #     # compute yaw goal FIRST
    #     yawg = yaw_from_quat(np.array([
    #         msg.pose.orientation.x, msg.pose.orientation.y,
    #         msg.pose.orientation.z, msg.pose.orientation.w
    #     ], dtype=float))

    #     goal_xyyaw = np.array([msg.pose.position.x, msg.pose.position.y, yawg], dtype=float)
    #     self.base_goal_xyyaw = goal_xyyaw
    #     # self.t0_base = self.get_clock().now()

    #     # latch
    #     if self.last_base_goal_xyyaw is not None:
    #         if np.linalg.norm(goal_xyyaw - self.last_base_goal_xyyaw) < 5e-3:
    #             self.get_logger().info(f"base goal too close to last, ignoring")

    #             return
    #     self.last_base_goal_xyyaw = goal_xyyaw.copy()

    #     # initialize start from current base TF
    #     try:
    #         p_b_w, R_wb, q_wb = self.lookup_world_T_base()
    #     # except Exception as e:
    #     #     self.get_logger().warn(f"TF world->base lookup failed on base goal: {e}")
    #     #     return
    #     except Exception as e:
    #         self.get_logger().warn(f"TF failed in tick: {e}")
    #         return

    def base_goal_cb(self, msg: PoseStamped):
        # goal yaw
        yawg = yaw_from_quat(np.array([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w
        ], dtype=float))

        goal_xyyaw = np.array([msg.pose.position.x, msg.pose.position.y, yawg], dtype=float)

        # latch with separate thresholds (meters vs radians)
        if self.last_base_goal_xyyaw is not None:
            dx = goal_xyyaw[0] - self.last_base_goal_xyyaw[0]
            dy = goal_xyyaw[1] - self.last_base_goal_xyyaw[1]
            dyaw = wrap_pi(goal_xyyaw[2] - self.last_base_goal_xyyaw[2])
            if math.hypot(dx, dy) < 5e-3 and abs(dyaw) < 1e-2:
                # self.get_logger().info(
                #     f"Base goal ignored: Δpos={math.hypot(dx, dy):.4f} m, "
                #     f"Δyaw={math.degrees(dyaw):.2f} deg"
                # )
                return


        self.last_base_goal_xyyaw = goal_xyyaw.copy()

        # Always store the goal.
        self.base_goal_xyyaw = goal_xyyaw

        # Mark start as "pending init" until TF is available
        self.base_start_xyyaw = None
        # self.t0_base = None

        # (Optional) try to initialize immediately if TF is already available
        # try:
        #     p_b_w, R_wb, q_wb = self.lookup_world_T_base()
        #     yaw0 = yaw_from_quat(q_wb)
        #     self.base_start_xyyaw = np.array([p_b_w[0], p_b_w[1], yaw0], dtype=float)
        #     # do NOT start timer here if you're gating on subscriber readiness
        #     # self.t0_base stays None; tick() will set it when ready
        # except Exception as e:
        #     self.get_logger().warn(f"TF world->base failed in base_goal_cb (will retry in tick): {e}")

    def ee_goal_cb(self, msg: PoseStamped):
        if self.ee_pose is None:
            return

        # goal in WORLD (assume /goal_pose is in world frame)
        pg_w = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
        qg_w = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                        msg.pose.orientation.z, msg.pose.orientation.w], dtype=float)

        # latch on position
        if self.last_ee_goal_pos is not None:
            if np.linalg.norm(pg_w - self.last_ee_goal_pos) < 5e-3:
                return
        self.last_ee_goal_pos = pg_w.copy()
        self.last_ee_goal_quat = qg_w.copy()

        # current EE pose in BASE (from /ee_pose)
        p0_b = np.array([self.ee_pose.pose.position.x,
                        self.ee_pose.pose.position.y,
                        self.ee_pose.pose.position.z], dtype=float)
        q0_b = np.array([self.ee_pose.pose.orientation.x,
                        self.ee_pose.pose.orientation.y,
                        self.ee_pose.pose.orientation.z,
                        self.ee_pose.pose.orientation.w], dtype=float)

        # base pose in WORLD at start
        try:
            p_b_w, R_wb, q_wb = self.lookup_world_T_base()
        except Exception as e:
            self.get_logger().warn(f"TF world->base lookup failed on ee goal: {e}")
            return

        # convert start pose BASE->WORLD
        p0_w = p_b_w + R_wb @ p0_b

        R_be0 = quat_to_rotmat(*q0_b)
        R_we0 = R_wb @ R_be0
        q0_w = rotmat_to_quat(R_we0)

        # store WORLD endpoints
        self.ee_start_p_w = p0_w
        self.ee_start_q_w = q0_w
        self.ee_goal_p_w  = pg_w
        self.ee_goal_q_w  = qg_w

        # start time (optionally gate like base; but keep simple for now)
        self.t0_ee = self.get_clock().now()



    # def ee_goal_cb(self, msg: PoseStamped):
    #     if self.ee_pose is None:
    #         return

    #     # compute world goal FIRST
    #     pg_w = np.array([
    #         msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
    #     ], dtype=float)

    #     qg_w = np.array([
    #         msg.pose.orientation.x, msg.pose.orientation.y,
    #         msg.pose.orientation.z, msg.pose.orientation.w
    #     ], dtype=float)

    #     # latch on position (you can also add orientation later)
    #     if self.last_ee_goal_pos is not None:
    #         if np.linalg.norm(pg_w - self.last_ee_goal_pos) < 5e-3:
    #             return
    #     self.last_ee_goal_pos = pg_w.copy()
    #     self.last_ee_goal_quat = qg_w.copy()

    #     # start pose in BASE frame
    #     p0 = np.array([
    #         self.ee_pose.pose.position.x,
    #         self.ee_pose.pose.position.y,
    #         self.ee_pose.pose.position.z
    #     ], dtype=float)

    #     q0 = np.array([
    #         self.ee_pose.pose.orientation.x,
    #         self.ee_pose.pose.orientation.y,
    #         self.ee_pose.pose.orientation.z,
    #         self.ee_pose.pose.orientation.w
    #     ], dtype=float)

    #     # WORLD -> BASE at start time
    #     try:
    #         p_b_w, R_wb, _ = self.lookup_world_T_base()
    #     except Exception as e:
    #         self.get_logger().warn(f"TF world->base lookup failed on ee goal: {e}")
    #         return

    #     R_bw = R_wb.T
    #     R_wg = quat_to_rotmat(*qg_w)

    #     pg_b = R_bw @ (pg_w - p_b_w)
    #     R_bg = R_bw @ R_wg
    #     qg_b = rotmat_to_quat(R_bg)

    #     # init trajectory
    #     self.ee_start_p = p0
    #     self.ee_start_q = q0
    #     self.ee_goal_p  = pg_b
    #     self.ee_goal_q  = qg_b
    #     self.t0_ee = self.get_clock().now()


    def tick(self):
        now = self.get_clock().now()
        # if self.t0_base is not None and self.base_start_xyyaw is not None and self.base_goal_xyyaw is not None:
            # self.get_logger().info(f"Just checking")

        # initialize start from current base TF
        try:
            p_b_w, R_wb, q_wb = self.lookup_world_T_base()
            yaw0 = yaw_from_quat(q_wb)
            # self.base_start_xyyaw = np.array([p_b_w[0], p_b_w[1], yaw0], dtype=float)    
        except Exception as e:
            # self.get_logger().warn(f"TF failed in tick: {e}")
            return
        # self.get_logger().warn(f"TF succeeded in tick")
        if self.base_goal_xyyaw is not None and self.base_start_xyyaw is None:
            self.base_start_xyyaw = np.array([p_b_w[0], p_b_w[1], yaw0], dtype=float)

        
        # ---------- BASE desired twist ----------
        # Only run timing when RR is subscribed to /base_desired_twist
        self.t0_base = self._hold_t0_until_ready(self.t0_base, now, self.pub_wb_cmd)
        # if self.base_goal_xyyaw is not None:
        #     self.get_logger().info(f"base_goal_xyyaw: {self.base_goal_xyyaw}")
        # if self.base_start_xyyaw is not None:
        #     self.get_logger().info(f"base_start_xyyaw: {self.base_start_xyyaw}")
        # if self.t0_base is not None:    
        #     self.get_logger().info(f"t0: {(now - self.t0_base).nanoseconds * 1e-9:.3f} s since start")
        base_ready = (
            self.t0_base is not None
            and self.base_start_xyyaw is not None
            and self.base_goal_xyyaw is not None
        )
        v_act_b = np.zeros(3)

        if base_ready:
            x, y = p_b_w[0], p_b_w[1]
            yaw = yaw_from_quat(q_wb)
            

            if self._prev_base_pose is not None and self._prev_base_time is not None:
                dt = (now - self._prev_base_time).nanoseconds * 1e-9
                if dt > 1e-4:
                    dx_w = x - self._prev_base_pose[0]
                    dy_w = y - self._prev_base_pose[1]
                    dyaw = wrap_pi(yaw - self._prev_base_pose[2])

                    # world vel
                    vx_w = dx_w / dt
                    vy_w = dy_w / dt

                    # world -> base (use same yaw you already use)
                    c = math.cos(-yaw); s_y = math.sin(-yaw)
                    vx_b_act = c * vx_w - s_y * vy_w
                    vy_b_act = s_y * vx_w + c * vy_w
                    wz_b_act = dyaw / dt

                    v_act_b[:] = [vx_b_act, vy_b_act, wz_b_act]

            # update history
            self._prev_base_pose = np.array([x, y, yaw], dtype=float)
            self._prev_base_time = now
            
            d = self.base_goal_xyyaw - self.base_start_xyyaw
            d[2] = wrap_pi(d[2])

            # ----- Build Bezier p0,p1,p2 exactly like you already do -----
            p0 = self.base_start_xyyaw[:2].copy()
            p2 = self.base_goal_xyyaw[:2].copy()
            dp = p2 - p0
            L = float(np.linalg.norm(dp))

            if L < 1e-6:
                p1 = p0.copy()
            else:
                curve_mag = float(np.clip(0.0 * L, 0.0, 0.50))
                t_hat = dp / L
                n_hat = np.array([-t_hat[1], t_hat[0]])
                p1 = 0.5*(p0 + p2) + curve_mag * n_hat
                # if self._log_counter % 50 == 0:   # ~5 Hz if tick is 100 Hz
                #     self.get_logger().info(
                #         f"[BASE BEZIER] "
                #         f"p0=({p0[0]:+.3f}, {p0[1]:+.3f}) | "
                #         f"p1=({p1[0]:+.3f}, {p1[1]:+.3f}) | "
                #         f"p2=({p2[0]:+.3f}, {p2[1]:+.3f}) | "
                #         f"L={L:.3f}"
                #     )


            # ----- Lookahead target -----
            robot_xy = np.array([p_b_w[0], p_b_w[1]], dtype=float)
            s_closest = closest_s_on_bezier(p0, p1, p2, robot_xy, 61)

            # choose Ld (meters)
            # v_now = np.linalg.norm(v_act_b[:2]) if self._prev_base_time is not None else 0.0
            v_now = float(np.linalg.norm(v_act_b[:2]))

            Ld = float(np.clip(self.lookahead_dist + self.lookahead_gain * v_now, 0.10, 1.00))

            dpds = dbezier_quad_ds(p0, p1, p2, s_closest)
            ds = Ld / (float(np.linalg.norm(dpds)) + 1e-9)
            s_la = float(np.clip(s_closest + ds, 0.0, 1.0))

            pref_xy = bezier_quad(p0, p1, p2, s_la)

            # ----- Tangent at lookahead (for feedforward direction) -----
            dpds_la = dbezier_quad_ds(p0, p1, p2, s_la)
            tangent = dpds_la / (float(np.linalg.norm(dpds_la)) + 1e-9)

            # distance to final goal (not lookahead point)
            goal_xy = p2
            d_goal = float(np.linalg.norm(goal_xy - robot_xy))

            # braking-speed rule (no time): v <= sqrt(2 a d)
            v_brake = math.sqrt(max(0.0, 2.0 * self.a_brake * d_goal))
            v_ref = min(self.v_cruise, v_brake)

            vref_xy = v_ref * tangent

            yaw = yaw_from_quat(q_wb)
            eyaw = wrap_pi(self.base_goal_xyyaw[2] - yaw)

            goal_yaw = self.base_goal_xyyaw[2]

            # --- Yaw control ---
            yaw_ref = goal_yaw

            eyaw_ref = wrap_pi(yaw_ref - yaw)

            wz_ref = clamp(
                self.k_yaw_goal * eyaw_ref,
                -self.max_base_w,
                self.max_base_w
            )
       
            ref = np.array([pref_xy[0], pref_xy[1], goal_yaw], dtype=float)
            vref_world = np.array([vref_xy[0], vref_xy[1], wz_ref], dtype=float)


            self._log_counter += 1


            # --- tracking error in world ---
            ex_w = ref[0] - x
            ey_w = ref[1] - y
            # eyaw = wrap_pi(ref[2] - yaw)

            # world -> base
            c = math.cos(-yaw); s_y = math.sin(-yaw)
            ex_b = c*ex_w - s_y*ey_w
            ey_b = s_y*ex_w + c*ey_w

            # reference linear velocity (world -> base)
            vx_ref_b = c*vref_world[0] - s_y*vref_world[1]
            vy_ref_b = s_y*vref_world[0] + c*vref_world[1]
            wz_ref   = vref_world[2]

            # velocity error (actual - reference)
            evx = v_act_b[0] - vx_ref_b
            evy = v_act_b[1] - vy_ref_b
            ewz = v_act_b[2] - wz_ref

            # --- command = feedforward + feedback ---
            vx_b = vx_ref_b + self.Kp_track * ex_b + self.Kd_track * evx
            vy_b = vy_ref_b + self.Kp_track * ey_b + self.Kd_track * evy
            wz_b = wz_ref + self.Kyaw * eyaw_ref + self.Kd_yaw * ewz


            # clamp
            vxy = math.hypot(vx_b, vy_b)
            if vxy > self.max_base_v:
                k = self.max_base_v / (vxy + 1e-9)
                vx_b *= k; vy_b *= k
            wz_b = clamp(wz_b, -self.max_base_w, self.max_base_w)

            wz_b = 0.0  # for logging below
            # --- stop condition (use FINAL goal, not lookahead) ---
            pos_done = (d_goal < self.xy_tol)
            yaw_done = (abs(eyaw_ref) < self.yaw_tol)

            # optional: require near-stopped to avoid "drive through" triggering too early
            vel_done = (math.hypot(v_act_b[0], v_act_b[1]) < self.v_tol)
            w_done   = (abs(v_act_b[2]) < self.w_tol)

            if pos_done and yaw_done and vel_done and w_done:
                self.base_done = True

            if self.base_done:
                # publish zero base twist and optionally clear goal/start
                bmsg = TwistStamped()
                bmsg.header.stamp = now.to_msg()
                bmsg.header.frame_id = self.base_frame
                bmsg.twist.linear.x = 0.0
                bmsg.twist.linear.y = 0.0
                bmsg.twist.angular.z = 0.0
                self.pub_base_des_twist.publish(bmsg)
                self.base_cmd_b = np.zeros(3, dtype=float)

                # optional: end the base segment so it won't restart
                # self.base_goal_xyyaw = None
                # self.base_start_xyyaw = None
                # self.t0_base = None

                # and skip the rest of base control
                # (but keep EE part running if you want)
                goto_base_done = True
            else:
                goto_base_done = False
            wz_b = 0.0  # for logging below

            if not goto_base_done:
                bmsg = TwistStamped()
                bmsg.header.stamp = now.to_msg()
                bmsg.header.frame_id = self.base_frame
                bmsg.twist.linear.x = float(vx_b)
                bmsg.twist.linear.y = float(vy_b)
                bmsg.twist.angular.z = float(wz_b)
                # if self._log_counter % 50 == 0:   # ~5 Hz
                    # self.get_logger().info(f"Publishing base twist vx={vx_b:.3f} vy={vy_b:.3f} wz={wz_b:.3f}")
                self.pub_base_des_twist.publish(bmsg)
                self.base_cmd_b = np.array([float(vx_b), float(vy_b), float(wz_b)], dtype=float)

        # ---------- EE desired pose + twist ----------

        ee_ready = (
            self.t0_ee is not None
            and self.ee_start_p_w is not None
            and self.ee_goal_p_w  is not None
            and self.ee_start_q_w is not None
            and self.ee_goal_q_w  is not None
        )

        if ee_ready:
            t = (now - self.t0_ee).nanoseconds * 1e-9
            s, sd, _ = quintic_time_scaling(self.T_ee, t)

            # ---- EE WORLD desired (from quintic) ----
            dp_w = (self.ee_goal_p_w - self.ee_start_p_w)
            p_w_des = self.ee_start_p_w + s  * dp_w
            v_w_ff  =                    sd * dp_w

            # ---- EE WORLD actual (from /ee_pose + base TF) ----
            p_ee_b = np.array([
                self.ee_pose.pose.position.x,
                self.ee_pose.pose.position.y,
                self.ee_pose.pose.position.z
            ], dtype=float)

            p_w_act = p_b_w + R_wb @ p_ee_b
            # self.get_logger().info(f"ee_pose frame_id = {self.ee_pose.header.frame_id}")


            # ---- feedback in WORLD, then convert to BASE ----
            e_w = p_w_des - p_w_act

            Kp_ee_world = 1.5   # start small like 0.5~2.0 (units 1/s)
            v_w_fb = Kp_ee_world * e_w

            # (optional clamp feedback so it doesn't explode)
            v_w_fb_norm = float(np.linalg.norm(v_w_fb))
            v_w_fb_max = 0.3  # m/s, tune
            if v_w_fb_norm > v_w_fb_max:
                v_w_fb *= v_w_fb_max / (v_w_fb_norm + 1e-9)

            # total desired EE velocity in WORLD
            v_w_des = v_w_ff + v_w_fb

            # ---------------- compensate base-induced EE motion ----------------
            # You already estimated v_act_b earlier in tick() (base twist in BASE frame)
            # v_act_b = [vx_b_act, vy_b_act, wz_b_act]

            # base linear velocity in WORLD
            v_base_b = np.array([v_act_b[0], v_act_b[1], 0.0], dtype=float)   # (vx,vy,0) in base
            v_base_w = R_wb @ v_base_b

            # base angular velocity in WORLD (planar)
            omega_w = np.array([0.0, 0.0, float(v_act_b[2])], dtype=float)

            # EE position relative to base, expressed in WORLD
            r_ee_w = R_wb @ p_ee_b

            # world velocity at the EE point induced by base motion
            v_induced_w = v_base_w + np.cross(omega_w, r_ee_w)

            # desired *relative* EE world velocity that the arm must generate
            v_w_rel_needed = v_w_des - v_induced_w

            # convert to BASE for publishing (controller expects base frame)
            R_bw = R_wb.T
            v_b_des = R_bw @ v_w_rel_needed

            # clamp to max_ee_v
            v_b_des = np.clip(v_b_des, -self.max_ee_v, self.max_ee_v)

            
            # publish desired twist in BASE frame
            tmsg = TwistStamped()
            tmsg.header.stamp = now.to_msg()
            tmsg.header.frame_id = self.base_frame
            tmsg.twist.linear.x = float(v_b_des[0])
            tmsg.twist.linear.y = float(v_b_des[1])
            tmsg.twist.linear.z = float(v_b_des[2])
            self.pub_ee_des_twist.publish(tmsg)

            self.ee_cmd = np.array([
                float(clamp(0.0, -self.max_ee_v, self.max_ee_v)),
                float(clamp(0.0, -self.max_ee_v, self.max_ee_v)),
                float(clamp(0.0, -self.max_ee_v, self.max_ee_v)),
                0.0, 0.0, 0.0
            ], dtype=float)
            # if self._log_counter % 50 == 0:
            #     self.get_logger().info(
            #         f"EE dbg: e_w={e_w} | v_w_ff={v_w_ff} | v_w_fb={v_w_fb} | "
            #         f"v_induced_w={v_induced_w} | v_w_rel_needed={v_w_rel_needed}"
            #     )




        # if ee_ready:
        #     t = (now - self.t0_ee).nanoseconds * 1e-9
        #     s, sd, _ = quintic_time_scaling(self.T_ee, t)

        #     p = self.ee_start_p + s * (self.ee_goal_p - self.ee_start_p)
        #     v = sd * (self.ee_goal_p - self.ee_start_p)  # base frame

        #     q = slerp(self.ee_start_q, self.ee_goal_q, s)

        #     # publish desired pose (base frame)
        #     pmsg = PoseStamped()
        #     pmsg.header.stamp = now.to_msg()
        #     pmsg.header.frame_id = self.base_frame
        #     pmsg.pose.position.x = float(p[0])
        #     pmsg.pose.position.y = float(p[1])
        #     pmsg.pose.position.z = float(p[2])
        #     pmsg.pose.orientation.x = float(q[0])
        #     pmsg.pose.orientation.y = float(q[1])
        #     pmsg.pose.orientation.z = float(q[2])
        #     pmsg.pose.orientation.w = float(q[3])
        #     self.pub_ee_des_pose.publish(pmsg)

        #     # angular FF: simplest start = 0 (let your commander PD handle orientation)
        #     # you can upgrade later by log-map on R_err to generate w_ff.
        #     tmsg = TwistStamped()
        #     tmsg.header.stamp = now.to_msg()
        #     tmsg.header.frame_id = self.base_frame
        #     tmsg.twist.linear.x = float(clamp(v[0], -self.max_ee_v, self.max_ee_v))
        #     tmsg.twist.linear.y = float(clamp(v[1], -self.max_ee_v, self.max_ee_v))
        #     tmsg.twist.linear.z = float(clamp(v[2], -self.max_ee_v, self.max_ee_v))
        #     # if self._log_counter % 50 == 0:  # ~5 Hz
        #     #     v_raw = v
        #     #     v_cmd = np.array([
        #     #         clamp(v[0], -self.max_ee_v, self.max_ee_v),
        #     #         clamp(v[1], -self.max_ee_v, self.max_ee_v),
        #     #         clamp(v[2], -self.max_ee_v, self.max_ee_v),
        #     #     ])

        #         # self.get_logger().info(
        #         #     f"[EE VEL] "
        #         #     f"v_raw=({v_raw[0]:+.3f}, {v_raw[1]:+.3f}, {v_raw[2]:+.3f}) | "
        #         #     f"v_cmd=({v_cmd[0]:+.3f}, {v_cmd[1]:+.3f}, {v_cmd[2]:+.3f})"
        #         # )

        #     # if self._log_counter % 50 == 0:
        #     #     dp = self.ee_goal_p - self.ee_start_p
        #     #     self.get_logger().info(
        #     #         f"[EE TRAJ] frame={self.base_frame} "
        #     #         f"start=({self.ee_start_p[0]:+.3f},{self.ee_start_p[1]:+.3f},{self.ee_start_p[2]:+.3f}) "
        #     #         f"goal=({self.ee_goal_p[0]:+.3f},{self.ee_goal_p[1]:+.3f},{self.ee_goal_p[2]:+.3f}) "
        #     #         f"dp=({dp[0]:+.3f},{dp[1]:+.3f},{dp[2]:+.3f}) "
        #     #         f"v=({v[0]:+.3f},{v[1]:+.3f},{v[2]:+.3f})"
        #     #     )
        #     # if self._log_counter % 50 == 0:
        #     #     self.get_logger().info(
        #     #         f"[EE TS] t={t:.3f} T={self.T_ee:.3f} s={s:.3f} sd={sd:.6f}"
        #     #     )



        #     # keep angular FF = 0 for now
        #     self.pub_ee_des_twist.publish(tmsg)

        #     self.ee_cmd = np.array([float(clamp(v[0], -self.max_ee_v, self.max_ee_v)), 
        #                             float(clamp(v[1], -self.max_ee_v, self.max_ee_v)), 
        #                             float(clamp(v[2], -self.max_ee_v, self.max_ee_v)), 
        #                             0.0, 0.0, 0.0], dtype=float)  # [vx,vy,vz,wx,wy,wz]

        # if self._log_counter % 50 == 0:  # ~5 Hz
        #     self.get_logger().info(
        #         f"[WB ACTIVE] "
        #         f"base_cmd=({self.base_cmd_b[0]:+.3f}, {self.base_cmd_b[1]:+.3f}, {self.base_cmd_b[2]:+.3f}) | "
        #         f"ee_cmd=({self.ee_cmd[0]:+.3f}, {self.ee_cmd[1]:+.3f}, {self.ee_cmd[2]:+.3f})"
        #     )

        both_ready = base_ready and ee_ready and (self.base_cmd_b is not None) and (self.ee_cmd is not None)

        if not both_ready:
            # Optional: publish nothing, or publish cmd.valid=False (your choice)
            return

        cmd = WholeBodyCmd()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.valid = True

        # EE
        cmd.ee.linear.x  = float(self.ee_cmd[0])
        cmd.ee.linear.y  = float(self.ee_cmd[1])
        cmd.ee.linear.z  = float(self.ee_cmd[2])
        cmd.ee.angular.x = float(self.ee_cmd[3])
        cmd.ee.angular.y = float(self.ee_cmd[4])
        cmd.ee.angular.z = float(self.ee_cmd[5])

        # Base
        cmd.bvx = float(self.base_cmd_b[0])
        cmd.bvy = float(self.base_cmd_b[1])
        cmd.bwz = float(self.base_cmd_b[2])

        self.pub_wb_cmd.publish(cmd)
        # if self._log_counter % 50 == 0:
        #     self.get_logger().info(
        #         f"[WB CMD] "
        #         f"base=({cmd.bvx:+.3f}, {cmd.bvy:+.3f}, {cmd.bwz:+.3f}) | "
        #         f"ee_lin=({cmd.ee.linear.x:+.3f}, {cmd.ee.linear.y:+.3f}, {cmd.ee.linear.z:+.3f}) | "
        #         f"ee_ang=({cmd.ee.angular.x:+.3f}, {cmd.ee.angular.y:+.3f}, {cmd.ee.angular.z:+.3f})"
        #     )







            

def main():
    rclpy.init()
    node = DualTrajGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
