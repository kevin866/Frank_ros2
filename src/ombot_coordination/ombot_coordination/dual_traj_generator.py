#!/usr/bin/env python3
from __future__ import annotations
import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf2_ros import Buffer, TransformListener
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy

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

        self.declare_parameter("max_base_v", 0.5)  # m/s
        self.declare_parameter("max_base_w", 0.6)  # rad/s
        self.declare_parameter("max_ee_v",   0.5)
        self.declare_parameter("max_ee_w",   1.0)

        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame  = self.get_parameter("base_frame").value
        self.rate_hz     = float(self.get_parameter("rate_hz").value)

        self.T_base = float(self.get_parameter("T_base").value)
        self.T_ee   = float(self.get_parameter("T_ee").value)

        self.max_base_v = float(self.get_parameter("max_base_v").value)
        self.max_base_w = float(self.get_parameter("max_base_w").value)
        self.max_ee_v   = float(self.get_parameter("max_ee_v").value)
        self.max_ee_w   = float(self.get_parameter("max_ee_w").value)

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
        self.ee_t0 = None
        self.ee_start_p = None
        self.ee_start_q = None
        self.ee_goal_p  = None
        self.ee_goal_q  = None

        self.base_t0 = None
        self.base_start_xyyaw = None
        self.base_goal_xyyaw  = None

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
    #     # self.base_t0 = self.get_clock().now()

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
        self.base_t0 = None

        # (Optional) try to initialize immediately if TF is already available
        # try:
        #     p_b_w, R_wb, q_wb = self.lookup_world_T_base()
        #     yaw0 = yaw_from_quat(q_wb)
        #     self.base_start_xyyaw = np.array([p_b_w[0], p_b_w[1], yaw0], dtype=float)
        #     # do NOT start timer here if you're gating on subscriber readiness
        #     # self.base_t0 stays None; tick() will set it when ready
        # except Exception as e:
        #     self.get_logger().warn(f"TF world->base failed in base_goal_cb (will retry in tick): {e}")




    def ee_goal_cb(self, msg: PoseStamped):
        if self.ee_pose is None:
            return

        # compute world goal FIRST
        pg_w = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ], dtype=float)

        qg_w = np.array([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w
        ], dtype=float)

        # latch on position (you can also add orientation later)
        if self.last_ee_goal_pos is not None:
            if np.linalg.norm(pg_w - self.last_ee_goal_pos) < 5e-3:
                return
        self.last_ee_goal_pos = pg_w.copy()
        self.last_ee_goal_quat = qg_w.copy()

        # start pose in BASE frame
        p0 = np.array([
            self.ee_pose.pose.position.x,
            self.ee_pose.pose.position.y,
            self.ee_pose.pose.position.z
        ], dtype=float)

        q0 = np.array([
            self.ee_pose.pose.orientation.x,
            self.ee_pose.pose.orientation.y,
            self.ee_pose.pose.orientation.z,
            self.ee_pose.pose.orientation.w
        ], dtype=float)

        # WORLD -> BASE at start time
        try:
            p_b_w, R_wb, _ = self.lookup_world_T_base()
        except Exception as e:
            self.get_logger().warn(f"TF world->base lookup failed on ee goal: {e}")
            return

        R_bw = R_wb.T
        R_wg = quat_to_rotmat(*qg_w)

        pg_b = R_bw @ (pg_w - p_b_w)
        R_bg = R_bw @ R_wg
        qg_b = rotmat_to_quat(R_bg)

        # init trajectory
        self.ee_start_p = p0
        self.ee_start_q = q0
        self.ee_goal_p  = pg_b
        self.ee_goal_q  = qg_b
        self.ee_t0 = self.get_clock().now()


    def tick(self):
        now = self.get_clock().now()
        # if self.base_t0 is not None and self.base_start_xyyaw is not None and self.base_goal_xyyaw is not None:
            # self.get_logger().info(f"Just checking")

        # initialize start from current base TF
        try:
            p_b_w, R_wb, q_wb = self.lookup_world_T_base()
            yaw0 = yaw_from_quat(q_wb)
            self.base_start_xyyaw = np.array([p_b_w[0], p_b_w[1], yaw0], dtype=float)
        except Exception as e:
            # self.get_logger().warn(f"TF failed in tick: {e}")
            return
        # self.get_logger().warn(f"TF succeeded in tick")

        # ---------- BASE desired twist ----------
        # Only run timing when RR is subscribed to /base_desired_twist
        self.base_t0 = self._hold_t0_until_ready(self.base_t0, now, self.pub_base_des_twist)
        # if self.base_goal_xyyaw is not None:
        #     self.get_logger().info(f"base_goal_xyyaw: {self.base_goal_xyyaw}")
        # if self.base_start_xyyaw is not None:
        #     self.get_logger().info(f"base_start_xyyaw: {self.base_start_xyyaw}")
        # if self.base_t0 is not None:    
        #     self.get_logger().info(f"base_t0: {(now - self.base_t0).nanoseconds * 1e-9:.3f} s since start")
        if self.base_t0 is not None and self.base_start_xyyaw is not None and self.base_goal_xyyaw is not None:
            t = (now - self.base_t0).nanoseconds * 1e-9
            s, sd, _ = quintic_time_scaling(self.T_base, t)
            d = self.base_goal_xyyaw - self.base_start_xyyaw
            d[2] = wrap_pi(d[2])

            # --- reference pose on the line (world frame) ---
            ref = self.base_start_xyyaw + s * d          # [x_ref, y_ref, yaw_ref]
            ref[2] = wrap_pi(ref[2])                     # yaw wrap if you want

            # --- reference velocity (world frame) ---
            vref_world = sd * d                          # [ẋ_ref, ẏ_ref, yaẇ_ref]

            # --- current base pose (world frame) ---
            x, y = p_b_w[0], p_b_w[1]
            yaw = yaw_from_quat(q_wb)

            # After computing s, sd
            if self._log_counter % 20 == 0:   # ~5 Hz if tick is 100 Hz
                self.get_logger().info(
                    f"[BASE TRAJ] "
                    f"s={s:.3f}, sd={sd:.3f} | "
                    f"d=[{d[0]:+.3f}, {d[1]:+.3f}, {d[2]:+.3f}] | "
                    f"ref=[{ref[0]:+.3f}, {ref[1]:+.3f}, {ref[2]:+.3f}] | "
                    f"vref_w=[{vref_world[0]:+.3f}, {vref_world[1]:+.3f}, {vref_world[2]:+.3f}]"
                )

            self._log_counter += 1


            # --- tracking error in world ---
            ex_w = ref[0] - x
            ey_w = ref[1] - y
            eyaw = wrap_pi(ref[2] - yaw)

            # world -> base
            c = math.cos(-yaw); s_y = math.sin(-yaw)
            ex_b = c*ex_w - s_y*ey_w
            ey_b = s_y*ex_w + c*ey_w
            # ex_b = ex_w
            # ey_b = ey_w


            if self._log_counter % 20 == 0:   # ~5 Hz
                self.get_logger().info(
                    f"[TRACK ERR] "
                    f"world=[ex={ex_w:+.3f}, ey={ey_w:+.3f}, eyaw={eyaw:+.3f}] | "
                    f"base =[ex={ex_b:+.3f}, ey={ey_b:+.3f}] | "
                    f"yaw={yaw:+.3f}"
                )


            # reference linear velocity (world -> base)
            vx_ref_b = c*vref_world[0] - s_y*vref_world[1]
            vy_ref_b = s_y*vref_world[0] + c*vref_world[1]
            wz_ref   = vref_world[2]
            if self._log_counter % 20 == 0:   # ~5 Hz if tick ≈100 Hz
                self.get_logger().info(
                    f"[BASE FF] "
                    f"vref_w=[{vref_world[0]:+.3f}, {vref_world[1]:+.3f}, {vref_world[2]:+.3f}] | "
                    f"vref_b=[{vx_ref_b:+.3f}, {vy_ref_b:+.3f}, {wz_ref:+.3f}] | "
                    f"yaw={yaw:+.3f}"
                )


            # --- feedback gains ---
            Kp_track = 1.0    # m/s per m
            Kyaw     = 0.2    # rad/s per rad

            # --- command = feedforward + feedback ---
            vx_b = vx_ref_b + Kp_track * ex_b
            vy_b = vy_ref_b + Kp_track * ey_b
            wz_b = wz_ref   + Kyaw     * eyaw
            self.get_logger().info_throttle(
                    0.2,
                    f"[TRACK CMD] "
                    f"vx_b={vx_b:+.3f} (vx_ref_b={vx_ref_b:+.3f}, Kp*ex={Kp_track*ex_b:+.3f}) | "
                    f"vy_b={vy_b:+.3f} (vy_ref_b={vy_ref_b:+.3f}, Kp*ey={Kp_track*ey_b:+.3f}) | "
                    f"wz_b={wz_b:+.3f} (wz_ref={wz_ref:+.3f}, Kyaw*eyaw={Kyaw*eyaw:+.3f})"
                )


            # clamp
            vxy = math.hypot(vx_b, vy_b)
            if vxy > self.max_base_v:
                k = self.max_base_v / (vxy + 1e-9)
                vx_b *= k; vy_b *= k
            wz_b = clamp(wz_b, -self.max_base_w, self.max_base_w)

            bmsg = TwistStamped()
            bmsg.header.stamp = now.to_msg()
            bmsg.header.frame_id = self.base_frame
            bmsg.twist.linear.x = float(vx_b)
            bmsg.twist.linear.y = float(vy_b)
            bmsg.twist.angular.z = float(wz_b)
            # self.get_logger().info(f"Publishing base twist vx={vx_b:.3f} vy={vy_b:.3f} wz={wz_b:.3f}")

            self.pub_base_des_twist.publish(bmsg)

        # ---------- EE desired pose + twist ----------
        self.ee_t0 = self._hold_t0_until_ready(self.ee_t0, now, self.pub_ee_des_twist)
        if self.ee_t0 is not None and self.ee_start_p is not None and self.ee_goal_p is not None:
            t = (now - self.ee_t0).nanoseconds * 1e-9
            s, sd, _ = quintic_time_scaling(self.T_ee, t)

            p = self.ee_start_p + s * (self.ee_goal_p - self.ee_start_p)
            v = sd * (self.ee_goal_p - self.ee_start_p)  # base frame

            q = slerp(self.ee_start_q, self.ee_goal_q, s)

            # publish desired pose (base frame)
            pmsg = PoseStamped()
            pmsg.header.stamp = now.to_msg()
            pmsg.header.frame_id = self.base_frame
            pmsg.pose.position.x = float(p[0])
            pmsg.pose.position.y = float(p[1])
            pmsg.pose.position.z = float(p[2])
            pmsg.pose.orientation.x = float(q[0])
            pmsg.pose.orientation.y = float(q[1])
            pmsg.pose.orientation.z = float(q[2])
            pmsg.pose.orientation.w = float(q[3])
            self.pub_ee_des_pose.publish(pmsg)

            # angular FF: simplest start = 0 (let your commander PD handle orientation)
            # you can upgrade later by log-map on R_err to generate w_ff.
            tmsg = TwistStamped()
            tmsg.header.stamp = now.to_msg()
            tmsg.header.frame_id = self.base_frame
            tmsg.twist.linear.x = float(clamp(v[0], -self.max_ee_v, self.max_ee_v))
            tmsg.twist.linear.y = float(clamp(v[1], -self.max_ee_v, self.max_ee_v))
            tmsg.twist.linear.z = float(clamp(v[2], -self.max_ee_v, self.max_ee_v))
            # keep angular FF = 0 for now
            self.pub_ee_des_twist.publish(tmsg)

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
