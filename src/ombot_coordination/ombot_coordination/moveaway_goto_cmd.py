#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped

from ombot_msgs.msg import WholeBodyCmd


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(q) -> float:
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class MoveAwayAndGoTo(Node):
    """
    Publishes WholeBodyCmd for:
      - GOTO goal in world_frame
      - RETREAT when obstacle close (from depth ROI)
      - RETURN to latched pose when obstacle removed
    EE command is zero twist (QP compensates).
    """
    def __init__(self):
        super().__init__("moveaway_goto_cmd")

        # ---- topics/frames ----
        self.declare_parameter("cmd_topic", "/wb_cmd")
        self.declare_parameter("depth_topic", "/zed/zed_node/depth/depth_registered")
        self.declare_parameter("world_frame", "world")      # or "odom"
        self.declare_parameter("base_frame", "base_link")   # or "link_1"
        self.declare_parameter("publish_rate", 50.0)

        # ---- obstacle params ----
        self.declare_parameter("roi", [0.40, 0.60, 0.55, 0.80])  # x0,x1,y0,y1 in [0..1]
        self.declare_parameter("min_valid_depth", 0.35)          # ZED min reliable
        self.declare_parameter("max_valid_depth", 5.0)
        self.declare_parameter("trigger_dist", 0.60)
        self.declare_parameter("clear_dist", 0.90)
        self.declare_parameter("depth_percentile", 10.0)         # use 10th percentile instead of min

        # ---- retreat/return ----
        self.declare_parameter("v_back", 0.15)       # m/s backward during retreat
        self.declare_parameter("return_tol_xy", 0.03)
        self.declare_parameter("return_tol_yaw", 0.08)

        # ---- goto params ----
        self.declare_parameter("goal_x", 2.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", 0.0)

        self.declare_parameter("kp_xy", 1.0)
        self.declare_parameter("kp_yaw", 1.0)
        self.declare_parameter("max_v", 0.6)
        self.declare_parameter("max_w", 1.0)

        # ---- load ----
        self.cmd_topic   = self.get_parameter("cmd_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame  = self.get_parameter("base_frame").value
        self.rate_hz     = float(self.get_parameter("publish_rate").value)

        self.roi = list(self.get_parameter("roi").value)
        self.min_valid_depth = float(self.get_parameter("min_valid_depth").value)
        self.max_valid_depth = float(self.get_parameter("max_valid_depth").value)
        self.trigger_dist = float(self.get_parameter("trigger_dist").value)
        self.clear_dist   = float(self.get_parameter("clear_dist").value)
        self.depth_pct    = float(self.get_parameter("depth_percentile").value)

        self.v_back = float(self.get_parameter("v_back").value)
        self.return_tol_xy  = float(self.get_parameter("return_tol_xy").value)
        self.return_tol_yaw = float(self.get_parameter("return_tol_yaw").value)

        self.goal_x   = float(self.get_parameter("goal_x").value)
        self.goal_y   = float(self.get_parameter("goal_y").value)
        self.goal_yaw = float(self.get_parameter("goal_yaw").value)

        self.kp_xy  = float(self.get_parameter("kp_xy").value)
        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.max_v  = float(self.get_parameter("max_v").value)
        self.max_w  = float(self.get_parameter("max_w").value)

        self.declare_parameter("kd_xy", 0.0)
        self.declare_parameter("kd_yaw", 0.0)

        self.kd_xy  = float(self.get_parameter("kd_xy").value)
        self.kd_yaw = float(self.get_parameter("kd_yaw").value)

        self.ee_pose_topic = (self.declare_parameter('ee_pose_topic', '/ee_pose').get_parameter_value().string_value)



        # self.declare_parameter("k_reach_p", 0.6)
        # self.declare_parameter("k_reach_d", 0.05)
        # self.declare_parameter("ee_vx_max", 0.20)

        # self.k_reach_p = float(self.get_parameter("k_reach_p").value)
        # self.k_reach_d = float(self.get_parameter("k_reach_d").value)
        # self.ee_vx_max = float(self.get_parameter("ee_vx_max").value)

      


        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- IO ----
        self.pub = self.create_publisher(WholeBodyCmd, self.cmd_topic, 10)
        self.sub_depth = self.create_subscription(
            Image, self.depth_topic, self.depth_cb, qos_profile_sensor_data
        )

        self.ee_pos_base = None  # cache latest EE position

        self.ee_pose_sub = self.create_subscription(
            PoseStamped,
            self.ee_pose_topic,
            self.ee_pose_cb,
            10
        )


        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

        # ---- runtime state ----
        self.depth_min = float("inf")
        self.mode = "GOTO"    # GOTO / RETREAT / RETURN
        self.return_pose = None  # (x,y,yaw) latched when obstacle triggers

        self.get_logger().info(
            f"MoveAway+GoTo publishing {self.cmd_topic}, depth={self.depth_topic}, "
            f"TF {self.world_frame}->{self.base_frame}, goal=({self.goal_x:.2f},{self.goal_y:.2f},{self.goal_yaw:.2f})"
        )


    def lookup_base_pose(self):
        t = self.tf_buffer.lookup_transform(self.world_frame, self.base_frame, rclpy.time.Time())
        x = float(t.transform.translation.x)
        y = float(t.transform.translation.y)
        yaw = yaw_from_quat(t.transform.rotation)
        return x, y, float(yaw)
    
    def ee_pose_cb(self, msg: PoseStamped):
        # IMPORTANT: make sure this is already in base_link
        if msg.header.frame_id != self.base_frame:
            self.get_logger().warn_throttle(
                2000, f"EE pose frame is {msg.header.frame_id}, expected {self.base_frame}"
            )
            return

        self.ee_pos_base = msg.pose.position


    def depth_cb(self, msg: Image):
        h, w = msg.height, msg.width

        if msg.encoding == "32FC1":
            depth_m = np.frombuffer(msg.data, dtype=np.float32).reshape((h, w))
        elif msg.encoding == "16UC1":
            depth_u16 = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w))
            depth_m = depth_u16.astype(np.float32) * 0.001
        else:
            self.get_logger().warn(f"Unexpected depth encoding: {msg.encoding}")
            self.depth_min = float("inf")
            return

        x0f, x1f, y0f, y1f = self.roi
        x0 = int(np.clip(x0f, 0.0, 1.0) * w)
        x1 = int(np.clip(x1f, 0.0, 1.0) * w)
        y0 = int(np.clip(y0f, 0.0, 1.0) * h)
        y1 = int(np.clip(y1f, 0.0, 1.0) * h)

        roi = depth_m[y0:y1, x0:x1]
        valid = np.isfinite(roi) & (roi > self.min_valid_depth) & (roi < self.max_valid_depth)

        if np.any(valid):
            vals = roi[valid]
            self.depth_min = float(np.percentile(vals, self.depth_pct))
        else:
            self.depth_min = float("inf")

        # spam log (you said OK)
        # self.get_logger().info(
        #     f"[DEPTH] d={self.depth_min:.3f}m mode={self.mode} enc={msg.encoding} roi=({x0}:{x1},{y0}:{y1}) "
        #     f"valid={int(np.count_nonzero(valid))}/{roi.size}"
        # )

    def compute_goto_cmd(self, x, y, yaw, goal):
        gx, gy, gyaw = goal
        ex_w = gx - x
        ey_w = gy - y
        eyaw = wrap_pi(gyaw - yaw)

        # world error -> base frame
        c = math.cos(-yaw)
        s = math.sin(-yaw)
        ex_b = c * ex_w - s * ey_w
        ey_b = s * ex_w + c * ey_w

        bvx = self.kp_xy * ex_b
        bvy = self.kp_xy * ey_b
        bwz = self.kp_yaw * eyaw
        # instead of pure proportional
        bvx = self.max_v * math.tanh(self.kp_xy * ex_b / self.max_v)
        bvy = self.max_v * math.tanh(self.kp_xy * ey_b / self.max_v)
        bwz = self.max_w * math.tanh(self.kp_yaw * eyaw / self.max_w)


        # clamp v
        vmag = math.hypot(bvx, bvy)
        if vmag > self.max_v:
            sc = self.max_v / (vmag + 1e-9)
            bvx *= sc
            bvy *= sc
        bwz = max(-self.max_w, min(self.max_w, bwz))

        dist = math.hypot(ex_w, ey_w)
        return bvx, bvy, bwz, dist, abs(eyaw)

    def tick(self):
        now = self.get_clock().now()

        try:
            x, y, yaw = self.lookup_base_pose()
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed ({self.world_frame}->{self.base_frame}): {e}")
            return

        # ---- transitions: obstacle overrides ----
        if self.mode != "RETREAT" and self.depth_min < self.trigger_dist:
            # latch return pose at the moment we decide to retreat
            self.return_pose = (x, y, yaw)
            self.mode = "RETREAT"
            self.get_logger().warn(f"RETREAT: latch return_pose=({x:.2f},{y:.2f},{yaw:.2f}) depth={self.depth_min:.2f}")

        elif self.mode == "RETREAT" and self.depth_min > self.clear_dist:
            self.mode = "RETURN"
            self.get_logger().warn(f"RETURN: clear depth={self.depth_min:.2f}")

        # ---- compute base cmd by mode ----
        if self.mode == "RETREAT":
            bvx, bvy, bwz = -abs(self.v_back), 0.0, 0.0

        elif self.mode == "RETURN":
            if self.return_pose is None:
                self.mode = "GOTO"
                bvx, bvy, bwz = 0.0, 0.0, 0.0
            else:
                bvx, bvy, bwz, dist, ayaw = self.compute_goto_cmd(x, y, yaw, self.return_pose)
                if dist < self.return_tol_xy and ayaw < self.return_tol_yaw:
                    self.mode = "GOTO"
                    self.get_logger().warn("GOTO: returned to latched pose")
                    bvx, bvy, bwz = 0.0, 0.0, 0.0

        else:  # GOTO
            bvx, bvy, bwz, dist, ayaw = self.compute_goto_cmd(
                x, y, yaw, (self.goal_x, self.goal_y, self.goal_yaw)
            )
            # optional: stop at goal
            if dist < 0.03 and ayaw < 0.08:
                bvx, bvy, bwz = 0.0, 0.0, 0.0

        # ---- publish WholeBodyCmd ----
        cmd = WholeBodyCmd()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.valid = True

        # Base twist in base frame
        v_bx, v_by = float(bvx), float(bvy)
        w_bz = float(bwz)
        if self.ee_pos_base is None:
            self.get_logger().warn_throttle(2000, "No /ee_pose received yet -> skipping EE task")
            # Option A: publish EE zeros (skip EE)
            cmd.ee.linear.x = 0.0
            cmd.ee.linear.y = 0.0
            cmd.ee.linear.z = 0.0
            cmd.ee.angular.x = 0.0
            cmd.ee.angular.y = 0.0
            cmd.ee.angular.z = 0.0
            # still publish base command as usual
        else:
            # Current EE position relative to base (base frame)
            # You need these three numbers from TF or your /ee_pose topic:
            x_be = float(self.ee_pos_base.x)
            y_be = float(self.ee_pos_base.y)
            z_be = float(self.ee_pos_base.z)

            # omega x r
            vx_ind = v_bx + (0.0 * z_be - w_bz * y_be)   # = v_bx - w_bz*y
            vy_ind = v_by + (w_bz * x_be - 0.0 * z_be)   # = v_by + w_bz*x
            vz_ind = 0.0  + (0.0 * y_be - 0.0 * x_be)    # = 0

            cmd.ee.linear.x  = vx_ind
            cmd.ee.linear.y  = vy_ind
            cmd.ee.linear.z  = vz_ind

            cmd.ee.angular.x = 0.0
            cmd.ee.angular.y = 0.0
            cmd.ee.angular.z = w_bz

        cmd.bvx = float(bvx)
        cmd.bvy = float(bvy)
        cmd.bwz = float(bwz)

        self.pub.publish(cmd)

        # spam log
        self.get_logger().info(
            f"[CMD] mode={self.mode} pose=({x:+.2f},{y:+.2f},{yaw:+.2f}) "
            f"b=({bvx:+.2f},{bvy:+.2f},{bwz:+.2f}) depth={self.depth_min:.2f}"
        )


def main():
    rclpy.init()
    node = MoveAwayAndGoTo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
