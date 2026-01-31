#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from ombot_msgs.msg import WholeBodyCmd

from tf2_ros import Buffer, TransformListener
import rclpy



class DepthMoveAwayCmdPublisher(Node):
    def __init__(self):
        super().__init__('depth_move_away_cmd_publisher')

        # ---------------- Parameters ----------------
        self.declare_parameter('cmd_topic', '/whole_body_cmd')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter("world_frame", "world")
        self.world_frame = self.get_parameter("world_frame").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_x = None
        self.return_tol = 0.02  # 2 cm


        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('publish_rate', 50.0)  # Hz

        # Hysteresis thresholds (meters)
        self.declare_parameter('trigger_dist', 0.60)  # start retreat when closer than this
        self.declare_parameter('clear_dist',   0.80)  # stop retreat when farther than this

        # Retreat speed (m/s). Positive number; node will publish negative bvx.
        self.declare_parameter('v_back', 0.10)

        # ROI in normalized image coordinates: [x0, x1, y0, y1]
        # Middle-bottom usually corresponds to "in front of the base".
        self.declare_parameter('roi', [0.40, 0.60, 0.55, 0.80])

        # Depth validity filtering
        self.declare_parameter('min_valid_depth', 0.15)  # meters
        self.declare_parameter('max_valid_depth', 8.0)   # meters

        # ---------------- Read params ----------------
        self.cmd_topic   = self.get_parameter('cmd_topic').value
        self.base_frame  = self.get_parameter('base_frame').value
        self.depth_topic = self.get_parameter('depth_topic').value

        self.rate         = float(self.get_parameter('publish_rate').value)
        self.trigger_dist = float(self.get_parameter('trigger_dist').value)
        self.clear_dist   = float(self.get_parameter('clear_dist').value)
        self.v_back       = float(self.get_parameter('v_back').value)

        self.roi = self.get_parameter('roi').value
        self.min_valid_depth = float(self.get_parameter('min_valid_depth').value)
        self.max_valid_depth = float(self.get_parameter('max_valid_depth').value)

        # ---------------- State ----------------
        self.depth_min = float('inf')
        self.mode = "CLEAR"   # CLEAR or RETREAT

        # ---------------- Publisher ----------------
        self.pub_wb_cmd = self.create_publisher(WholeBodyCmd, self.cmd_topic, 10)

        # ---------------- Subscriber ----------------
        self.sub_depth = self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)

        # ---------------- Timer ----------------
        self.timer = self.create_timer(1.0 / self.rate, self.publish_cmd)

        self.get_logger().info(
            f"DepthMoveAwayCmdPublisher\n"
            f"  cmd_topic:    {self.cmd_topic}\n"
            f"  depth_topic:  {self.depth_topic}\n"
            f"  trigger/clear:{self.trigger_dist:.2f}/{self.clear_dist:.2f} m\n"
            f"  v_back:       {self.v_back:.3f} m/s\n"
            f"  roi:          {self.roi}\n"
        )

    def lookup_world_T_base_x(self):
        t = self.tf_buffer.lookup_transform(self.world_frame, self.base_frame, rclpy.time.Time())
        return float(t.transform.translation.x)
    
    def get_base_x(self):
        t = self.tf_buffer.lookup_transform(self.world_frame, self.base_frame, rclpy.time.Time())
        return float(t.transform.translation.x)



    def depth_cb(self, msg: Image):
        h, w = msg.height, msg.width

        # ZED depth_registered is usually 32FC1 in meters.
        # Some configs publish 16UC1 (often mm). Handle both.
        if msg.encoding == "32FC1":
            depth_m = np.frombuffer(msg.data, dtype=np.float32).reshape((h, w))
        elif msg.encoding == "16UC1":
            depth_u16 = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w))
            depth_m = depth_u16.astype(np.float32) * 0.001
        else:
            # If this triggers, tell me the encoding string and we’ll adapt.
            self.get_logger().warn_throttle(2.0, f"Unexpected depth encoding: {msg.encoding}")
            self.depth_min = float('inf')
            return

        x0f, x1f, y0f, y1f = self.roi
        x0 = int(np.clip(x0f, 0.0, 1.0) * w)
        x1 = int(np.clip(x1f, 0.0, 1.0) * w)
        y0 = int(np.clip(y0f, 0.0, 1.0) * h)
        y1 = int(np.clip(y1f, 0.0, 1.0) * h)

        roi = depth_m[y0:y1, x0:x1]

        valid = np.isfinite(roi) & (roi > self.min_valid_depth) & (roi < self.max_valid_depth)
        if np.any(valid):
            # min depth is “closest object” in ROI
            self.depth_min = float(np.min(roi[valid]))
        else:
            self.depth_min = float('inf')

        # # Update mode with hysteresis
        # if self.mode == "CLEAR":
        #     if self.depth_min < self.trigger_dist:
        #         self.mode = "RETREAT"
        #         self.get_logger().warn(f"RETREAT: min_depth={self.depth_min:.2f} m")
        # else:
        #     if self.depth_min > self.clear_dist:
        #         self.mode = "CLEAR"
        #         self.get_logger().warn(f"CLEAR: min_depth={self.depth_min:.2f} m")


        if self.mode == "CLEAR":
            if self.depth_min < self.trigger_dist:
                self.mode = "RETREAT"
                self.start_x = self.lookup_world_T_base_x()  # latch
                self.get_logger().warn(f"RETREAT: latch start_x={self.start_x:.3f} m, depth={self.depth_min:.2f}")

        elif self.mode == "RETREAT":
            if self.depth_min > self.clear_dist:
                self.mode = "RETURN"
                self.get_logger().warn(f"RETURN: depth clear {self.depth_min:.2f} m")

        elif self.mode == "RETURN":
            # If obstacle reappears, retreat again immediately
            if self.depth_min < self.trigger_dist:
                self.mode = "RETREAT"
                self.start_x = self.lookup_world_T_base_x()
                self.get_logger().warn("RETREAT again (obstacle came back)")
            else:
                x = self.lookup_world_T_base_x()
                if self.start_x is not None and (x >= self.start_x - self.return_tol):
                    self.mode = "CLEAR"
                    self.get_logger().warn(f"CLEAR: returned to start_x (x={x:.3f})")


        # self.get_logger().info(
        #     f"depth_min={self.depth_min:.3f} m | mode={self.mode} | "
        #     f"encoding={msg.encoding} | ROI=({x0}:{x1},{y0}:{y1})"
        # )


    def publish_cmd(self):
        now = self.get_clock().now()

        cmd = WholeBodyCmd()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.valid = True

        # -------- End Effector (all zeros) --------
        # In your current QP, ee_des = 0 means “hold EE still” (arm compensates base).
        cmd.ee.linear.x  = 0.0
        cmd.ee.linear.y  = 0.0
        cmd.ee.linear.z  = 0.0
        cmd.ee.angular.x = 0.0
        cmd.ee.angular.y = 0.0
        cmd.ee.angular.z = 0.0

        # -------- Base --------
        if self.mode == "RETREAT":
            cmd.bvx = -abs(self.v_back)
        elif self.mode == "RETURN":
            cmd.bvx = +abs(self.v_back)   # forward to return
        else:
            cmd.bvx = 0.0

        cmd.bvy = 0.0
        cmd.bwz = 0.0

        self.pub_wb_cmd.publish(cmd)


def main():
    rclpy.init()
    node = DepthMoveAwayCmdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
