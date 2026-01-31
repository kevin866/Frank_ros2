#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time

# CHANGE this import to match your package
from ombot_msgs.msg import WholeBodyCmd


class WholeBodyCmdPublisher(Node):
    def __init__(self):
        super().__init__('whole_body_cmd_publisher')

        # ---------------- Parameters ----------------
        self.declare_parameter('cmd_topic', '/whole_body_cmd')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('base_vx', 1.0)   # m/s
        self.declare_parameter('publish_rate', 50.0)  # Hz

        self.cmd_topic   = self.get_parameter('cmd_topic').value
        self.base_frame  = self.get_parameter('base_frame').value
        self.base_vx     = float(self.get_parameter('base_vx').value)
        self.rate        = float(self.get_parameter('publish_rate').value)

        # ---------------- Publisher ----------------
        self.pub_wb_cmd = self.create_publisher(
            WholeBodyCmd,
            self.cmd_topic,
            10
        )

        # ---------------- Timer ----------------
        self.timer = self.create_timer(1.0 / self.rate, self.publish_cmd)

        self.get_logger().info(
            f"Publishing WholeBodyCmd on {self.cmd_topic} | "
            f"base_vx={self.base_vx:.3f} m/s"
        )

    def publish_cmd(self):
        now = self.get_clock().now()

        cmd = WholeBodyCmd()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.valid = True

        # -------- End Effector (all zeros) --------
        cmd.ee.linear.x  = 0.0
        cmd.ee.linear.y  = 0.0
        cmd.ee.linear.z  = 0.0
        cmd.ee.angular.x = 0.0
        cmd.ee.angular.y = 0.0
        cmd.ee.angular.z = 0.0

        # -------- Base (small x, zero y & yaw) --------
        cmd.bvx = self.base_vx
        cmd.bvy = 0.0
        cmd.bwz = 0.0

        self.pub_wb_cmd.publish(cmd)


def main():
    rclpy.init()
    node = WholeBodyCmdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
