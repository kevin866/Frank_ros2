#!/usr/bin/env python3
"""
ee_twist_cmd_publisher.py

Publishes geometry_msgs/TwistStamped to drive the EeTwistVelocityController.

Usage:
  ros2 run <your_pkg> ee_twist_cmd_publisher.py
or:
  python3 ee_twist_cmd_publisher.py

Examples:
  # Send a constant +x linear velocity in base frame
  ros2 run <your_pkg> ee_twist_cmd_publisher.py --ros-args -p mode:=const -p vx:=0.05

  # Send a circle in XY (vx, vy sine/cosine)
  ros2 run <your_pkg> ee_twist_cmd_publisher.py --ros-args -p mode:=circle -p vxy:=0.05 -p freq:=0.1

  # Send a yaw oscillation (wz sine)
  ros2 run <your_pkg> ee_twist_cmd_publisher.py --ros-args -p mode:=yaw -p wz:=0.6 -p freq:=0.2
"""

import math
from typing import Literal

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


Mode = Literal["const", "circle", "yaw", "zero"]


class EeTwistCmdPublisher(Node):
    def __init__(self) -> None:
        super().__init__("ee_twist_cmd_publisher")

        # ---- parameters ----
        self.declare_parameter("topic", "/ee_twist_cmd")
        self.declare_parameter("frame_id", "base_link")

        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("mode", "const")  # const | circle | yaw | zero

        # const mode
        self.declare_parameter("vx", -0.05)
        self.declare_parameter("vy", 0.0)
        self.declare_parameter("vz", 0.0)
        self.declare_parameter("wx", 0.0)
        self.declare_parameter("wy", 0.0)
        self.declare_parameter("wz", 0.0)

        # circle/yaw mode
        self.declare_parameter("vxy", 0.00)   # circle amplitude (m/s)
        self.declare_parameter("freq", 0.10)  # Hz
        self.declare_parameter("wz_amp", 0.6) # yaw amplitude (rad/s)

        self.topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.pub = self.create_publisher(TwistStamped, self.topic, 10)
        self.t0 = self.get_clock().now()

        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1.0), self._tick)

        self.get_logger().info(
            f"Publishing TwistStamped to {self.topic} at {self.rate_hz:.1f} Hz (frame_id='{self.frame_id}')."
        )
        self.get_logger().info(
            "Set controller param twist_topic to match this topic, e.g. twist_topic: /ee_twist_cmd"
        )

    def _tick(self) -> None:
        mode: str = str(self.get_parameter("mode").value)

        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id

        if mode == "zero":
            # all zeros
            pass

        elif mode == "const":
            msg.twist.linear.x = float(self.get_parameter("vx").value)
            msg.twist.linear.y = float(self.get_parameter("vy").value)
            msg.twist.linear.z = float(self.get_parameter("vz").value)
            msg.twist.angular.x = float(self.get_parameter("wx").value)
            msg.twist.angular.y = float(self.get_parameter("wy").value)
            msg.twist.angular.z = float(self.get_parameter("wz").value)

        elif mode == "circle":
            amp = float(self.get_parameter("vxy").value)
            freq = float(self.get_parameter("freq").value)
            w = 2.0 * math.pi * freq
            msg.twist.linear.x = amp * math.cos(w * t)
            msg.twist.linear.y = amp * math.sin(w * t)
            msg.twist.linear.z = 0.0
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = 0.0

        elif mode == "yaw":
            freq = float(self.get_parameter("freq").value)
            amp = float(self.get_parameter("wz_amp").value)
            w = 2.0 * math.pi * freq
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = amp * math.sin(w * t)

        else:
            self.get_logger().warn(f"Unknown mode='{mode}'. Use const|circle|yaw|zero. Sending zero twist.")

        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = EeTwistCmdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()