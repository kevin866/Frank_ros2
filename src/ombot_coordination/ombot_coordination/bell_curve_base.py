#!/usr/bin/env python3
"""
Bell-curve (Gaussian) turning profile for a smooth single curve.

Publishes TwistStamped to /mecanum_controller/reference:
- vx: constant forward speed
- wz: Gaussian bump (bell curve) -> smooth turn then straight again

Run:
  chmod +x bell_curve_base.py
  ./bell_curve_base.py

Optional args:
  --vx 0.3
  --wz_peak 0.6
  --t_peak 3.0
  --sigma 0.8
  --duration 7.0
  --rate 50
  --topic /mecanum_controller/reference
  --frame base_link
"""

from __future__ import annotations
import math
import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


def gaussian(t: float, t_peak: float, sigma: float) -> float:
    # exp(-(t - t_peak)^2 / (2*sigma^2))
    if sigma <= 1e-9:
        return 0.0
    a = (t - t_peak) / sigma
    return math.exp(-0.5 * a * a)


class BellCurveTurn(Node):
    def __init__(self, args):
        super().__init__("bell_curve_turn_pub")

        self.topic = args.topic
        self.frame = args.frame
        self.rate = args.rate

        self.vx = args.vx
        self.vy = args.vy
        self.wz_peak = args.wz_peak
        self.t_peak = args.t_peak
        self.sigma = args.sigma
        self.duration = args.duration

        self.pub = self.create_publisher(TwistStamped, self.topic, 10)
        self.t0 = self.get_clock().now()

        self.timer = self.create_timer(1.0 / max(1e-6, self.rate), self.tick)

        self.get_logger().info(
            f"Publishing bell-curve turn to {self.topic} @ {self.rate:.1f} Hz\n"
            f"  vx={self.vx:.3f} m/s, vy={self.vy:.3f} m/s\n"
            f"  wz_peak={self.wz_peak:.3f} rad/s, t_peak={self.t_peak:.3f} s, sigma={self.sigma:.3f} s\n"
            f"  duration={self.duration:.3f} s (then publish zeros + exit)"
        )

    def publish_cmd(self, vx: float, vy: float, wz: float):
        now = self.get_clock().now()
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame

        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = float(wz)
        self.pub.publish(msg)

    def tick(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        # Stop after duration: send zeros for a short moment, then quit
        if self.duration > 0.0 and t >= self.duration:
            self.publish_cmd(0.0, 0.0, 0.0)
            if t >= self.duration + 0.5:
                self.get_logger().info("Done. Shutting down.")
                rclpy.shutdown()
            return

        # Bell-curve yaw rate
        bump = gaussian(t, self.t_peak, self.sigma)
        wz = self.wz_peak * bump

        # Constant translation (you can set vy != 0 for a drifted curve on mecanum)
        self.publish_cmd(self.vx, self.vy, wz)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--topic", default="/mecanum_controller/reference")
    p.add_argument("--frame", default="base_link")
    p.add_argument("--rate", type=float, default=50.0)

    p.add_argument("--vx", type=float, default=2.5, help="m/s")
    p.add_argument("--vy", type=float, default=0.00, help="m/s (sideways for mecanum)")

    p.add_argument("--wz_peak", type=float, default=0.60, help="rad/s peak yaw rate (turn strength)")
    p.add_argument("--t_peak", type=float, default=3.0, help="seconds when yaw bump peaks")
    p.add_argument("--sigma", type=float, default=0.8, help="seconds (width of bell curve)")

    p.add_argument("--duration", type=float, default=7.0, help="seconds total; <=0 runs forever")
    args = p.parse_args()

    rclpy.init()
    node = BellCurveTurn(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
