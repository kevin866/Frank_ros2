#!/usr/bin/env python3
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class EETwistSweepPub(Node):
    def __init__(self, topic, frame, rate_hz,
                 min_x, max_x, period_x,
                 min_y, max_y, period_y):
        super().__init__('ee_twist_sweep_pub')
        self.pub = self.create_publisher(TwistStamped, topic, 10)

        self.frame = frame
        self.rate_hz = rate_hz

        # X triangular sweep
        self.min_x = min_x
        self.max_x = max_x
        self.period_x = period_x
        slope_x_per_sec = 2.0 * (self.max_x - self.min_x) / self.period_x
        self.dx = slope_x_per_sec / self.rate_hz
        self.x = self.min_x
        self.dir_x = 1.0

        # Y triangular sweep
        self.min_y = min_y
        self.max_y = max_y
        self.period_y = period_y
        slope_y_per_sec = 2.0 * (self.max_y - self.min_y) / self.period_y
        self.dy = slope_y_per_sec / self.rate_hz
        self.y = self.min_y
        self.dir_y = 1.0

        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_cb)

    def _sweep(self, value, d, direction, vmin, vmax):
        value += direction * d
        if value >= vmax:
            value = vmax
            direction = -1.0
        elif value <= vmin:
            value = vmin
            direction = 1.0
        return value, direction

    def timer_cb(self):
        # Update sweeps
        self.x, self.dir_x = self._sweep(self.x, self.dx, self.dir_x, self.min_x, self.max_x)
        self.y, self.dir_y = self._sweep(self.y, self.dy, self.dir_y, self.min_y, self.max_y)

        # Publish TwistStamped
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.twist.linear.x = float(self.x)
        msg.twist.linear.y = float(self.y)
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.pub.publish(msg)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/resolved_rate_controller/ee_twist')
    parser.add_argument('--frame', default='base_link')
    parser.add_argument('--rate', type=float, default=100.0)
    parser.add_argument('--min-x', type=float, default=-0.2)
    parser.add_argument('--max-x', type=float, default=0.2)
    parser.add_argument('--period', type=float, default=2.0)
    parser.add_argument('--min-y', type=float)
    parser.add_argument('--max-y', type=float)
    parser.add_argument('--period-y', type=float)
    args = parser.parse_args()

    # If Y args not provided, mirror X settings
    min_y = args.min_y if args.min_y is not None else args.min_x
    max_y = args.max_y if args.max_y is not None else args.max_x
    period_y = args.period_y if args.period_y is not None else args.period

    rclpy.init()
    node = EETwistSweepPub(
        topic=args.topic, frame=args.frame, rate_hz=args.rate,
        min_x=args.min_x, max_x=args.max_x, period_x=args.period,
        min_y=min_y, max_y=max_y, period_y=period_y
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()