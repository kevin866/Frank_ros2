#!/usr/bin/env python3
"""
ee_twist_oscillate.py

Sends +0.01 m/s in x for 2 seconds, then -0.01 m/s for 2 seconds, repeat.
Stops if any joint is near its limit in the direction of motion.
Sends zero twist on shutdown.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState


# Edit these to match your URDF joint limits
JOINT_LIMITS = {
    'joint_1': (-2.967, 2.967),
    'joint_2': (-1.745, 1.745),
    'joint_3': (-2.269, 2.269),
    'joint_4': (-3.054, 3.054),
    'joint_5': (-2.007, 2.007),
    'joint_6': (-2.967, 2.967),
}

SPEED        = 0.05   # m/s
PERIOD       = 2.0    # seconds per direction
JOINT_MARGIN = 0.1    # rad — stop this far from limit
RATE_HZ      = 50.0
TOPIC        = '/resolved_rate_controller/ee_twist'
FRAME_ID     = 'base_link'


class BackAndForthPublisher(Node):
    def __init__(self):
        super().__init__('ee_twist_oscillate')

        self.pub = self.create_publisher(TwistStamped, TOPIC, 10)

        self.joint_pos  = {}
        self.got_joints = False
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        self.t0    = None
        self.timer = self.create_timer(1.0 / RATE_HZ, self._tick)

        self.get_logger().info(
            f'BackAndForthPublisher ready → {TOPIC}  '
            f'{SPEED} m/s in x, {PERIOD}s per direction'
        )

    def _js_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_pos[name] = pos
        if not self.got_joints:
            self.got_joints = True
            self.get_logger().info('Joint states received — starting.')

    def _near_limit(self, vx: float) -> bool:
        for name, (lo, hi) in JOINT_LIMITS.items():
            pos = self.joint_pos.get(name)
            if pos is None:
                continue
            if vx > 0.0 and pos >= hi - JOINT_MARGIN:
                self.get_logger().warn(f'{name} near upper limit — holding')
                return True
            if vx < 0.0 and pos <= lo + JOINT_MARGIN:
                self.get_logger().warn(f'{name} near lower limit — holding')
                return True
        return False

    def _tick(self):
        now = self.get_clock().now()
        msg = TwistStamped()
        msg.header.stamp    = now.to_msg()
        msg.header.frame_id = FRAME_ID

        if not self.got_joints:
            self.pub.publish(msg)  # zero until joints arrive
            return

        if self.t0 is None:
            self.t0 = now

        t = (now - self.t0).nanoseconds * 1e-9

        # alternate direction every PERIOD seconds
        cycle = int(t / PERIOD)
        vx = SPEED if cycle % 2 == 0 else -SPEED

        if self._near_limit(vx):
            vx = 0.0

        msg.twist.linear.x = vx
        self.pub.publish(msg)

    def _send_zero(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = BackAndForthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._send_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()