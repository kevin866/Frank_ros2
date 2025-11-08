#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import qos_profile_sensor_data
class EETwistGotoOffset(Node):
    def __init__(self):
        super().__init__('ee_twist_goto_offset')

        # -------------------------------------------------
        # ✅ USER PARAMETERS (edit these)
        # -------------------------------------------------
        self.topic_cmd = '/resolved_rate_controller/ee_twist'
        self.topic_pose = '/ee_pose'
        self.frame_id = 'link_1'

        self.rate_hz = 100.0
        self.offset_x = 0.10     # desired offset in X (m)
        self.offset_y = 0.10      # desired offset in Y (m)

        self.tol = 0.01           # stop tolerance (m)
        self.kp = 1.0             # proportional gain
        self.vmax = 0.25          # max velocity (m/s)
        self.settle_cycles = 10   # consecutive cycles within tol before stopping
        self.zeros_on_stop = 5    # publish 0 velocity this many times before stopping
        # -------------------------------------------------

        self.pub = self.create_publisher(TwistStamped, self.topic_cmd, 10)
        # self.sub = self.create_subscription(PoseStamped, self.topic_pose, self.pose_cb, 10)
        self.sub = self.create_subscription(
                PoseStamped, self.topic_pose, self.pose_cb, qos_profile_sensor_data
            )
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self.timer_cb)

        # state
        self.ee_x = None
        self.ee_y = None
        self.goal_x = None
        self.goal_y = None
        self._settle_count = 0
        self._zeros_left = 0
        self._stopped = False

        self.get_logger().info(
            f"EE Twist Goto Offset started\n"
            f"Offset=({self.offset_x:+.3f}, {self.offset_y:+.3f}) [frame={self.frame_id}]"
        )

    def pose_cb(self, msg: PoseStamped):
        self.ee_x = msg.pose.position.x
        self.ee_y = msg.pose.position.y

        # Once we get the first pose, define our goal once
        if self.goal_x is None and self.goal_y is None:
            self.goal_x = self.ee_x + self.offset_x
            self.goal_y = self.ee_y + self.offset_y
            self.get_logger().info(
                f"Captured start pos (x={self.ee_x:.3f}, y={self.ee_y:.3f}) → "
                f"goal (x={self.goal_x:.3f}, y={self.goal_y:.3f})"
            )

    def publish_twist(self, vx, vy):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.pub.publish(msg)

    def timer_cb(self):
        # wait until we have a goal
        if self.goal_x is None or self._stopped:
            return

        if self._zeros_left > 0:
            self.publish_twist(0.0, 0.0)
            self._zeros_left -= 1
            if self._zeros_left == 0:
                self.get_logger().info("Goal reached — stopped publishing; controller will hold after timeout.")
                self.timer.cancel()
                self._stopped = True
            return

        ex = self.goal_x - self.ee_x
        ey = self.goal_y - self.ee_y
        err = math.hypot(ex, ey)

        if err < self.tol:
            self._settle_count += 1
        else:
            self._settle_count = 0

        if self._settle_count >= self.settle_cycles:
            self._zeros_left = self.zeros_on_stop
            return

        # proportional velocity control
        vx = self.kp * ex
        vy = self.kp * ey

        # saturate speed
        vnorm = math.hypot(vx, vy)
        if vnorm > self.vmax and vnorm > 1e-9:
            scale = self.vmax / vnorm
            vx *= scale
            vy *= scale

        self.publish_twist(vx, vy)


def main():
    rclpy.init()
    node = EETwistGotoOffset()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
