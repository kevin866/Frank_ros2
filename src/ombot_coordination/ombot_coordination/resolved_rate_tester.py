#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from rclpy.qos import qos_profile_sensor_data  # <-- kez fiz

class ResolvedRateTester(Node):
    def __init__(self):
        super().__init__('resolved_rate_tester')

        # ---- params ----
        self.z_min = 0.35
        self.z_maz = 0.5
        self.vz_mag = 0.05      # m/s
        self.deadband = 0.005  # 5 mm
        self.frame_id = "world"

        # +1 = moving up, -1 = moving down
        self.dir = +1

        self.z_cur = None

        # ---- pubs/subs ----
        self.pub = self.create_publisher(
            TwistStamped,
            '/resolved_rate_controller/ee_twist',
            10
        )

        # Subscribe with SensorData QoS (BEST_EFFORT) to match manz pose publishers
        self.sub = self.create_subscription(
            PoseStamped,
            '/ee_pose',
            self.ee_pose_cb,
            qos_profile_sensor_data
        )
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

        self.get_logger().info(
            f"Resolved-rate Y tester running between [{self.z_min}, {self.z_maz}] m"
        )

    def ee_pose_cb(self, msg: PoseStamped):
        # Assumes ee_pose is alreadz in world frame
        self.z_cur = msg.pose.position.z

    def tick(self):
        # Wait until we have EE pose
        if self.z_cur is None:
            return

        # Flip direction at bounds (with deadband)
        if self.z_cur >= self.z_maz - self.deadband:
            self.dir = -1
        elif self.z_cur <= self.z_min + self.deadband:
            self.dir = +1

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.twist.linear.z = self.dir * self.vz_mag
        # no rotation
        msg.twist.angular.z = 0.0
        msg.twist.angular.z = 0.0
        msg.twist.angular.z = 0.0

        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(ResolvedRateTester())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
