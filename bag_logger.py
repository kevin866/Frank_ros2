#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import csv

class CSVLogger(Node):
    def __init__(self):
        super().__init__('csv_logger')
        self.start_time = self.get_clock().now()

        # Prepare CSV files
        self.files = {
            '/mecanum_controller/reference': open('mecanum_controller_reference.csv', 'w', newline=''),
            '/resolved_rate_controller/ee_twist': open('resolved_rate_controller_ee_twist.csv', 'w', newline=''),
            '/goal_pose': open('goal_pose.csv', 'w', newline=''),
            '/vrpn_mocap/RigidBody_1/pose': open('vrpn_mocap_RigidBody_1_pose.csv', 'w', newline='')
        }
        self.writers = {topic: csv.writer(f) for topic, f in self.files.items()}

        # Write headers
        for topic, writer in self.writers.items():
            if 'Twist' in topic or 'twist' in topic:
                writer.writerow(['t', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz'])
            else:
                writer.writerow(['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

        # Subscriptions — use lambdas to pass the topic name explicitly
        self.create_subscription(
            TwistStamped,
            '/mecanum_controller/reference',
            lambda msg, topic='/mecanum_controller/reference': self.twist_cb(msg, topic),
            10)
        self.create_subscription(
            TwistStamped,
            '/resolved_rate_controller/ee_twist',
            lambda msg, topic='/resolved_rate_controller/ee_twist': self.twist_cb(msg, topic),
            10)
        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            lambda msg, topic='/goal_pose': self.pose_cb(msg, topic),
            10)
        self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/RigidBody_1/pose',
            lambda msg, topic='/vrpn_mocap/RigidBody_1/pose': self.pose_cb(msg, topic),
            10)

        self.get_logger().info("CSV Logger started — waiting for topics...")

    def time_now(self):
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def twist_cb(self, msg: TwistStamped, topic: str):
        t = self.time_now()
        l, a = msg.twist.linear, msg.twist.angular
        self.writers[topic].writerow([t, l.x, l.y, l.z, a.x, a.y, a.z])

    def pose_cb(self, msg: PoseStamped, topic: str):
        t = self.time_now()
        p, q = msg.pose.position, msg.pose.orientation
        self.writers[topic].writerow([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    def destroy_node(self):
        for f in self.files.values():
            f.close()
        self.get_logger().info("CSV files closed.")
        super().destroy_node()

def main():
    rclpy.init()
    node = CSVLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
