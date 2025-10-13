#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

HOME = [-0.048001135752655535, -0.4151982449018653, -0.007936554979054274,
        0.5437359966568645, -0.09646544584951455, 1.587365319160496]

NAMES = ['joint_1','joint_2','joint_4','joint_5','joint_3','joint_6']

class SendTraj(Node):
    def __init__(self):
        super().__init__('send_traj')
        self.pub = self.create_publisher(JointTrajectory, '/joint_impedance_controller/trajectory', 1)

        # Goal 1: go to HOME in 3s
        msg1 = JointTrajectory()
        msg1.joint_names = NAMES
        pt1 = JointTrajectoryPoint()
        pt1.positions = HOME
        pt1.velocities = [0.0]*6
        pt1.accelerations = [0.0]*6
        pt1.time_from_start.sec = 3
        msg1.points = [pt1]

        # Goal 2: small offset in 2s (so you see motion even if already at home)
        off = HOME.copy()
        off[1] += 0.20   # joint_2 +0.2 rad
        off[3] -= 0.15   # joint_5 -0.15 rad
        off[4] -= 0.05   # joint_3 -0.05 rad
        msg2 = JointTrajectory()
        msg2.joint_names = NAMES
        pt2 = JointTrajectoryPoint()
        pt2.positions = off
        pt2.velocities = [0.0]*6
        pt2.accelerations = [0.0]*6
        pt2.time_from_start.sec = 2
        msg2.points = [pt2]

        # Publish both with a short delay
        self.timer = self.create_timer(0.5, lambda: self.publish_two(msg1, msg2))

    def publish_two(self, msg1, msg2):
        self.pub.publish(msg1)
        self.get_logger().info('Sent trajectory to HOME (3s).')
        def second():
            self.pub.publish(msg2)
            self.get_logger().info('Sent small offset move (2s).')
        self.create_timer(0.5, second)  # send second shortly after first
        self.timer.cancel()

def main():
    rclpy.init()
    node = SendTraj()
    rclpy.spin_once(node, timeout_sec=1.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# Note: run this after starting joint_impedance_controller.launch.py