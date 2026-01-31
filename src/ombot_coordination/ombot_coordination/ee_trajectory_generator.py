#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy

@dataclass
class Quintic1D:
    a0: float; a1: float; a2: float; a3: float; a4: float; a5: float; T: float
    def eval(self, t):
        t = max(0.0, min(self.T, t))
        x  = self.a0 + self.a1*t + self.a2*t**2 + self.a3*t**3 + self.a4*t**4 + self.a5*t**5
        xd = self.a1 + 2*self.a2*t + 3*self.a3*t**2 + 4*self.a4*t**3 + 5*self.a5*t**4
        return x, xd

def make_quintic(p0, pf, T):
    dp = pf - p0
    return Quintic1D(
        a0=p0, a1=0.0, a2=0.0,
        a3=10*dp/T**3,
        a4=-15*dp/T**4,
        a5=6*dp/T**5,
        T=T
    )

class EETrajectoryGenerator(Node):
    def __init__(self):
        super().__init__("ee_trajectory_generator")

        self.declare_parameter("traj_T", 4.0)
        self.T = float(self.get_parameter("traj_T").value)

        self.ee: Optional[PoseStamped] = None
        self.goal: Optional[PoseStamped] = None

        # self.sub_ee   = self.create_subscription(PoseStamped, "/ee_pose", self.ee_cb, 10)
        # self.sub_goal = self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)

        sensor_qos = qos_profile_sensor_data  # BEST_EFFORT, VOLATILE

        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub_ee = self.create_subscription(
            PoseStamped, "/ee_pose", self.ee_cb, sensor_qos)

        self.sub_goal = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_cb, reliable_qos)

        self.pub_pose  = self.create_publisher(PoseStamped, "/ee_desired_pose", 10)
        self.pub_twist = self.create_publisher(TwistStamped, "/ee_desired_twist", 10)

        self.timer = self.create_timer(0.01, self.spin)

        self.active = False
        self.t = 0.0

    def ee_cb(self, msg): self.ee = msg
    def goal_cb(self, msg):
        self.goal = msg
        if self.ee is not None:
            self.start_traj()

    def start_traj(self):
        p0 = self.ee.pose.position
        pf = self.goal.pose.position
        self.traj_x = make_quintic(p0.x, pf.x, self.T)
        self.traj_y = make_quintic(p0.y, pf.y, self.T)
        self.traj_z = make_quintic(p0.z, pf.z, self.T)
        self.t = 0.0
        self.active = True
        self.get_logger().info("EE trajectory started")

    def spin(self):
        if not self.active:
            return

        self.t += 0.01

        x, xd = self.traj_x.eval(self.t)
        y, yd = self.traj_y.eval(self.t)
        z, zd = self.traj_z.eval(self.t)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.goal.header.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = self.goal.pose.orientation
        self.pub_pose.publish(pose)

        twist = TwistStamped()
        twist.header = pose.header
        twist.twist.linear.x = xd
        twist.twist.linear.y = yd
        twist.twist.linear.z = zd
        self.pub_twist.publish(twist)

def main():
    rclpy.init()
    rclpy.spin(EETrajectoryGenerator())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
