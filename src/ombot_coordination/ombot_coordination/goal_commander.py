#!/usr/bin/env python3
from __future__ import annotations
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))

class GoalCommander(Node):
    def __init__(self):
        super().__init__("goal_commander")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("topic", "/goal_pose")
        self.declare_parameter("rate_hz", 5.0)   # keep publishing so late subscribers still get it

        self.world_frame = self.get_parameter("world_frame").value
        self.topic = self.get_parameter("topic").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.pub = self.create_publisher(PoseStamped, self.topic, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.have_goal = False
        self.goal = PoseStamped()
        self.goal.header.frame_id = self.world_frame

        self.base_pub = self.create_publisher(PoseStamped, "/base_goal_pose", 10)


        self.get_logger().info(
            "GoalCommander ready.\n"
            "Use: ros2 param set /goal_commander goal \"x y z yaw\"\n"
            "Example: ros2 param set /goal_commander goal \"0.8 0.2 0.5 0.0\""
        )

        # string param for goal input
        self.declare_parameter("goal", "")

    def on_timer(self):
        s = self.get_parameter("goal").value
        if isinstance(s, str) and s.strip():
            try:
                x, y, z, yaw = [float(v) for v in s.split()]

                qx, qy, qz, qw = yaw_to_quat(yaw)

                self.goal.header.stamp = self.get_clock().now().to_msg()
                # self.goal.pose.position.x = x - 0.22
                # self.goal.pose.position.y = y - 0.32
                self.goal.pose.position.x = x 
                self.goal.pose.position.y = -y 
                self.goal.pose.position.z = z
                self.goal.pose.orientation.x = 0.0
                self.goal.pose.orientation.y = 0.0
                self.goal.pose.orientation.z = 0.0
                self.goal.pose.orientation.w = 0.0

                self.have_goal = True
            except Exception as e:
                self.get_logger().warn(f"Bad goal param '{s}': {e}")

        if self.have_goal:
            now = self.get_clock().now().to_msg()

            # publish EE goal (unchanged)
            self.goal.header.stamp = now
            self.pub.publish(self.goal)

            # -----------------------------
            # base goal = EE goal shifted
            # -----------------------------
            base_goal = PoseStamped()
            base_goal.header.stamp = now
            base_goal.header.frame_id = self.world_frame

            base_goal.pose.position.x = self.goal.pose.position.x - 0.2
            base_goal.pose.position.y = self.goal.pose.position.y
            base_goal.pose.position.z = self.goal.pose.position.z

            base_goal.pose.orientation = self.goal.pose.orientation

            # self.base_pub.publish(base_goal)


def main():
    rclpy.init()
    node = GoalCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
