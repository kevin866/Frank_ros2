# goal_from_base_offset.py (world-frame goal publisher)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalFromOffset(Node):
    def __init__(self):
        super().__init__('goal_from_offset')
        self.sub = self.create_subscription(PoseStamped, '/vrpn_mocap/RigidBody_1/pose', self.base_cb, 10)
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.offset_x = self.declare_parameter('offset_x', -1.0).get_parameter_value().double_value  # meters

    def base_cb(self, msg: PoseStamped):
        goal = PoseStamped()
        goal.header = msg.header           # frame_id = 'world'
        goal.pose = msg.pose
        goal.pose.position.x = msg.pose.position.x + self.offset_x
        # keep y, z, and orientation identical for now (simplest test)
        self.pub.publish(goal)

def main():
    rclpy.init()
    rclpy.spin(GoalFromOffset())
    rclpy.shutdown()
if __name__ == '__main__':
    main()
