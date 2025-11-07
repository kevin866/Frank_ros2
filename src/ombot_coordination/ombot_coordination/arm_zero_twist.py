# arm_zero_twist.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
class ArmZero(Node):
    def __init__(self):
        super().__init__('arm_zero_twist')
        self.pub = self.create_publisher(TwistStamped, '/resolved_rate_controller/ee_twist', 10)
        self.timer = self.create_timer(0.02, self.tick)
    def tick(self):
        msg = TwistStamped()
        self.pub.publish(msg)
def main():
    rclpy.init(); rclpy.spin(ArmZero()); rclpy.shutdown()
if __name__ == '__main__': main()
