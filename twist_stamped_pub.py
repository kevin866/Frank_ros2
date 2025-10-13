import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class TwistStampedPublisher(Node):
    def __init__(self):
        super().__init__('twist_stamped_pub')
        self.pub = self.create_publisher(TwistStamped, '/mecanum_controller/reference', 10)
        self.timer = self.create_timer(1/50, self.timer_callback)  # 50 Hz

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.y = 1.0
        msg.twist.angular.z = 0.0
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TwistStampedPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

