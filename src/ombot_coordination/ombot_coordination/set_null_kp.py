# set_null_kp.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
class SetNullKP(Node):
    def __init__(self):
        super().__init__('set_null_kp')
        self.pub = self.create_publisher(Float64, '/resolved_rate_controller/null_kp', 10)
        self.timer = self.create_timer(0.1, self.tick)
    def tick(self):
        self.pub.publish(Float64(data=0.3))   # higher = faster retract toward q_home
def main():
    rclpy.init(); rclpy.spin(SetNullKP()); rclpy.shutdown()
if __name__ == '__main__': main()
