# base_p_on_x.py (drives base toward x_goal in world)
import rclpy, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped

class BasePX(Node):
    def __init__(self):
        super().__init__('base_p_on_x')
        # Use BEST_EFFORT + VOLATILE for sensor/mocap streams
        mocap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub_base = self.create_subscription(PoseStamped, '/vrpn_mocap/RigidBody_1/pose', self.base_cb, mocap_qos)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.pub_cmd  = self.create_publisher(TwistStamped, '/mecanum_controller/reference', 10)
        self.tol = self.declare_parameter('tol', 0.02).value        # stop within 2 cm
        self.min_v = self.declare_parameter('min_v', 0.5).value    # creep floor (m/s)

        self.kx = self.declare_parameter('kx', 0.8).get_parameter_value().double_value
        self.vmax = self.declare_parameter('vmax', 5.0).get_parameter_value().double_value
        self.flip_forward = self.declare_parameter('flip_forward', False).get_parameter_value().bool_value  # set True if fwd is inverted

        self.x = None; self.xg = None

    def base_cb(self, msg):
        self.x = msg.pose.position.x
        self.tick()

    def goal_cb(self, msg):
        self.xg = msg.pose.position.x
        self.tick()

    def tick(self):
        if self.x is None or self.xg is None: return
        ex = self.xg - self.x
        v = self.kx * ex

        if abs(ex) <= self.tol:
            v = 0.0
        else:
            if abs(v) < self.min_v:
                v = math.copysign(self.min_v, ex)
            v = max(-self.vmax, min(self.vmax, v))

        if self.flip_forward:
            v = -v               # quick one-line inversion if base forward is flipped
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()   # <-- add this
        ts.header.frame_id = 'world'                         # or 'base_link' (not critical here)
        ts.twist.linear.x = float(v)
        self.pub_cmd.publish(ts)

def main():
    rclpy.init()
    rclpy.spin(BasePX())
    rclpy.shutdown()
if __name__ == '__main__':
    main()
