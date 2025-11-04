import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy

class EEPoseToBaseVelocity(Node):
    def __init__(self):
        super().__init__('ee_pose_to_base_velocity')

        # ---- QoS: try BEST_EFFORT first (common for streaming topics)
        # If your publisher is RELIABLE, set use_best_effort=False or switch the QoS below.
        self.declare_parameter('use_best_effort', True)
        use_best_effort = self.get_parameter('use_best_effort').get_parameter_value().bool_value

        if use_best_effort:
            ee_qos = qos_profile_sensor_data  # depth=10, BEST_EFFORT, KEEP_LAST
            self.get_logger().info("Subscribing to /ee_pose with BEST_EFFORT QoS.")
        else:
            ee_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            )
            self.get_logger().info("Subscribing to /ee_pose with RELIABLE QoS.")

        # Publisher → base controller
        self.pub = self.create_publisher(TwistStamped, '/mecanum_controller/reference', 10)

        # Subscriber → EE pose, with chosen QoS
        self.sub = self.create_subscription(PoseStamped, '/ee_pose', self.ee_pose_callback, ee_qos)

        # 50 Hz publish loop
        self.timer = self.create_timer(1/50, self.timer_callback)

        # State
        self.ee_pose = None
        self.ee_pose_init = None

        # Start slow: small gains
        self.kx = -10000
        self.ky = 10000
        self.kz = 10000

        # Velocity limits
        self.v_lin_max = 3.0
        self.v_ang_max = 3.0

        self.get_logger().info("EE Pose to Base Velocity node started.")

    def apply_deadzone(self, value, threshold):
        """Zero out small values within ±threshold."""
        if abs(value) < threshold:
            return 0.0
        return value


    def ee_pose_callback(self, msg: PoseStamped):
        if self.ee_pose_init is None:
            self.ee_pose_init = msg.pose
            self.get_logger().info("Initial EE pose recorded.")
        self.ee_pose = msg.pose

    def timer_callback(self):
        if self.ee_pose is None or self.ee_pose_init is None:
            return

        dx = self.ee_pose.position.x - self.ee_pose_init.position.x
        dy = self.ee_pose.position.y - self.ee_pose_init.position.y
        # dz = self.ee_pose.position.z - self.ee_pose_init.position.z  # not used yet

        deadzone_x = 0.01   # 1 cm dead zone
        deadzone_y = 0.005   # 5 mm dead zone
        dx = self.apply_deadzone(dx, deadzone_x)
        dy = self.apply_deadzone(dy, deadzone_y)

        vx = max(min(self.kx * dx,  self.v_lin_max), -self.v_lin_max)
        vy = max(min(self.ky * dy,  self.v_lin_max), -self.v_lin_max)
        wz = max(min(self.kz * dy,  self.v_ang_max), -self.v_ang_max)  # small yaw from lateral deflection

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = EEPoseToBaseVelocity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
