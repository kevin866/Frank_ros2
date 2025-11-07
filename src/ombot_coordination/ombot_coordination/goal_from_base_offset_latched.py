# goal_from_base_offset_latched.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool, Trigger

class GoalFromOffset(Node):
    def __init__(self):
        super().__init__('goal_from_offset')

        # Params
        self.offset_x = self.declare_parameter('offset_x', -1.0) \
                            .get_parameter_value().double_value  # meters
        self.rate_hz  = self.declare_parameter('publish_rate_hz', 20.0) \
                            .get_parameter_value().double_value
        # Modes: 'latch' or 'track'
        self.mode     = self.declare_parameter('mode', 'latch') \
                            .get_parameter_value().string_value.lower()

        # QoS for mocap (sensor-like)
        mocap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            PoseStamped, '/vrpn_mocap/RigidBody_1/pose', self.base_cb, mocap_qos
        )
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Services to control behavior
        self.srv_set_tracking = self.create_service(SetBool, 'set_tracking', self.set_tracking_cb)
        self.srv_reset        = self.create_service(Trigger,  'reset',        self.reset_cb)

        # Internal state
        self.last_mocap: PoseStamped | None = None
        self.latched_goal: PoseStamped | None = None

        # Republish at fixed rate so downstream nodes always see fresh stamps
        period = 1.0 / max(self.rate_hz, 1e-3)
        self.timer = self.create_timer(period, self.timer_cb)

        # Allow runtime param updates for offset_x/mode
        self.add_on_set_parameters_callback(self.on_param_set)

        self.get_logger().info(f"GoalFromOffset started in mode='{self.mode}', offset_x={self.offset_x}")

    def on_param_set(self, params):
        for p in params:
            if p.name == 'offset_x' and p.type_ == p.TYPE_DOUBLE:
                self.offset_x = p.value
                # If we’re latched, you can choose to re-latch on offset change; here we just update the latched goal if it exists.
                if self.latched_goal is not None and self.last_mocap is not None and self.mode == 'latch':
                    self.latched_goal.pose.position.x = self.last_mocap.pose.position.x + self.offset_x
            elif p.name == 'mode' and p.type_ == p.TYPE_STRING:
                m = p.value.lower()
                if m in ('latch', 'track'):
                    self.mode = m
                    self.get_logger().info(f"Mode set to '{self.mode}'")
                else:
                    return [rclpy.parameter.SetParametersResult(successful=False, reason="mode must be 'latch' or 'track'")]
        return [rclpy.parameter.SetParametersResult(successful=True)]

    def set_tracking_cb(self, req: SetBool.Request, res: SetBool.Response):
        self.mode = 'track' if req.data else 'latch'
        res.success = True
        res.message = f"Mode set to '{self.mode}'"
        self.get_logger().info(res.message)
        return res

    def reset_cb(self, _req: Trigger.Request, res: Trigger.Response):
        self.latched_goal = None
        res.success = True
        res.message = "Latched goal cleared; next mocap message will (re)latch in 'latch' mode."
        self.get_logger().info(res.message)
        return res

    def base_cb(self, msg: PoseStamped):
        self.last_mocap = msg

        if self.mode == 'track':
            # Always follow current pose + offset
            goal = PoseStamped()
            goal.header.frame_id = 'world'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose = msg.pose
            goal.pose.position.x = msg.pose.position.x + self.offset_x
            self._publish(goal)

        elif self.mode == 'latch':
            # If we haven't latched yet, latch once from the first mocap message after reset
            if self.latched_goal is None:
                goal = PoseStamped()
                goal.header.frame_id = 'world'
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose = msg.pose
                goal.pose.position.x = msg.pose.position.x + self.offset_x
                self.latched_goal = goal
                self._publish(goal)
                self.get_logger().info("Latched goal from mocap + offset")

    def timer_cb(self):
        # Periodically re-publish the current goal so consumers see fresh timestamps
        goal_to_pub = None
        if self.mode == 'track':
            # Only republish if we have a mocap reading to base on
            if self.last_mocap is not None:
                goal = PoseStamped()
                goal.header.frame_id = 'world'
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose = self.last_mocap.pose
                goal.pose.position.x = self.last_mocap.pose.position.x + self.offset_x
                goal_to_pub = goal
        else:  # 'latch'
            if self.latched_goal is not None:
                goal = PoseStamped()
                goal.header.frame_id = 'world'
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose = self.latched_goal.pose
                goal_to_pub = goal

        if goal_to_pub is not None:
            self._publish(goal_to_pub)

    def _publish(self, goal: PoseStamped):
        self.pub.publish(goal)

def main():
    rclpy.init()
    rclpy.spin(GoalFromOffset())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
