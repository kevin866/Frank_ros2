import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult
import numpy as np


class GoalFromOffset(Node):
    def __init__(self):
        super().__init__('goal_from_offset')

        # Parameters
        self.xyz_offset = np.array(
            self.declare_parameter('offset_xyz', [0.0, 0.0, 0.0]).value,
            dtype=float,
        )
        self.rate_hz = self.declare_parameter('publish_rate_hz', 20.0).value

        # Fixed rotation: world -> base
        self.R_wb = np.array([
            [1.0,  0.0,  0.0],
            [0.0,  0.0, 1.0],
            [0.0,  1.0,  0.0],
        ])
        # base -> world
        self.R_bw = self.R_wb.T

        # QoS
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscribers
        self.sub_base = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/RigidBody_1/pose',
            self.base_cb,
            qos,
        )
        self.sub_ee = self.create_subscription(
            PoseStamped,
            '/ee_pose',
            self.ee_cb,
            qos,
        )

        # Publisher
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Reset service
        self.srv_reset = self.create_service(Trigger, 'reset', self.reset_cb)

        # State
        self.last_base: PoseStamped | None = None
        self.last_ee:   PoseStamped | None = None
        self.latched_goal: PoseStamped | None = None

        # Timer for republishing goal with fresh timestamps
        period = 1.0 / max(self.rate_hz, 1e-3)
        self.timer = self.create_timer(period, self.timer_cb)

        # Allow parameter updates
        self.add_on_set_parameters_callback(self.on_param_set)

        self.get_logger().info(
            f"GoalFromOffset (latch-only, offset in base frame) started, "
            f"offset_xyz={self.xyz_offset.tolist()}"
        )

    # ------------------- parameter callback -------------------

    def on_param_set(self, params):
        for p in params:
            if p.name == 'offset_xyz' and p.type_ == p.TYPE_DOUBLE_ARRAY:
                self.xyz_offset = np.array(p.value, dtype=float)

                # If we've already latched and still have a base pose, update latched position
                if self.latched_goal is not None and self.last_base is not None:
                    p_b_w = np.array([
                        self.last_base.pose.position.x,
                        self.last_base.pose.position.y,
                        self.last_base.pose.position.z,
                    ])
                    off_w = self.R_bw @ self.xyz_offset
                    p_goal_w = self.R_bw @ p_b_w + off_w

                    self.latched_goal.pose.position.x = float(p_goal_w[0])
                    self.latched_goal.pose.position.y = float(p_goal_w[1])
                    self.latched_goal.pose.position.z = float(p_goal_w[2])

        return SetParametersResult(successful=True)

    # ------------------- reset service -------------------

    def reset_cb(self, _req, res):
        self.latched_goal = None
        res.success = True
        res.message = "Latched goal cleared. Next base+ee update will latch a new goal."
        self.get_logger().info(res.message)
        return res

    # ------------------- subscribers -------------------

    def ee_cb(self, msg: PoseStamped):
        self.last_ee = msg

    def base_cb(self, msg: PoseStamped):
        self.last_base = msg

        # Only latch once, when both base and ee are available
        if self.latched_goal is None and self.last_ee is not None:
            goal = self.build_goal()
            if goal is not None:
                self.latched_goal = goal
                self.pub.publish(goal)
                self.get_logger().info("Latched goal (offset in base frame, pose in base frame)")

    # ------------------- helpers -------------------

    def build_goal(self) -> PoseStamped | None:
        if self.last_base is None or self.last_ee is None:
            return None

        base = self.last_base
        ee   = self.last_ee

        # Base position in world
        p_b_w = np.array([
            base.pose.position.x,
            base.pose.position.y,
            base.pose.position.z,
        ])

        # Offset is given in base frame; rotate to world
        off_b = self.xyz_offset

        p_ee= np.array([ee.pose.position.x,
                   ee.pose.position.y,
                   ee.pose.position.z])

        p_goal = self.R_bw @ p_b_w + off_b + p_ee

        goal = PoseStamped()
        goal.header.frame_id = 'base'
        goal.header.stamp = self.get_clock().now().to_msg()

        # Position = base + rotated offset
        goal.pose.position.x = float(p_goal[0])
        goal.pose.position.y = float(p_goal[1])
        goal.pose.position.z = float(p_goal[2])

        # Orientation: just use EE orientation (assumed already correct for your stack)
        goal.pose.orientation = ee.pose.orientation

        return goal

    # ------------------- timer republish -------------------

    def timer_cb(self):
        if self.latched_goal is None:
            return

        goal = PoseStamped()
        goal.header.frame_id = 'base'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose = self.latched_goal.pose

        self.pub.publish(goal)


def main():
    rclpy.init()
    rclpy.spin(GoalFromOffset())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
