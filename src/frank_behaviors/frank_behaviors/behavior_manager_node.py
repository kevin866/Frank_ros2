# frank_behaviors/frank_behaviors/behavior_manager_node.py

from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class BehaviorManagerNode(Node):
    def __init__(self):
        super().__init__('behavior_manager_node')

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub_intent = self.create_subscription(
            String, '/frank/intent', self.intent_callback, 10
        )
        self.sub_face = self.create_subscription(
            String, '/vision/face_info', self.face_callback, 10
        )

        self.mode = 'idle'
        self.latest_face_center: Optional[int] = None
        self.image_width = 1280  # adjust if needed
        self.center_tolerance_px = 80

        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('BehaviorManagerNode started.')

    def intent_callback(self, msg: String):
        intent = msg.data.strip().lower()
        self.get_logger().info(f'Received intent: {intent}')

        if intent == 'stop':
            self.mode = 'idle'
            self.publish_stop()

        elif intent == 'find_face':
            self.mode = 'find_face'

        elif intent == 'track_face':
            self.mode = 'track_face'

        elif intent == 'go_home':
            self.mode = 'go_home'
            self.get_logger().info('TODO: hook this into your existing home behavior.')

        else:
            self.get_logger().warn(f'Unknown intent: {intent}')

    def face_callback(self, msg: String):
        data = msg.data.strip()

        if data == 'no_face':
            self.latest_face_center = None
            return

        if data.startswith('face_detected:'):
            try:
                center_str = data.split(':')[1]
                self.latest_face_center = int(center_str)
            except Exception:
                self.get_logger().warn(f'Could not parse face info: {data}')

    def control_loop(self):
        if self.mode == 'idle':
            return

        if self.mode == 'go_home':
            # Placeholder: stop for now.
            self.publish_stop()
            return

        if self.mode == 'find_face':
            if self.latest_face_center is None:
                twist = Twist()
                twist.angular.z = 0.2
                self.pub_cmd_vel.publish(twist)
            else:
                self.publish_stop()
                self.mode = 'idle'
                self.get_logger().info('Face found. Stopping.')
            return

        if self.mode == 'track_face':
            if self.latest_face_center is None:
                self.publish_stop()
                return

            image_center = self.image_width // 2
            error = self.latest_face_center - image_center

            twist = Twist()

            if abs(error) < self.center_tolerance_px:
                twist.angular.z = 0.0
            else:
                # Simple proportional turning
                k = 0.0015
                twist.angular.z = -k * error

                # Clamp
                if twist.angular.z > 0.3:
                    twist.angular.z = 0.3
                elif twist.angular.z < -0.3:
                    twist.angular.z = -0.3

            self.pub_cmd_vel.publish(twist)

    def publish_stop(self):
        twist = Twist()
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()