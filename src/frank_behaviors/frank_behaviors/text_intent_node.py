# frank_behaviors/frank_behaviors/text_intent_node.py

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TextIntentNode(Node):
    def __init__(self):
        super().__init__('text_intent_node')
        self.pub_intent = self.create_publisher(String, '/frank/intent', 10)

        self.get_logger().info('TextIntentNode started.')
        self.get_logger().info('Type commands: go_home, stop, find_face, track_face')
        self.get_logger().info('Press Ctrl+C to quit.')

        self._thread = threading.Thread(target=self._input_loop, daemon=True)
        self._thread.start()

    def _input_loop(self):
        while rclpy.ok():
            try:
                cmd = input('Enter intent> ').strip()
            except EOFError:
                break
            except Exception as e:
                self.get_logger().error(f'Input error: {e}')
                continue

            if not cmd:
                continue

            msg = String()
            msg.data = cmd
            self.pub_intent.publish(msg)
            self.get_logger().info(f'Published intent: {cmd}')


def main(args=None):
    rclpy.init(args=args)
    node = TextIntentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()