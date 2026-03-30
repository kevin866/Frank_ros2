# frank_vision/frank_vision/face_detector_node.py

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class FaceDetectorNode(Node):
    def __init__(self):
        super().__init__('face_detector_node')

        self.declare_parameter('image_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('display_debug', False)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.display_debug = self.get_parameter('display_debug').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.sub_image = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.pub_face = self.create_publisher(String, '/vision/face_info', 10)

        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )

        if self.face_cascade.empty():
            self.get_logger().error('Failed to load Haar cascade for face detection.')

        self.get_logger().info(f'FaceDetectorNode subscribed to: {image_topic}')

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge conversion failed: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(60, 60),
        )

        out = String()

        if len(faces) == 0:
            out.data = 'no_face'
            self.pub_face.publish(out)
        else:
            # Use the largest face
            largest = max(faces, key=lambda f: f[2] * f[3])
            x, y, w, h = largest
            cx = x + w // 2
            out.data = f'face_detected:{cx}'
            self.pub_face.publish(out)

            if self.display_debug:
                debug_frame = frame.copy()
                for (fx, fy, fw, fh) in faces:
                    cv2.rectangle(debug_frame, (fx, fy), (fx + fw, fy + fh), (0, 255, 0), 2)
                cv2.imshow('face_detector_debug', debug_frame)
                cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()