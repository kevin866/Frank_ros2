import cv2
from cv_bridge import CvBridge
import os
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D

from ament_index_python.packages import get_package_share_directory


# def _check_cuda() -> bool:
#     try:
#         count = cv2.cuda.getCudaEnabledDeviceCount()
#         return count > 0
#     except Exception:
#         return False


class FaceDetectorNode(Node):
    def __init__(self):
        super().__init__('face_detector_node')

        self.declare_parameter('image_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('display_debug', True)
        self.declare_parameter('score_threshold', 0.60)   # lowered: catches far/small faces
        self.declare_parameter('nms_threshold', 0.3)
        self.declare_parameter('top_k', 5000)
        self.declare_parameter('alpha', 0.25)             # smoother EMA
        self.declare_parameter('multiscale', True)        # second pass zoom for far faces
        self.declare_parameter('multiscale_crop_factor', 0.5)  # center crop fraction

        image_topic      = self.get_parameter('image_topic').get_parameter_value().string_value
        self.display_debug     = self.get_parameter('display_debug').get_parameter_value().bool_value
        self.score_threshold   = self.get_parameter('score_threshold').get_parameter_value().double_value
        self.nms_threshold     = self.get_parameter('nms_threshold').get_parameter_value().double_value
        self.top_k             = self.get_parameter('top_k').get_parameter_value().integer_value
        self.alpha             = self.get_parameter('alpha').get_parameter_value().double_value
        self.multiscale        = self.get_parameter('multiscale').get_parameter_value().bool_value
        self.crop_factor       = self.get_parameter('multiscale_crop_factor').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.sub_image = self.create_subscription(Image, image_topic, self.image_callback, 10)

        # Rich structured detections
        self.pub_detections = self.create_publisher(Detection2DArray, '/vision/face_detections', 10)
        # Legacy string topic kept for backward compatibility
        self.pub_face  = self.create_publisher(String, '/vision/face_info', 10)
        self.pub_debug = self.create_publisher(Image,  '/vision/debug_image', 10)

        pkg_share  = get_package_share_directory('frank_vision')
        model_path = os.path.join(pkg_share, 'models', 'face_detection_yunet_2022mar.onnx')

        if not os.path.exists(model_path):
            self.get_logger().error(f'YuNet model not found: {model_path}')
            raise FileNotFoundError(model_path)

        # ── CUDA auto-detection ──────────────────────────────────────────────
        # self.use_cuda = _check_cuda()
        # backend = cv2.dnn.DNN_BACKEND_CUDA   if self.use_cuda else cv2.dnn.DNN_BACKEND_OPENCV
        # target  = cv2.dnn.DNN_TARGET_CUDA    if self.use_cuda else cv2.dnn.DNN_TARGET_CPU
        # self.get_logger().info(f'{"CUDA detected — using GPU" if self.use_cuda else "No CUDA — using CPU"}')
        self.DETECTOR_INPUT_SIZE = (640, 640)  # put this as a class constant at the top of __init__
 
        def _make_detector(input_size=self.DETECTOR_INPUT_SIZE):
            det = cv2.FaceDetectorYN.create(
                model=model_path,
                config='',
                input_size=input_size,
                score_threshold=self.score_threshold,
                nms_threshold=self.nms_threshold,
                top_k=self.top_k,
            )
            return det

        self.detector       = _make_detector()
        self.detector_zoom  = _make_detector() if self.multiscale else None

        self.filtered_cx = None
        self.filtered_cy = None

        self.get_logger().info(f'Subscribed to: {image_topic}')
        self.get_logger().info(f'Multiscale: {self.multiscale}, score_threshold: {self.score_threshold}')


    # ── helpers ──────────────────────────────────────────────────────────────

    # AFTER — resizes frame to fixed 640x640, avoids the eltwise_layer bug

    def _detect(self, detector, frame) -> list:
        h, w = frame.shape[:2]
        resized = cv2.resize(frame, self.DETECTOR_INPUT_SIZE)
        _, faces = detector.detect(resized)
        if faces is None:
            return []

        # scale bounding boxes and landmarks back to original frame size
        sx = w / self.DETECTOR_INPUT_SIZE[0]
        sy = h / self.DETECTOR_INPUT_SIZE[1]
        scaled = []
        for f in faces:
            fc = f.copy()
            fc[0] = f[0] * sx  # x
            fc[1] = f[1] * sy  # y
            fc[2] = f[2] * sx  # w
            fc[3] = f[3] * sy  # h
            for i in range(5):
                fc[4 + 2*i] = f[4 + 2*i] * sx
                fc[5 + 2*i] = f[5 + 2*i] * sy
            scaled.append(fc)
        return scaled
   
    def _multiscale_detect(self, frame) -> list:
        """
        Second-pass detection on a centre crop then re-maps coords back to
        the full frame. Helps a lot for faces that are small / far away.
        """
        h, w = frame.shape[:2]
        cf = self.crop_factor
        x0 = int(w * (1 - cf) / 2)
        y0 = int(h * (1 - cf) / 2)
        x1 = int(w * (1 + cf) / 2)
        y1 = int(h * (1 + cf) / 2)

        crop = frame[y0:y1, x0:x1]
        crop_resized = cv2.resize(crop, (w, h))   # upscale crop to full res

        faces_crop = self._detect(self.detector_zoom, crop_resized)

        # remap coordinates from upscaled crop space → original frame space
        sx = (x1 - x0) / w
        sy = (y1 - y0) / h
        remapped = []
        for f in faces_crop:
            fc = f.copy()
            fc[0] = f[0] * sx + x0   # bbox x
            fc[1] = f[1] * sy + y0   # bbox y
            fc[2] = f[2] * sx         # bbox w
            fc[3] = f[3] * sy         # bbox h
            for i in range(5):        # landmarks
                fc[4 + 2*i]   = f[4 + 2*i]   * sx + x0
                fc[5 + 2*i]   = f[5 + 2*i]   * sy + y0
            remapped.append(fc)
        return remapped

    def _merge_detections(self, faces_full, faces_zoom) -> list:
        """
        Combine full-frame and zoom detections, then apply a simple
        IoU-based deduplication so we don't double-count the same face.
        """
        all_faces = faces_full + faces_zoom
        if len(all_faces) == 0:
            return []

        import numpy as np

        def iou(a, b):
            ax1, ay1, aw, ah = a[0], a[1], a[2], a[3]
            bx1, by1, bw, bh = b[0], b[1], b[2], b[3]
            ax2, ay2 = ax1 + aw, ay1 + ah
            bx2, by2 = bx1 + bw, by1 + bh
            ix1, iy1 = max(ax1, bx1), max(ay1, by1)
            ix2, iy2 = min(ax2, bx2), min(ay2, by2)
            inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
            union = aw * ah + bw * bh - inter
            return inter / union if union > 0 else 0.0

        keep = []
        used = [False] * len(all_faces)
        for i, fi in enumerate(all_faces):
            if used[i]:
                continue
            best = fi
            for j, fj in enumerate(all_faces):
                if i == j or used[j]:
                    continue
                if iou(fi, fj) > 0.4:
                    # keep the higher-scoring detection
                    best = fi if fi[14] >= fj[14] else fj
                    used[j] = True
            keep.append(best)
            used[i] = True
        return keep

    # ── main callback ─────────────────────────────────────────────────────────

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        faces_full = self._detect(self.detector, frame)

        if self.multiscale:
            faces_zoom = self._multiscale_detect(frame)
            faces = self._merge_detections(faces_full, faces_zoom)
        else:
            faces = faces_full

        # ── publish Detection2DArray ──────────────────────────────────────
        det_array = Detection2DArray()
        det_array.header = msg.header

        for face in faces:
            d = Detection2D()
            d.header = msg.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'face'
            hyp.hypothesis.score    = float(face[14])
            d.results.append(hyp)
            d.bbox.center.position.x = float(face[0] + face[2] / 2)
            d.bbox.center.position.y = float(face[1] + face[3] / 2)
            d.bbox.size_x = float(face[2])
            d.bbox.size_y = float(face[3])
            det_array.detections.append(d)

        self.pub_detections.publish(det_array)

        # ── legacy String publish (largest face only) ─────────────────────
        str_msg = String()
        if not faces:
            str_msg.data = 'no_face'
            self.pub_face.publish(str_msg)
        else:
            largest = max(faces, key=lambda f: f[2] * f[3])
            cx = int(largest[0] + largest[2] / 2)
            cy = int(largest[1] + largest[3] / 2)

            if self.filtered_cx is None:
                self.filtered_cx, self.filtered_cy = cx, cy
            else:
                self.filtered_cx = int(self.alpha * cx + (1 - self.alpha) * self.filtered_cx)
                self.filtered_cy = int(self.alpha * cy + (1 - self.alpha) * self.filtered_cy)

            str_msg.data = f'face_detected:{self.filtered_cx},{self.filtered_cy}'
            self.pub_face.publish(str_msg)

        # ── debug image ───────────────────────────────────────────────────
        if self.display_debug:
            debug = frame.copy()
            h, w = debug.shape[:2]

            for face in faces:
                fx, fy, fw, fh = map(int, face[0:4])
                score = float(face[14])
                cv2.rectangle(debug, (fx, fy), (fx+fw, fy+fh), (0, 255, 0), 2)
                cv2.putText(debug, f'{score:.2f}', (fx, max(fy-8, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                for i in range(5):
                    lx = int(face[4 + 2*i])
                    ly = int(face[5 + 2*i])
                    cv2.circle(debug, (lx, ly), 2, (255, 0, 0), -1)

            if self.filtered_cx is not None:
                cv2.circle(debug, (self.filtered_cx, self.filtered_cy), 5, (0, 0, 255), -1)

            # draw crop region
            if self.multiscale:
                cf = self.crop_factor
                x0 = int(w * (1 - cf) / 2)
                y0 = int(h * (1 - cf) / 2)
                x1 = int(w * (1 + cf) / 2)
                y1 = int(h * (1 + cf) / 2)
                cv2.rectangle(debug, (x0, y0), (x1, y1), (255, 255, 0), 1)

            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()