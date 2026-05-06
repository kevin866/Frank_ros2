import cv2
from cv_bridge import CvBridge
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D

from ament_index_python.packages import get_package_share_directory


class FaceDetectorNode(Node):
    def __init__(self):
        super().__init__('face_detector_node')

        self.declare_parameter('image_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('display_debug', True)
        self.declare_parameter('score_threshold', 0.60)        # loose net — catches far/small faces
        self.declare_parameter('nms_threshold', 0.3)
        self.declare_parameter('top_k', 5000)
        self.declare_parameter('alpha', 0.25)                  # EMA smoothing for published center
        self.declare_parameter('multiscale', True)
        self.declare_parameter('multiscale_crop_factor', 0.5)
        # ── quality / confirmation params ─────────────────────────────────────
        self.declare_parameter('confirm_threshold', 0.85)      # score gate for confirmation
        self.declare_parameter('confirm_frames', 4)            # consecutive frames to confirm
        self.declare_parameter('drop_frames', 2)               # consecutive misses before drop
        self.declare_parameter('detect_interval', 5)           # run detector every N frames; tracker fills gaps

        image_topic            = self.get_parameter('image_topic').get_parameter_value().string_value
        self.display_debug     = self.get_parameter('display_debug').get_parameter_value().bool_value
        self.score_threshold   = self.get_parameter('score_threshold').get_parameter_value().double_value
        self.nms_threshold     = self.get_parameter('nms_threshold').get_parameter_value().double_value
        self.top_k             = self.get_parameter('top_k').get_parameter_value().integer_value
        self.alpha             = self.get_parameter('alpha').get_parameter_value().double_value
        self.multiscale        = self.get_parameter('multiscale').get_parameter_value().bool_value
        self.crop_factor       = self.get_parameter('multiscale_crop_factor').get_parameter_value().double_value
        self.confirm_threshold = self.get_parameter('confirm_threshold').get_parameter_value().double_value
        self.confirm_frames    = self.get_parameter('confirm_frames').get_parameter_value().integer_value
        self.drop_frames       = self.get_parameter('drop_frames').get_parameter_value().integer_value
        self.detect_interval   = self.get_parameter('detect_interval').get_parameter_value().integer_value

        self.bridge    = CvBridge()
        self.sub_image = self.create_subscription(Image, image_topic, self.image_callback, 10)

        self.pub_detections = self.create_publisher(Detection2DArray, '/vision/face_detections', 10)
        self.pub_face       = self.create_publisher(String, '/vision/face_info', 10)
        self.pub_debug      = self.create_publisher(Image,  '/vision/debug_image', 10)

        pkg_share  = get_package_share_directory('frank_vision')
        model_path = os.path.join(pkg_share, 'models', 'face_detection_yunet_2022mar.onnx')

        if not os.path.exists(model_path):
            self.get_logger().error(f'YuNet model not found: {model_path}')
            raise FileNotFoundError(model_path)

        self.DETECTOR_INPUT_SIZE = (640, 640)

        def _make_detector(input_size=self.DETECTOR_INPUT_SIZE):
            return cv2.FaceDetectorYN.create(
                model=model_path,
                config='',
                input_size=input_size,
                score_threshold=self.score_threshold,
                nms_threshold=self.nms_threshold,
                top_k=self.top_k,
            )

        self.detector      = _make_detector()
        self.detector_zoom = _make_detector() if self.multiscale else None

        # ── EMA state ────────────────────────────────────────────────────────
        self.filtered_cx: Optional[int] = None
        self.filtered_cy: Optional[int] = None

        # ── frame-based confirmation state ────────────────────────────────────
        self.hit_count:      int  = 0
        self.miss_count:     int  = 0
        self.confirmed_face: bool = False

        # ── CSRT tracker ─────────────────────────────────────────────────────
        # tracker fills in position between detector runs so a single missed
        # detection frame doesn't immediately affect the output
        self.tracker:         Optional[cv2.TrackerCSRT] = None
        self.tracker_bbox:    Optional[tuple]           = None   # (x, y, w, h) in frame coords
        self.frame_count:     int                       = 0

        self.get_logger().info(f'Subscribed to: {image_topic}')
        self.get_logger().info(
            f'Multiscale: {self.multiscale}  score_threshold: {self.score_threshold}  '
            f'confirm_threshold: {self.confirm_threshold}  confirm_frames: {self.confirm_frames}'
        )

    # ── detection helpers ─────────────────────────────────────────────────────

    def _detect(self, detector, frame) -> list:
        h, w = frame.shape[:2]
        resized = cv2.resize(frame, self.DETECTOR_INPUT_SIZE)
        _, faces = detector.detect(resized)
        if faces is None:
            return []

        sx = w / self.DETECTOR_INPUT_SIZE[0]
        sy = h / self.DETECTOR_INPUT_SIZE[1]
        scaled = []
        for f in faces:
            fc = f.copy()
            fc[0] = f[0] * sx
            fc[1] = f[1] * sy
            fc[2] = f[2] * sx
            fc[3] = f[3] * sy
            for i in range(5):
                fc[4 + 2*i] = f[4 + 2*i] * sx
                fc[5 + 2*i] = f[5 + 2*i] * sy
            scaled.append(fc)
        return scaled

    def _multiscale_detect(self, frame) -> list:
        h, w = frame.shape[:2]
        cf = self.crop_factor
        x0 = int(w * (1 - cf) / 2)
        y0 = int(h * (1 - cf) / 2)
        x1 = int(w * (1 + cf) / 2)
        y1 = int(h * (1 + cf) / 2)

        crop         = frame[y0:y1, x0:x1]
        crop_resized = cv2.resize(crop, (w, h))
        faces_crop   = self._detect(self.detector_zoom, crop_resized)

        sx = (x1 - x0) / w
        sy = (y1 - y0) / h
        remapped = []
        for f in faces_crop:
            fc = f.copy()
            fc[0] = f[0] * sx + x0
            fc[1] = f[1] * sy + y0
            fc[2] = f[2] * sx
            fc[3] = f[3] * sy
            for i in range(5):
                fc[4 + 2*i] = f[4 + 2*i] * sx + x0
                fc[5 + 2*i] = f[5 + 2*i] * sy + y0
            remapped.append(fc)
        return remapped

    def _merge_detections(self, faces_full, faces_zoom) -> list:
        all_faces = faces_full + faces_zoom
        if not all_faces:
            return []

        def iou(a, b):
            ax2, ay2 = a[0] + a[2], a[1] + a[3]
            bx2, by2 = b[0] + b[2], b[1] + b[3]
            ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
            ix2, iy2 = min(ax2, bx2),  min(ay2, by2)
            inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
            union = a[2]*a[3] + b[2]*b[3] - inter
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
                    best = fi if fi[14] >= fj[14] else fj
                    used[j] = True
            keep.append(best)
            used[i] = True
        return keep

    # ── quality filters ───────────────────────────────────────────────────────

    def _landmarks_plausible(self, face) -> bool:
        """Reject detections whose landmark geometry is anatomically implausible."""
        left_eye  = (face[4], face[5])
        right_eye = (face[6], face[7])
        nose      = (face[8], face[9])

        # nose should be below both eyes
        if nose[1] < left_eye[1] or nose[1] < right_eye[1]:
            return False

        # eyes should be horizontally separated by a meaningful amount
        eye_dist = abs(right_eye[0] - left_eye[0])
        if eye_dist < 10:
            return False

        return True

    def _quality_filter(self, faces: list) -> list:
        """Apply all single-frame quality filters."""
        return [f for f in faces if self._landmarks_plausible(f)]

    # ── CSRT tracker helpers ──────────────────────────────────────────────────

    def _init_tracker(self, frame, face) -> None:
        """Initialise (or re-initialise) the CSRT tracker from a detector bbox."""
        x, y, w, h = int(face[0]), int(face[1]), int(face[2]), int(face[3])
        self.tracker = cv2.TrackerCSRT_create()
        self.tracker.init(frame, (x, y, w, h))
        self.tracker_bbox = (x, y, w, h)

    def _step_tracker(self, frame) -> Optional[tuple]:
        """
        Advance the tracker by one frame.
        Returns (x, y, w, h) on success, None on failure.
        """
        if self.tracker is None:
            return None
        ok, bbox = self.tracker.update(frame)
        if not ok:
            self.get_logger().warn('CSRT tracker lost target', throttle_duration_sec=1.0)
            self.tracker      = None
            self.tracker_bbox = None
            return None
        x, y, w, h = (int(v) for v in bbox)
        self.tracker_bbox = (x, y, w, h)
        return self.tracker_bbox

    # ── temporal confirmation ─────────────────────────────────────────────────

    def _update_confirmation(self, candidates: list) -> Optional[object]:
        """
        Confirm a face only after it appears in N consecutive frames.
        Drop it after M consecutive misses. Returns confirmed face or None.
        """
        if not candidates:
            self.hit_count = 0
            if self.confirmed_face:
                self.miss_count += 1
                if self.miss_count >= self.drop_frames:
                    self.get_logger().info('Face lost — dropping confirmed track')
                    self.confirmed_face = False
                    self.miss_count     = 0
                    self.filtered_cx    = None
                    self.filtered_cy    = None
                    self.tracker        = None
                    self.tracker_bbox   = None
            return None

        largest = max(candidates, key=lambda f: f[2] * f[3])
        self.miss_count  = 0
        self.hit_count  += 1

        if self.hit_count == 1 and not self.confirmed_face:
            self.get_logger().info('Tentative face — waiting to confirm...')

        if self.hit_count >= self.confirm_frames and not self.confirmed_face:
            self.get_logger().info(f'Face confirmed after {self.hit_count} consecutive frames')
            self.confirmed_face = True

        return largest if self.confirmed_face else None

    # ── main callback ─────────────────────────────────────────────────────────

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        self.frame_count += 1
        run_detector = (self.frame_count % self.detect_interval == 1)

        confirmed      = None
        tracker_active = False

        if run_detector:
            # ── full detection pass ───────────────────────────────────────
            faces_full = self._detect(self.detector, frame)
            if self.multiscale:
                faces_zoom = self._multiscale_detect(frame)
                faces      = self._merge_detections(faces_full, faces_zoom)
            else:
                faces = faces_full

            faces      = self._quality_filter(faces)
            candidates = [f for f in faces if float(f[14]) >= self.confirm_threshold]
            confirmed  = self._update_confirmation(candidates)

            # re-initialise tracker whenever detector finds a confirmed face
            if confirmed is not None:
                self._init_tracker(frame, confirmed)
        else:
            faces = []   # no detector output this frame

            if self.confirmed_face and self.tracker is not None:
                # ── tracker fill-in ───────────────────────────────────────
                bbox = self._step_tracker(frame)
                if bbox is not None:
                    tracker_active = True
                    confirmed      = bbox   # (x, y, w, h) — not a YuNet array
                else:
                    # tracker failed mid-interval — fall back to miss logic
                    self._update_confirmation([])

        # ── build publish position ────────────────────────────────────────
        # confirmed is either a YuNet face array (detector frame) or
        # a plain (x,y,w,h) tuple (tracker frame) — handle both
        publish_faces = []
        if confirmed is not None:
            if tracker_active:
                # tracker output: synthetic face-like values for publishing
                x, y, w, h = confirmed
                cx, cy     = x + w // 2, y + h // 2
                score      = 1.0   # tracker doesn't produce a score
            else:
                # detector output: full YuNet array
                x   = int(confirmed[0]); y = int(confirmed[1])
                w   = int(confirmed[2]); h = int(confirmed[3])
                cx  = int(confirmed[0] + confirmed[2] / 2)
                cy  = int(confirmed[1] + confirmed[3] / 2)
                score = float(confirmed[14])
            publish_faces = [(x, y, w, h, cx, cy, score)]

        # ── publish Detection2DArray ──────────────────────────────────────
        det_array        = Detection2DArray()
        det_array.header = msg.header
        for (x, y, w, h, cx, cy, score) in publish_faces:
            d                        = Detection2D()
            d.header                 = msg.header
            hyp                      = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id  = 'face'
            hyp.hypothesis.score     = score
            d.results.append(hyp)
            d.bbox.center.position.x = float(cx)
            d.bbox.center.position.y = float(cy)
            d.bbox.size_x            = float(w)
            d.bbox.size_y            = float(h)
            det_array.detections.append(d)
        self.pub_detections.publish(det_array)

        # ── legacy String publish ─────────────────────────────────────────
        str_msg = String()
        if not publish_faces:
            str_msg.data = 'no_face'
        else:
            _, _, _, _, cx, cy, _ = publish_faces[0]
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
            h_img, w_img = debug.shape[:2]

            # tentative detections (detector frames only) in grey
            for face in faces:
                fx, fy, fw, fh = map(int, face[0:4])
                score          = float(face[14])
                color          = (128, 128, 128)
                cv2.rectangle(debug, (fx, fy), (fx+fw, fy+fh), color, 2)
                cv2.putText(debug, f'{score:.2f}', (fx, max(fy-8, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                for i in range(5):
                    lx = int(face[4 + 2*i]); ly = int(face[5 + 2*i])
                    cv2.circle(debug, (lx, ly), 2, (255, 0, 0), -1)

            # confirmed bbox — green (detector) or cyan (tracker)
            if publish_faces:
                px, py, pw, ph, _, _, _ = publish_faces[0]
                color = (255, 200, 0) if tracker_active else (0, 255, 0)
                label = 'TRACKING' if tracker_active else 'CONFIRMED'
                cv2.rectangle(debug, (px, py), (px+pw, py+ph), color, 2)
                cv2.putText(debug, label, (px, max(py-8, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            # EMA center dot
            if self.filtered_cx is not None and publish_faces:
                cv2.circle(debug, (self.filtered_cx, self.filtered_cy), 5, (0, 0, 255), -1)

            # confirmation progress bar
            if self.hit_count > 0 and not self.confirmed_face:
                pct   = min(self.hit_count / self.confirm_frames, 1.0)
                bar_w = int(w_img * pct)
                cv2.rectangle(debug, (0, 0), (bar_w, 8), (0, 200, 255), -1)
                cv2.putText(debug, f'confirming... {self.hit_count}/{self.confirm_frames} frames',
                            (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1)
            elif self.confirmed_face:
                cv2.putText(debug, 'CONFIRMED', (8, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

            # multiscale crop region
            if self.multiscale:
                cf = self.crop_factor
                x0 = int(w_img * (1 - cf) / 2);  y0 = int(h_img * (1 - cf) / 2)
                x1 = int(w_img * (1 + cf) / 2);  y1 = int(h_img * (1 + cf) / 2)
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