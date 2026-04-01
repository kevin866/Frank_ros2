from typing import Optional, Deque
import collections

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

import numpy as np
from cv_bridge import CvBridge


class BehaviorManagerNode(Node):
    def __init__(self):
        super().__init__('behavior_manager_node')

        self.pub_cmd_vel = self.create_publisher(TwistStamped, '/mecanum_controller/reference', 10)

        self.sub_intent = self.create_subscription(
            String, '/frank/intent', self.intent_callback, 10)
        self.sub_face_info = self.create_subscription(
            String, '/vision/face_info', self.face_callback, 10)
        self.sub_detections = self.create_subscription(
            Detection2DArray, '/vision/face_detections', self.detections_callback, 10)
        self.sub_depth = self.create_subscription(
            Image, '/zed/zed_node/depth/depth_registered', self.depth_callback, 10)

        self.bridge = CvBridge()

        self.mode = 'idle'
        self.latest_face_center: Optional[tuple] = None   # (cx, cy)
        self.latest_face_bbox: Optional[tuple] = None     # (x, y, w, h)
        self.latest_depth_image = None

        self.image_width = 640
        self.center_tolerance_px = 40

        # ── tuning ────────────────────────────────────────────────────────────
        self.yaw_k          = 0.0008
        self.yaw_max        = 0.2
        
        # EMA filter for depth
        self.filtered_depth: Optional[float] = None
        self.depth_alpha = 0.2  # lower = smoother, higher = more responsive

        self.approach_k     = 0.3
        self.approach_max   = 0.15
        self.target_dist    = 1.5
        self.dist_tolerance = 0.05

        # ── obstacle detection ────────────────────────────────────────────────
        self.depth_history: Deque[float] = collections.deque(maxlen=5)  # last 5 readings
        self.obstacle_drop_threshold = 0.4   # if depth drops >0.4m in one step → obstacle
        self.obstacle_detected = False

        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('BehaviorManagerNode started.')

    # ── callbacks ─────────────────────────────────────────────────────────────

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
        else:
            self.get_logger().warn(f'Unknown intent: {intent}')

    def face_callback(self, msg: String):
        data = msg.data.strip()
        if data == 'no_face':
            self.latest_face_center = None
            return
        if data.startswith('face_detected:'):
            try:
                parts = data.split(':')[1].split(',')
                cx = int(parts[0])
                cy = int(parts[1]) if len(parts) > 1 else 0
                self.latest_face_center = (cx, cy)
            except Exception:
                self.get_logger().warn(f'Could not parse face info: {data}')

    def detections_callback(self, msg: Detection2DArray):
        """Get face bounding box from Detection2DArray for better depth sampling."""
        if not msg.detections:
            self.latest_face_bbox = None
            return

        # pick largest face — same logic as face_detector_node
        largest = max(msg.detections, key=lambda d: d.bbox.size_x * d.bbox.size_y)
        cx = largest.bbox.center.position.x
        cy = largest.bbox.center.position.y
        w  = largest.bbox.size_x
        h  = largest.bbox.size_y
        self.latest_face_bbox = (cx, cy, w, h)

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')

    # ── depth sampling ────────────────────────────────────────────────────────

    def _sample_depth(self, cx: int, cy: int) -> Optional[float]:
        if self.latest_depth_image is None:
            return None

        h, w = self.latest_depth_image.shape

        # if we have a bbox use it, otherwise fall back to 10px patch
        if self.latest_face_bbox is not None:
            _, _, bw, bh = self.latest_face_bbox
            # shrink bbox slightly to avoid background at edges
            px = int(bw * 0.3)
            py = int(bh * 0.3)
        else:
            px = py = 10

        x0 = max(0, cx - px)
        x1 = min(w, cx + px)
        y0 = max(0, cy - py)
        y1 = min(h, cy + py)

        patch = self.latest_depth_image[y0:y1, x0:x1]
        valid = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 10.0)]

        if len(valid) == 0:
            return None

        # use 75th percentile instead of median — biased toward farther readings
        # so small obstacles in front of face don't pull depth down
        raw_depth = float(np.percentile(valid, 75))
        if self.filtered_depth is None:
            self.filtered_depth = raw_depth
        else:
            self.filtered_depth = self.depth_alpha * raw_depth + (1 - self.depth_alpha) * self.filtered_depth
        return self.filtered_depth

    def _check_obstacle(self, depth: float) -> bool:
        """Returns True if an obstacle is detected based on sudden depth drop."""
        if len(self.depth_history) < 2:
            return False

        prev_depth = self.depth_history[-1]
        drop = prev_depth - depth  # positive = getting closer suddenly

        if drop > self.obstacle_drop_threshold:
            self.get_logger().warn(
                f'Obstacle detected! depth dropped {drop:.2f}m '
                f'({prev_depth:.2f}m → {depth:.2f}m)',
                throttle_duration_sec=1.0
            )
            return True
        return False

    # ── control loop ──────────────────────────────────────────────────────────

    def control_loop(self):
        if self.mode == 'idle':
            return

        if self.mode == 'go_home':
            self.publish_stop()
            return

        if self.mode == 'find_face':
            if self.latest_face_center is None:
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.twist.angular.z = 0.2
                self.pub_cmd_vel.publish(msg)
            else:
                self.publish_stop()
                self.mode = 'idle'
                self.get_logger().info('Face found. Stopping.')
            return

        if self.mode == 'track_face':
            if self.latest_face_center is None:
                self.publish_stop()
                return

            cx, cy = self.latest_face_center
            image_center = self.image_width // 2
            error = cx - image_center

            # ── yaw control ───────────────────────────────────────────────
            if abs(error) < self.center_tolerance_px:
                yaw_cmd = 0.0
            else:
                yaw_cmd = self.yaw_k * error
                yaw_cmd = max(-self.yaw_max, min(self.yaw_max, yaw_cmd))

            # ── approach control with obstacle detection ───────────────────
            depth = self._sample_depth(cx, cy)
            linear_cmd = 0.0

            if depth is not None:
                # obstacle check
                self.obstacle_detected = self._check_obstacle(depth)
                self.depth_history.append(depth)

                if self.obstacle_detected:
                    # stop forward motion but keep turning
                    linear_cmd = 0.0
                else:
                    dist_error = depth - self.target_dist
                    if abs(dist_error) > self.dist_tolerance:
                        linear_cmd = self.approach_k * dist_error
                        linear_cmd = max(-self.approach_max, min(self.approach_max, linear_cmd))

                self.get_logger().info(
                    f'cx={cx} err={error:.0f} yaw={yaw_cmd:.3f} '
                    f'depth={depth:.2f}m dist_err={depth - self.target_dist:.2f} '
                    f'linear={linear_cmd:.3f} obstacle={self.obstacle_detected}',
                    throttle_duration_sec=1.0
                )
            else:
                self.get_logger().warn('No valid depth at face center', throttle_duration_sec=1.0)
                
            # ignore tiny commands — deadband to prevent jitter
            MIN_LINEAR  = 0.03   # m/s
            MIN_ANGULAR = 0.02   # rad/s

            if abs(linear_cmd) < MIN_LINEAR:
                linear_cmd = 0.0
            if abs(yaw_cmd) < MIN_ANGULAR:
                yaw_cmd = 0.0

            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.angular.z = yaw_cmd
            msg.twist.linear.x  = linear_cmd
            self.pub_cmd_vel.publish(msg)

    # ── helpers ───────────────────────────────────────────────────────────────

    def publish_stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_cmd_vel.publish(msg)


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