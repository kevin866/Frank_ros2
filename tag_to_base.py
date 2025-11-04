# tag_to_base_pose.py
# --- fix for numpy np.float removal ---
import numpy as np
if not hasattr(np, 'float'):
    np.float = float
# --------------------------------------

from tf_transformations import quaternion_from_matrix
import rclpy
from rclpy.node import Node
# ... rest of your code ...
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
if not hasattr(np, 'float'):
    np.float = float  # fix for deprecated alias in recent numpy


class TagPoseToBase(Node):
    def __init__(self):
        super().__init__('tag_pose_to_base')
        self.tag_frame = 'zed_imu_link_0'                 # CHANGE to your tag frame id
        self.base_frame = 'base_link'            # CHANGE if needed
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(PoseStamped, 'tag_0_pose_base', 10)
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

    def tick(self):
        try:
            t = self.buffer.lookup_transform(self.base_frame, self.tag_frame, rclpy.time.Time())
            ps = PoseStamped()
            ps.header = t.header
            ps.header.frame_id = self.base_frame
            ps.pose.position.x = t.transform.translation.x
            ps.pose.position.y = t.transform.translation.y
            ps.pose.position.z = t.transform.translation.z
            ps.pose.orientation = t.transform.rotation
            self.pub.publish(ps)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # Tag not visible or transform not available - don't publish
            self.get_logger().debug(f'Tag not available: {str(e)}')
            pass

def main():
    rclpy.init()
    n = TagPoseToBase()
    rclpy.spin(n)

if __name__ == '__main__':
    main()
