#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Your joint names and home positions
NAMES = ['joint_1', 'joint_2', 'joint_4', 'joint_5', 'joint_3', 'joint_6']
HOME  = [-0.048001135752655535,
         -0.4151982449018653,
         -0.007936554979054274,
          0.5437359966568645,
         -0.09646544584951455,
          1.587365319160496]

class ContinuousTraj(Node):
    def __init__(self):
        super().__init__('continuous_traj')

        # --- Motion parameters ---
        self.topic     = '/joint_impedance_controller/trajectory'
        self.duration  = 4.0  # seconds per leg
        self.names     = NAMES
        self.home      = HOME

        # 2× longer amplitude (deg → rad)
        amp_deg = [12, 16, 8, 12, 10, 20]  # safe-ish but visible
        self.amp = [math.radians(a) for a in amp_deg]

        self.pub = self.create_publisher(JointTrajectory, self.topic, 1)
        self.get_logger().info(
            f"Continuous bidirectional ping-pong motion (±offset): "
            f"{self.duration:.1f}s per leg"
        )

        # states: 0=home, 1=+offset, 2=-offset
        self.state = 0
        self.timer = self.create_timer(self.duration + 0.3, self._tick)
        self._tick()  # start immediately

    def _tick(self):
        # Define next target in sequence: home → +off → -off → +off …
        if self.state == 0:
            target = self._target(+1)
            self.state = 1
        elif self.state == 1:
            target = self._target(-1)
            self.state = 2
        else:
            target = self._target(+1)
            self.state = 1  # alternate between + and -

        self._send_one_point(target, self.duration)
        direction = "home" if self.state == 0 else ("+offset" if self.state == 1 else "-offset")
        self.get_logger().info(f"Moving toward {direction} (leg {self.state}) in {self.duration:.1f}s")

    def _target(self, sign):
        # Offset in both + and - directions
        target = [self.home[i] + sign * self.amp[i] for i in range(len(self.home))]
        return target

    def _send_one_point(self, positions, seconds):
        msg = JointTrajectory()
        msg.joint_names = self.names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.velocities = [0.0]*len(positions)
        pt.accelerations = [0.0]*len(positions)
        pt.time_from_start.sec = int(seconds)
        pt.time_from_start.nanosec = int((seconds - int(seconds)) * 1e9)
        msg.points = [pt]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ContinuousTraj()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
