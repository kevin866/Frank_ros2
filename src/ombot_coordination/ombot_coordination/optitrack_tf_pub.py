#!/usr/bin/env python3
from __future__ import annotations

import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data

# ---- Small helpers ----
def quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
    # Returns 3x3 rotation matrix
    x, y, z, w = qx, qy, qz, qw
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    R = np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),       2*(xz + wy)],
        [2*(xy + wz),         1 - 2*(xx + zz),   2*(yz - wx)],
        [2*(xz - wy),         2*(yz + wx),       1 - 2*(xx + yy)],
    ], dtype=float)
    return R

def rot_to_quat(R: np.ndarray):
    # Returns (x,y,z,w)
    m00, m01, m02 = R[0,0], R[0,1], R[0,2]
    m10, m11, m12 = R[1,0], R[1,1], R[1,2]
    m20, m21, m22 = R[2,0], R[2,1], R[2,2]
    tr = m00 + m11 + m22
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = math.sqrt(1.0 + m00 - m11 - m22) * 2
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22) * 2
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11) * 2
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S
    return (qx, qy, qz, qw)

def wrap_pi(a):
    while a > math.pi:  a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def yaw_from_rot(R: np.ndarray) -> float:
    # yaw about +Z (ROS convention)
    return math.atan2(R[1,0], R[0,0])

def rot_from_yaw(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]], dtype=float)

class OptiTrackTF(Node):
    def __init__(self):
        super().__init__("optitrack_tf_pub")

        # ---- Params ----
        self.declare_parameter("pose_topic", "/vrpn_mocap/RigidBody_1/pose")
        self.declare_parameter("world_frame", "world")      # or "map"
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("use_planar", True)          # publish x,y,yaw only (set z=0, roll=pitch=0)
        self.declare_parameter("axis_fix", "xzy")           # your case: mocap plane is x-z; map it to x-y
        self.declare_parameter("yaw_offset", 0.0)           # radians
        self.declare_parameter("alpha_pos", 0.2)            # low-pass (0=no filter, 1=heavy)
        self.declare_parameter("alpha_yaw", 0.2)

        self.pose_topic = self.get_parameter("pose_topic").value
        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.use_planar = bool(self.get_parameter("use_planar").value)
        self.axis_fix = str(self.get_parameter("axis_fix").value)
        self.yaw_offset = float(self.get_parameter("yaw_offset").value)
        self.alpha_pos = float(self.get_parameter("alpha_pos").value)
        self.alpha_yaw = float(self.get_parameter("alpha_yaw").value)

        self.br = TransformBroadcaster(self)
        # self.sub = self.create_subscription(PoseStamped, self.pose_topic, self.cb, 10)

        self.sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.cb,
            qos_profile_sensor_data
        )

        self.have_filt = False
        self.p_f = np.zeros(3)
        self.yaw_f = 0.0

        self.get_logger().info(
            f"Sub: {self.pose_topic} | TF: {self.world_frame}->{self.base_frame} | planar={self.use_planar} axis_fix={self.axis_fix}"
        )

    def axis_fix_rot(self) -> np.ndarray:
        """
        Maps OptiTrack axes into ROS world axes.
        axis_fix="xzy" means: ROS [x,y,z] = [mocap x, mocap z, mocap y]
        Common for your note: OptiTrack plane is x-z (so treat mocap z as ROS y).
        """
        if self.axis_fix == "none":
            return np.eye(3)
        if self.axis_fix == "xzy":
            # ROSx=mocapx, ROSy=mocapz, ROSz=mocapy
            return np.array([[1,0,0],
                             [0,0,1],
                             [0,1,0]], dtype=float)
        if self.axis_fix == "xyz":
            return np.eye(3)
        # Add more if needed
        self.get_logger().warn(f"Unknown axis_fix={self.axis_fix}, using identity.")
        return np.eye(3)

    def cb(self, msg: PoseStamped):
        # Raw pose
        p_m = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
        q = msg.pose.orientation
        R_m = quat_to_rot(q.x, q.y, q.z, q.w)

        # Axis remap
        A = self.axis_fix_rot()
        p_w = A @ p_m
        R_w = A @ R_m @ A.T

        # Planar option: keep x,y and yaw only
        if self.use_planar:
            yaw = wrap_pi(yaw_from_rot(R_w) + self.yaw_offset)
            R_w = rot_from_yaw(yaw)
            p_w[2] = 0.0
        else:
            # still apply yaw offset as a pure yaw rotation in world frame if you want
            if abs(self.yaw_offset) > 1e-9:
                R_w = rot_from_yaw(self.yaw_offset) @ R_w

        # Low-pass filter (helps jitter)
        if not self.have_filt:
            self.p_f = p_w.copy()
            self.yaw_f = yaw_from_rot(R_w)
            self.have_filt = True
        else:
            a = self.alpha_pos
            self.p_f = (1 - a) * p_w + a * self.p_f

            yaw = yaw_from_rot(R_w)
            # blend angles safely
            dy = wrap_pi(yaw - self.yaw_f)
            self.yaw_f = wrap_pi(self.yaw_f + (1 - self.alpha_yaw) * dy)

        # Build TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(self.p_f[0])
        t.transform.translation.y = float(self.p_f[1])
        t.transform.translation.z = float(self.p_f[2])

        R_out = rot_from_yaw(self.yaw_f) if self.use_planar else R_w
        qx, qy, qz, qw = rot_to_quat(R_out)
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = OptiTrackTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
