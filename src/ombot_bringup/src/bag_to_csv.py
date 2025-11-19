#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import csv
import os
from datetime import datetime


class BagToCSV(Node):
    def __init__(self):
        super().__init__('bag_to_csv')

        # Parameters (optional): you could make these configurable later
        self.output_filename = 'joint6_id_data.csv'
        self.joint_name = 'joint_6'
        self.tau_joint_index = 5  # index in tau_cmd.data corresponding to joint_6

        # State buffers
        self.last_js = None
        self.last_tau = None
        self.t0 = self.get_clock().now()


        # Set up CSV file
        # If file exists, overwrite it
        self.get_logger().info(f"Writing CSV to {os.path.abspath(self.output_filename)}")
        self.csv_file = open(self.output_filename, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        # header
        self.writer.writerow(['t', 'q6', 'dq6', 'tau6'])

        # Subscriptions
        self.js_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.js_callback,
            50
        )
        self.tau_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_excitation_controller/tau_cmd',
            self.tau_callback,
            50
        )

        # === Auto-stop after 30 sec ===
        self.start_time = self.get_clock().now()
        self.timeout_sec = 30.0
        self.done = False
        self.timer = self.create_timer(0.1, self.check_timeout)

    def js_callback(self, msg: JointState):
        self.last_js = msg
        self.maybe_write_row()

    def tau_callback(self, msg: Float64MultiArray):
        self.last_tau = msg
        self.maybe_write_row()

    def maybe_write_row(self):
        # Need both messages at least once
        if self.last_js is None or self.last_tau is None:
            return

        js = self.last_js
        tau = self.last_tau

        # Find joint_6 index in joint_states.name
        try:
            idx_js = js.name.index(self.joint_name)
        except ValueError:
            # joint_6 not in this message; nothing to log
            return

        # Safety: check arrays large enough
        if idx_js >= len(js.position):
            return

        # velocity may be shorter; if missing, treat as 0.0
        dq6 = 0.0
        if idx_js < len(js.velocity):
            dq6 = js.velocity[idx_js]

        # torque command index in tau_cmd.data (0..5 for joint_1..joint_6)
        if self.tau_joint_index >= len(tau.data):
            return

        tau6 = tau.data[self.tau_joint_index]

        # Time stamp from joint_states
        now = self.get_clock().now()
        t_rel = (now - self.t0).nanoseconds * 1e-9
        q6 = js.position[idx_js]

        # Write row
        self.writer.writerow([t_rel, q6, dq6, tau6])

    def finalize_csv(self):
        """Flush, close, and auto-rename the CSV file."""
        # Close file
        try:
            if getattr(self, "csv_file", None) is not None and not self.csv_file.closed:
                self.csv_file.flush()
                self.csv_file.close()
        except Exception as e:
            self.get_logger().warn(f"Error while closing CSV: {e}")

        # Build new filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        new_name = f"joint6_id_data_{timestamp}.csv"

        try:
            os.rename(self.output_filename, new_name)
            self.get_logger().info(
                f"Renamed CSV to {os.path.abspath(new_name)}"
            )
        except OSError as e:
            self.get_logger().error(f"Failed to rename CSV: {e}")

    def check_timeout(self):
        if self.done:
            return

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        if elapsed >= self.timeout_sec:
            self.done = True
            self.get_logger().info(
                f"{self.timeout_sec:.1f} seconds passed — finalizing CSV and shutting down."
            )
            self.finalize_csv()
            rclpy.shutdown()

    def destroy_node(self):
        # Close file cleanly (in case not already closed)
        try:
            if getattr(self, "csv_file", None) is not None and not self.csv_file.closed:
                self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BagToCSV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Avoid double-shutdown errors
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
