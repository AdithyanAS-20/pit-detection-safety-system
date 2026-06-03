#!/usr/bin/env python3
"""
pit_safety_node.py  —  ROS2 Humble/Iron/Jazzy  (Raspberry Pi)
==============================================================
Acts as a VELOCITY GATE between your nav stack and the motors.

Subscriptions:
  /cmd_vel          [Twist]  — from Nav2 / teleop
  /pit_detected     [Bool]   — from ESP32-S3 micro-ROS
  /pit_status       [UInt8]  — bitmask FL/FR/BL/BR

Publications:
  /cmd_vel_out      [Twist]  — to motor controller / diff_drive
  /pit_override     [Bool]   — send True to override pit stop

Logic:
  • When pit_detected=True, gate passes only zero-vel.
  • When pit_detected=False and hold timer expired, pass through cmd_vel normally.
  • Publishes override=False on startup (safe default).

Usage:
  # Terminal 1 — start micro-ROS agent (connects ESP32 serial)
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

  # Terminal 2
  ros2 run <your_package> pit_safety_node

Install:
  Place in src/<your_package>/<your_package>/pit_safety_node.py
  Add entry point in setup.py:
    'pit_safety_node = <your_package>.pit_safety_node:main'
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt8
import time


# Bit positions match Arduino protocol
SENSOR_NAMES = {0: "Front-Left", 1: "Front-Right", 2: "Rear-Left", 3: "Rear-Right"}


class PitSafetyNode(Node):
    def __init__(self):
        super().__init__('pit_safety_node')

        # Parameters
        self.declare_parameter('stop_hold_sec',  2.0)   # seconds to hold stop after clear
        self.declare_parameter('cmd_vel_in',    '/cmd_vel')
        self.declare_parameter('cmd_vel_out',   '/cmd_vel_out')

        self.stop_hold_sec = self.get_parameter('stop_hold_sec').value
        cmd_in  = self.get_parameter('cmd_vel_in').value
        cmd_out = self.get_parameter('cmd_vel_out').value

        # State
        self.pit_active     = False
        self.last_danger_ts = 0.0
        self.latest_cmd_vel = Twist()   # last nav stack command

        # Subscriptions
        self.create_subscription(Twist,  cmd_in,          self.cmd_vel_cb,      10)
        self.create_subscription(Bool,   '/pit_detected',  self.pit_detected_cb, 10)
        self.create_subscription(UInt8,  '/pit_status',    self.pit_status_cb,   10)

        # Publishers
        self.pub_cmd_vel_out = self.create_publisher(Twist, cmd_out, 10)
        self.pub_override    = self.create_publisher(Bool,  '/pit_override', 10)

        # Timer — publish gated cmd_vel at 20 Hz
        self.create_timer(0.05, self.gate_timer_cb)

        # Safe default: override OFF
        self._publish_override(False)

        self.get_logger().info(
            f"[PitSafety] Ready | in={cmd_in} → out={cmd_out} | hold={self.stop_hold_sec}s"
        )

    # ── Callbacks ─────────────────────────────────────────

    def cmd_vel_cb(self, msg: Twist):
        """Store latest navigation command."""
        self.latest_cmd_vel = msg

    def pit_detected_cb(self, msg: Bool):
        if msg.data:
            if not self.pit_active:
                self.get_logger().warn("[PitSafety] ⚠ PIT/STEP DETECTED — motors stopped!")
            self.pit_active     = True
            self.last_danger_ts = time.monotonic()
        else:
            if self.pit_active:
                self.get_logger().info("[PitSafety] Path clear. Resuming after hold timer...")
            self.pit_active = False

    def pit_status_cb(self, msg: UInt8):
        """Log which specific sensors triggered."""
        bitmask = msg.data
        if bitmask:
            triggered = [name for bit, name in SENSOR_NAMES.items() if bitmask & (1 << bit)]
            self.get_logger().warn(f"[PitSafety] Triggered: {', '.join(triggered)}")

    # ── Gate Timer ────────────────────────────────────────

    def gate_timer_cb(self):
        """
        Publishes to /cmd_vel_out every 50 ms.
        While pit is active OR hold timer running → publish zero-vel.
        Otherwise → pass through latest nav cmd_vel.
        """
        now = time.monotonic()
        in_hold = (now - self.last_danger_ts) < self.stop_hold_sec

        if self.pit_active or in_hold:
            self.pub_cmd_vel_out.publish(Twist())   # zero Twist = stop
        else:
            self.pub_cmd_vel_out.publish(self.latest_cmd_vel)

    # ── Helpers ───────────────────────────────────────────

    def _publish_override(self, val: bool):
        msg = Bool()
        msg.data = val
        self.pub_override.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PitSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
