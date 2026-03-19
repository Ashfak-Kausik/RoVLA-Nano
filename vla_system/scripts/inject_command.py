#!/usr/bin/env python3
"""
VLA Command Injector
--------------------
One-shot script to send a command to the VLA system from the terminal,
without running the full interactive CLI.

Usage:
  # Send a command
  ros2 run vla_system inject "Pick the screw and place it in bin A"

  # Or run directly
  python3 scripts/inject_command.py "Pick the screw"

  # Watch the result (waits up to 10 s)
  python3 scripts/inject_command.py "Pick the screw" --wait
"""

import sys
import argparse
import threading
import time
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_interfaces.msg import ActionResult


class CommandInjector(Node):
    def __init__(self, command: str, wait_for_result: bool):
        super().__init__("vla_inject")
        self.command         = command
        self.wait_for_result = wait_for_result
        self._done           = threading.Event()
        self._result         = None

        self.pub = self.create_publisher(String, "/raw_command", 10)

        if wait_for_result:
            self.sub = self.create_subscription(
                ActionResult, "/action_result", self._result_cb, 10)
            self.sub_status = self.create_subscription(
                String, "/system_status", self._status_cb, 10)

        # Publish after a short delay so the subscriber has time to connect
        self.create_timer(0.5, self._send)

    def _send(self):
        msg = String()
        msg.data = self.command
        self.pub.publish(msg)
        self.get_logger().info(f"Sent: '{self.command}'")
        if not self.wait_for_result:
            self._done.set()

    def _result_cb(self, msg: ActionResult):
        self._result = msg
        icon = "✓" if msg.success else "✗"
        print(f"\n{icon} Result: {msg.message}")
        print(f"  Status:  {msg.status}")
        print(f"  Time:    {msg.execution_time_sec:.2f}s")
        self._done.set()

    def _status_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
            if d.get("state") in ("working", "executing"):
                print(f"  ⟳  {d.get('detail', '')}")
        except Exception:
            pass

    def wait(self, timeout=10.0):
        self._done.wait(timeout)
        return self._result


def main():
    parser = argparse.ArgumentParser(description="Send a command to the VLA system")
    parser.add_argument("command", help="Natural language command")
    parser.add_argument("--wait", action="store_true",
                        help="Wait for action result before exiting")
    parser.add_argument("--timeout", type=float, default=15.0,
                        help="Timeout in seconds when --wait is used")
    args = parser.parse_args()

    rclpy.init()
    node = CommandInjector(args.command, args.wait)

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    result = node.wait(args.timeout)

    if args.wait and result is None:
        print("⏱  Timeout — no result received within "
              f"{args.timeout:.0f}s")
        sys.exit(1)

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if (result is None or result.success) else 1)


if __name__ == "__main__":
    main()
