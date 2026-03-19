#!/usr/bin/env python3
"""
VLA CLI — Interactive command-line interface for the VLA system.

Usage:
  python3 vla_cli.py

  Or as a ROS2 script:
  ros2 run vla_system vla_cli
"""

import sys
import json
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_interfaces.msg import ObjectDetection, ActionResult


class VlaCli(Node):

    def __init__(self):
        super().__init__("vla_cli")

        self._status = {"state": "unknown", "detail": ""}
        self._detections: dict[str, ObjectDetection] = {}
        self._lock = threading.Lock()

        # ---- Publisher ----
        self.pub_cmd = self.create_publisher(String, "/raw_command", 10)

        # ---- Subscribers ----
        self.create_subscription(
            String, "/system_status", self._status_cb, 10)
        self.create_subscription(
            ObjectDetection, "/detected_objects", self._det_cb, 10)
        self.create_subscription(
            ActionResult, "/action_result", self._result_cb, 10)

    def send_command(self, text: str):
        msg = String()
        msg.data = text
        self.pub_cmd.publish(msg)
        print(f"  ↗  Sent: '{text}'")

    def _status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._status = data
            state  = data.get("state", "")
            detail = data.get("detail", "")
            if state == "idle":
                print(f"\n  ✓  {detail}")
            elif state == "error":
                print(f"\n  ✗  {detail}")
            elif state in ("working", "executing"):
                print(f"\n  ⟳  {detail}")
        except Exception:
            pass

    def _det_cb(self, msg: ObjectDetection):
        with self._lock:
            self._detections[msg.label] = msg

    def _result_cb(self, msg: ActionResult):
        icon = "✓" if msg.success else "✗"
        print(f"\n  {icon}  Result: {msg.message} "
              f"(status={msg.status}, time={msg.execution_time_sec:.2f}s)")

    def print_status(self):
        with self._lock:
            s = self._status
        print(f"  State:  {s.get('state','—')}")
        print(f"  Detail: {s.get('detail','—')}")

    def print_detections(self):
        with self._lock:
            dets = dict(self._detections)
        if not dets:
            print("  No objects currently detected.")
        else:
            print(f"  {'Label':<12} {'Conf':>6}  {'X':>7}  {'Y':>7}  {'Z':>7}")
            print(f"  {'─'*12} {'─'*6}  {'─'*7}  {'─'*7}  {'─'*7}")
            for label, d in sorted(dets.items()):
                print(f"  {label:<12} {d.confidence:>6.2f}  "
                      f"{d.x:>7.3f}  {d.y:>7.3f}  {d.z:>7.3f}")


def spin_background(node: Node):
    """Spin the node in a background thread."""
    import rclpy.executors
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()


def main():
    rclpy.init()
    cli = VlaCli()

    # Spin in background thread
    thread = threading.Thread(target=spin_background, args=(cli,), daemon=True)
    thread.start()

    print()
    print("┌─────────────────────────────────────────┐")
    print("│       VLA System — Command Interface     │")
    print("├─────────────────────────────────────────┤")
    print("│  Type natural language commands, e.g.:  │")
    print("│    Pick the screw                        │")
    print("│    Pick the smallest rivet and place     │")
    print("│    it in bin A                           │")
    print("│  Special: :status  :objects  :quit       │")
    print("└─────────────────────────────────────────┘")
    print()

    try:
        while True:
            try:
                text = input("vla> ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not text:
                continue

            if text.lower() in (":quit", ":exit", "q", "quit"):
                break
            elif text.lower() == ":status":
                cli.print_status()
            elif text.lower() == ":objects":
                cli.print_detections()
            else:
                cli.send_command(text)

    finally:
        print("\nShutting down.")
        cli.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()