#!/usr/bin/env python3
"""
VLA Topic Monitor
-----------------
Subscribes to all VLA topics and prints a live dashboard in the terminal.
Useful for debugging and demo recordings.

Usage:
  python3 scripts/monitor_topics.py
  ros2 run vla_system monitor
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_interfaces.msg import ObjectDetection, TaskCommand, ActionResult

import json
import time
from collections import defaultdict

RESET  = "\033[0m"
BOLD   = "\033[1m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
RED    = "\033[91m"
BLUE   = "\033[94m"
CYAN   = "\033[96m"
GRAY   = "\033[90m"
CLEAR  = "\033[2J\033[H"


class TopicMonitor(Node):

    def __init__(self):
        super().__init__("vla_monitor")
        self._detections: dict[str, ObjectDetection] = {}
        self._last_cmd: TaskCommand | None = None
        self._last_result: ActionResult | None = None
        self._system_state = "unknown"
        self._system_detail = ""
        self._event_log: list[str] = []
        self._t0 = time.time()

        self.create_subscription(
            ObjectDetection, "/detected_objects", self._det_cb, 10)
        self.create_subscription(
            TaskCommand, "/task_command", self._cmd_cb, 10)
        self.create_subscription(
            TaskCommand, "/execute_action", self._exec_cb, 10)
        self.create_subscription(
            ActionResult, "/action_result", self._result_cb, 10)
        self.create_subscription(
            String, "/system_status", self._status_cb, 10)
        self.create_subscription(
            String, "/raw_command", self._raw_cb, 10)

        self.create_timer(0.5, self._render)

    # ── Callbacks ────────────────────────────────────────────────────
    def _det_cb(self, msg):
        self._detections[msg.label] = msg

    def _cmd_cb(self, msg):
        self._last_cmd = msg
        self._log(f"{BLUE}Language{RESET} → action={msg.action} "
                  f"obj={msg.object_label} target={msg.target_location}")

    def _exec_cb(self, msg):
        self._log(f"{CYAN}Coordinator{RESET} → dispatched {msg.action} "
                  f"on {msg.object_label}")

    def _result_cb(self, msg):
        self._last_result = msg
        icon = f"{GREEN}✓{RESET}" if msg.success else f"{RED}✗{RESET}"
        self._log(f"{icon} Action result: {msg.message} "
                  f"({msg.execution_time_sec:.2f}s)")

    def _status_cb(self, msg):
        try:
            d = json.loads(msg.data)
            self._system_state  = d.get("state", "unknown")
            self._system_detail = d.get("detail", "")
        except Exception:
            pass

    def _raw_cb(self, msg):
        self._log(f"{YELLOW}User{RESET} → \"{msg.data}\"")

    def _log(self, text: str):
        t = time.time() - self._t0
        self._event_log.append(f"{GRAY}+{t:6.1f}s{RESET}  {text}")
        if len(self._event_log) > 12:
            self._event_log.pop(0)

    # ── Render ───────────────────────────────────────────────────────
    def _render(self):
        lines = [CLEAR]

        # Header
        lines.append(f"{BOLD}  VLA System Monitor{RESET}   "
                     f"{GRAY}(Ctrl+C to quit){RESET}")
        lines.append(f"  {'─' * 52}")

        # System state
        state_color = {
            "idle": GREEN, "working": YELLOW,
            "executing": CYAN, "error": RED,
        }.get(self._system_state, GRAY)
        lines.append(
            f"\n  State   {state_color}{BOLD}{self._system_state:<12}{RESET}"
            f"  {self._system_detail[:42]}"
        )

        # Detected objects
        lines.append(f"\n  {BOLD}Detected Objects{RESET}")
        if self._detections:
            lines.append(
                f"  {'Label':<10} {'Conf':>6}  {'X':>7}  {'Y':>7}  {'Z':>7}")
            lines.append(f"  {'─'*10} {'─'*6}  {'─'*7}  {'─'*7}  {'─'*7}")
            for label, d in sorted(self._detections.items()):
                bar = "█" * int(d.confidence * 10) + "░" * (10 - int(d.confidence * 10))
                lines.append(
                    f"  {label:<10} {d.confidence:>6.2f}  "
                    f"{d.x:>7.3f}  {d.y:>7.3f}  {d.z:>7.3f}  "
                    f"{GRAY}{bar}{RESET}"
                )
        else:
            lines.append(f"  {GRAY}(none detected yet){RESET}")

        # Last command
        if self._last_cmd:
            c = self._last_cmd
            lines.append(f"\n  {BOLD}Last Command{RESET}")
            lines.append(f"  action={c.action}  obj={c.object_label}  "
                         f"target={c.target_location}  "
                         f"mods={list(c.modifiers)}")

        # Last result
        if self._last_result:
            r = self._last_result
            icon = f"{GREEN}✓{RESET}" if r.success else f"{RED}✗{RESET}"
            lines.append(f"\n  {BOLD}Last Result{RESET}  "
                         f"{icon} {r.message} — {r.status} "
                         f"({r.execution_time_sec:.2f}s)")

        # Event log
        lines.append(f"\n  {BOLD}Events{RESET}")
        if self._event_log:
            for entry in self._event_log[-8:]:
                lines.append(f"  {entry}")
        else:
            lines.append(f"  {GRAY}(waiting for events…){RESET}")

        print("\n".join(lines), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = TopicMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
