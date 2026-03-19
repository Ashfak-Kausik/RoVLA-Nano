#!/usr/bin/env python3
"""
VLA Task Coordinator
---------------------
The brain of the VLA system. Listens for task commands, queries the vision
node to find the target object, then triggers the action node.

Flow:
  1. Receives /task_command (from language node)
  2. Scans /detected_objects for the requested object
  3. If "smallest" modifier: picks the object with smallest bounding box
  4. Publishes /execute_action with object pose
  5. Monitors /action_result and reports back

Subscribes:
  /task_command       (vla_interfaces/TaskCommand)
  /detected_objects   (vla_interfaces/ObjectDetection)
  /action_result      (vla_interfaces/ActionResult)

Publishes:
  /execute_action     (vla_interfaces/TaskCommand  — enriched with pose)
  /system_status      (std_msgs/String)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from collections import defaultdict
from threading import Lock

from std_msgs.msg import String, Header
from vla_interfaces.msg import ObjectDetection, TaskCommand, ActionResult


# ---------------------------------------------------------------------------
# Coordinator Node
# ---------------------------------------------------------------------------
class TaskCoordinator(Node):

    def __init__(self):
        super().__init__("task_coordinator")

        self.declare_parameter("object_timeout_sec", 3.0)
        self.declare_parameter("min_confidence", 0.6)

        self.obj_timeout  = self.get_parameter("object_timeout_sec").value
        self.min_conf     = self.get_parameter("min_confidence").value

        # ---- State ----
        self._lock = Lock()
        self._detections: dict[str, list[ObjectDetection]] = defaultdict(list)
        self._pending_task: TaskCommand | None = None
        self._busy = False

        # ---- Subscribers ----
        self.sub_cmd = self.create_subscription(
            TaskCommand, "/task_command", self._task_callback, 10)
        self.sub_det = self.create_subscription(
            ObjectDetection, "/detected_objects", self._detection_callback, 10)
        self.sub_result = self.create_subscription(
            ActionResult, "/action_result", self._result_callback, 10)

        # ---- Publishers ----
        self.pub_exec   = self.create_publisher(TaskCommand, "/execute_action", 10)
        self.pub_status = self.create_publisher(String, "/system_status", 10)

        # ---- Timer: clear stale detections every 0.5 s ----
        self.create_timer(0.5, self._clear_stale_detections)

        self.get_logger().info("TaskCoordinator ready.")
        self._publish_status("idle", "System ready.")

    # ------------------------------------------------------------------
    # Detection buffer
    # ------------------------------------------------------------------
    def _detection_callback(self, msg: ObjectDetection):
        with self._lock:
            label = msg.label
            # Keep a rolling window — max 20 detections per class
            self._detections[label].append(msg)
            if len(self._detections[label]) > 20:
                self._detections[label].pop(0)

    def _clear_stale_detections(self):
        now = self.get_clock().now()
        with self._lock:
            for label in list(self._detections.keys()):
                self._detections[label] = [
                    d for d in self._detections[label]
                    if (now - rclpy.time.Time.from_msg(d.header.stamp)).nanoseconds
                       < self.obj_timeout * 1e9
                ]

    # ------------------------------------------------------------------
    # Task received
    # ------------------------------------------------------------------
    def _task_callback(self, msg: TaskCommand):
        if self._busy:
            self.get_logger().warn(
                "Still executing previous task — ignoring new command.")
            self._publish_status("busy", "Previous task still running.")
            return

        if msg.action == "unknown" or not msg.object_label:
            self.get_logger().warn("Received incomplete task command — skipping.")
            self._publish_status("error", "Incomplete command. Could not understand instruction.")
            return

        self.get_logger().info(
            f"New task: action={msg.action} object={msg.object_label} "
            f"target={msg.target_location}"
        )
        self._pending_task = msg
        self._busy = True
        self._publish_status("working", f"Looking for {msg.object_label}...")

        # Try to find the object immediately; if not found, wait briefly
        self._try_execute()

    def _try_execute(self, retry_count: int = 0):
        """Attempt to find the target object and dispatch the action."""
        task = self._pending_task
        if task is None:
            return

        target = self._find_object(task.object_label, list(task.modifiers))

        if target is None:
            if retry_count < 5:
                # Wait 0.5 s and retry (gives vision node time to detect)
                self.get_logger().info(
                    f"Object '{task.object_label}' not found yet — "
                    f"retry {retry_count + 1}/5"
                )
                self.create_timer(
                    0.5,
                    lambda: self._try_execute(retry_count + 1))
                return
            else:
                self.get_logger().error(
                    f"Object '{task.object_label}' not found after 5 retries.")
                self._publish_status(
                    "error",
                    f"Object not found: {task.object_label}"
                )
                self._busy = False
                self._pending_task = None
                return

        # Object found — build enriched command
        self.get_logger().info(
            f"Found {target.label} at "
            f"({target.x:.3f}, {target.y:.3f}, {target.z:.3f})"
        )
        self._publish_status(
            "executing",
            f"Executing: {task.action} → {target.label} at "
            f"({target.x:.2f}, {target.y:.2f}, {target.z:.2f})"
        )

        # Embed the 3D pose inside the TaskCommand (reuse x/y/z fields
        # by publishing with a custom header frame_id carrying the pose)
        exec_cmd = TaskCommand()
        exec_cmd.header              = Header()
        exec_cmd.header.stamp        = self.get_clock().now().to_msg()
        exec_cmd.header.frame_id     = target.header.frame_id
        exec_cmd.action              = task.action
        exec_cmd.object_label        = target.label
        exec_cmd.target_location     = task.target_location
        exec_cmd.raw_command         = task.raw_command
        exec_cmd.modifiers           = task.modifiers

        # Embed pose in a structured way via modifiers (quick trick)
        # Action node reads these back
        exec_cmd.modifiers = list(task.modifiers) + [
            f"__x={target.x:.4f}",
            f"__y={target.y:.4f}",
            f"__z={target.z:.4f}",
        ]

        self.pub_exec.publish(exec_cmd)

    # ------------------------------------------------------------------
    # Object selection logic
    # ------------------------------------------------------------------
    def _find_object(
            self,
            label: str,
            modifiers: list[str]) -> ObjectDetection | None:
        """
        Returns the best matching ObjectDetection for `label`.
        Applies modifier-based selection (smallest, leftmost, etc.).
        """
        with self._lock:
            candidates = [
                d for d in self._detections.get(label, [])
                if d.confidence >= self.min_conf
            ]

        if not candidates:
            return None

        # ---- Modifier-based selection ----
        if "smallest" in modifiers:
            return min(candidates, key=lambda d: d.width * d.height)

        if "largest" in modifiers:
            return max(candidates, key=lambda d: d.width * d.height)

        if "leftmost" in modifiers:
            return min(candidates, key=lambda d: d.x)

        if "rightmost" in modifiers:
            return max(candidates, key=lambda d: d.x)

        if "nearest" in modifiers:
            return min(candidates, key=lambda d: d.z)

        # Default: highest confidence
        return max(candidates, key=lambda d: d.confidence)

    # ------------------------------------------------------------------
    # Action result callback
    # ------------------------------------------------------------------
    def _result_callback(self, msg: ActionResult):
        if msg.success:
            self.get_logger().info(
                f"Task completed: {msg.message} "
                f"(took {msg.execution_time_sec:.2f}s)"
            )
            self._publish_status("idle", f"Done: {msg.message}")
        else:
            self.get_logger().error(
                f"Task failed: {msg.status} — {msg.message}"
            )
            self._publish_status("error", f"Failed: {msg.message}")

        self._busy = False
        self._pending_task = None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _publish_status(self, state: str, detail: str):
        import json
        msg = String()
        msg.data = json.dumps({"state": state, "detail": detail})
        self.pub_status.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TaskCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
