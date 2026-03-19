#!/usr/bin/env python3
"""
VLA Action Node
---------------
Receives enriched TaskCommands from the coordinator and executes them
via MoveIt2 on the Franka Panda arm.

In simulation (use_sim:=true), it runs a mock trajectory without MoveIt2
so you can test the full pipeline in Gazebo without real hardware.

Subscribes:  /execute_action   (vla_interfaces/TaskCommand)
Publishes:   /action_result    (vla_interfaces/ActionResult)

Bin positions (camera frame, metres):
  bin_A  →  (0.70, -0.20, 0.05)
  bin_B  →  (0.70,  0.00, 0.05)
  bin_C  →  (0.70,  0.20, 0.05)
  tray   →  (0.45,  0.10, 0.10)
"""

import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from vla_interfaces.msg import TaskCommand, ActionResult

# MoveIt2 Python bindings — only import if available
try:
    from moveit_py import MoveItPy
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False


# ---------------------------------------------------------------------------
# Known bin positions (in robot base_link frame, metres)
# Adjust these to match your Gazebo / real setup
# ---------------------------------------------------------------------------
BIN_POSITIONS = {
    "bin_A": (0.70, -0.20, 0.05),
    "bin_B": (0.70,  0.00, 0.05),
    "bin_C": (0.70,  0.20, 0.05),
    "tray":  (0.45,  0.10, 0.10),
    "":      (0.70,  0.00, 0.10),   # default drop
}

# Pre-grasp offset: approach from above
PRE_GRASP_Z_OFFSET = 0.12   # metres above object
GRASP_Z_OFFSET     = 0.005  # finger clearance at grasp
POST_GRASP_Z_LIFT  = 0.15   # lift after picking


# ---------------------------------------------------------------------------
# MoveIt2 executor (real/sim with MoveIt2 installed)
# ---------------------------------------------------------------------------
class MoveItExecutor:
    """Wraps MoveItPy for pick-and-place motion planning."""

    def __init__(self, node: Node):
        self.node  = node
        self.robot = MoveItPy(node_name="vla_moveit_py")
        self.arm   = self.robot.get_planning_component("panda_arm")
        self.hand  = self.robot.get_planning_component("hand")
        node.get_logger().info("MoveIt2 executor initialised.")

    def open_gripper(self):
        self.hand.set_start_state_to_current_state()
        self.hand.set_goal_state(configuration_name="open")
        self.hand.plan()
        self.hand.execute()

    def close_gripper(self):
        self.hand.set_start_state_to_current_state()
        self.hand.set_goal_state(configuration_name="close")
        self.hand.plan()
        self.hand.execute()

    def move_to_pose(self, x: float, y: float, z: float) -> bool:
        """Plan and execute a Cartesian move to (x, y, z)."""
        target = Pose()
        target.position    = Point(x=x, y=y, z=z)
        target.orientation = Quaternion(x=0.924, y=-0.383, z=0.0, w=0.0)
        # ^ pointing straight down (roll=π rotation around X)

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=target,
                                pose_link="panda_hand")
        plan = self.arm.plan()
        if not plan:
            return False
        return self.arm.execute()

    def go_home(self):
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name="ready")
        self.arm.plan()
        self.arm.execute()


# ---------------------------------------------------------------------------
# Mock executor (no MoveIt2 / no hardware — full pipeline test)
# ---------------------------------------------------------------------------
class MockExecutor:
    """Simulates motion execution with realistic delays."""

    def __init__(self, node: Node):
        self.node = node

    def open_gripper(self):
        self.node.get_logger().info("[MOCK] Gripper OPEN")
        time.sleep(0.3)

    def close_gripper(self):
        self.node.get_logger().info("[MOCK] Gripper CLOSE")
        time.sleep(0.3)

    def move_to_pose(self, x: float, y: float, z: float) -> bool:
        self.node.get_logger().info(
            f"[MOCK] Planning move → ({x:.3f}, {y:.3f}, {z:.3f})")
        time.sleep(0.8)
        self.node.get_logger().info(
            f"[MOCK] Executing move → ({x:.3f}, {y:.3f}, {z:.3f})")
        time.sleep(1.2)
        return True

    def go_home(self):
        self.node.get_logger().info("[MOCK] Returning to home pose")
        time.sleep(1.0)


# ---------------------------------------------------------------------------
# Action Node
# ---------------------------------------------------------------------------
class ActionNode(Node):

    def __init__(self):
        super().__init__("action_node")

        self.declare_parameter("use_mock", not MOVEIT_AVAILABLE)
        use_mock = self.get_parameter("use_mock").value

        if use_mock:
            self.get_logger().warn(
                "MoveIt2 not available — using MockExecutor. "
                "Set use_mock:=false for real execution."
            )
            self.executor = MockExecutor(self)
        else:
            self.get_logger().info("Using MoveIt2 executor.")
            self.executor = MoveItExecutor(self)

        # ---- Subscribers ----
        self.sub = self.create_subscription(
            TaskCommand, "/execute_action", self._execute_callback, 10)

        # ---- Publishers ----
        self.pub_result = self.create_publisher(ActionResult, "/action_result", 10)

        self.get_logger().info("ActionNode ready.")

    # ------------------------------------------------------------------
    # Parse embedded pose from modifiers
    # ------------------------------------------------------------------
    @staticmethod
    def _extract_pose(modifiers: list) -> tuple[float, float, float] | None:
        """Extract __x=, __y=, __z= values injected by the coordinator."""
        vals = {}
        for m in modifiers:
            if m.startswith("__x="):
                vals["x"] = float(m.split("=")[1])
            elif m.startswith("__y="):
                vals["y"] = float(m.split("=")[1])
            elif m.startswith("__z="):
                vals["z"] = float(m.split("=")[1])
        if len(vals) == 3:
            return vals["x"], vals["y"], vals["z"]
        return None

    # ------------------------------------------------------------------
    # Execute callback
    # ------------------------------------------------------------------
    def _execute_callback(self, msg: TaskCommand):
        t_start = time.time()
        self.get_logger().info(
            f"Executing: {msg.action} | {msg.object_label} → {msg.target_location}"
        )

        pose = self._extract_pose(list(msg.modifiers))
        if pose is None:
            self._publish_result(
                success=False, status="planning_failed",
                message="No object pose in command — coordinator error.",
                action=msg.action, elapsed=0.0
            )
            return

        ox, oy, oz = pose
        target = BIN_POSITIONS.get(msg.target_location, BIN_POSITIONS[""])
        tx, ty, tz = target

        success = False
        try:
            if msg.action in ("pick", "pick_and_place"):
                success = self._do_pick(ox, oy, oz)

            if msg.action == "pick_and_place" and success:
                success = self._do_place(tx, ty, tz)

            elif msg.action == "place":
                success = self._do_place(tx, ty, tz)

            elif msg.action == "inspect":
                success = self._do_inspect(ox, oy, oz)

            elif msg.action in ("count", "unknown"):
                # These are handled by vision, not action
                self._publish_result(
                    success=True, status="completed",
                    message=f"Action '{msg.action}' is a vision query — no motion needed.",
                    action=msg.action, elapsed=0.0
                )
                return

            # Return home
            self.executor.go_home()

        except Exception as e:
            self.get_logger().error(f"Execution error: {e}")
            success = False

        elapsed = time.time() - t_start

        if success:
            self._publish_result(
                success=True, status="completed",
                message=(
                    f"{msg.action} '{msg.object_label}' "
                    f"→ {msg.target_location} completed."
                ),
                action=msg.action, elapsed=elapsed
            )
        else:
            self._publish_result(
                success=False, status="execution_failed",
                message=f"Failed to execute {msg.action} for {msg.object_label}.",
                action=msg.action, elapsed=elapsed
            )

    # ------------------------------------------------------------------
    # Motion primitives
    # ------------------------------------------------------------------
    def _do_pick(self, ox: float, oy: float, oz: float) -> bool:
        """Standard pick sequence: pre-grasp → open → descend → close → lift."""
        self.get_logger().info(
            f"Pick sequence at ({ox:.3f}, {oy:.3f}, {oz:.3f})")

        # 1. Pre-grasp (above object)
        if not self.executor.move_to_pose(ox, oy, oz + PRE_GRASP_Z_OFFSET):
            self.get_logger().error("Pre-grasp move failed.")
            return False

        # 2. Open gripper
        self.executor.open_gripper()

        # 3. Descend to grasp height
        if not self.executor.move_to_pose(ox, oy, oz + GRASP_Z_OFFSET):
            self.get_logger().error("Grasp descend failed.")
            return False

        # 4. Close gripper
        self.executor.close_gripper()

        # 5. Lift
        if not self.executor.move_to_pose(ox, oy, oz + POST_GRASP_Z_LIFT):
            self.get_logger().error("Post-grasp lift failed.")
            return False

        self.get_logger().info("Pick sequence complete.")
        return True

    def _do_place(self, tx: float, ty: float, tz: float) -> bool:
        """Place sequence: move to bin → lower → open → lift."""
        self.get_logger().info(
            f"Place sequence at ({tx:.3f}, {ty:.3f}, {tz:.3f})")

        # 1. Move to above bin
        if not self.executor.move_to_pose(tx, ty, tz + PRE_GRASP_Z_OFFSET):
            return False

        # 2. Lower into bin
        if not self.executor.move_to_pose(tx, ty, tz + GRASP_Z_OFFSET + 0.02):
            return False

        # 3. Open gripper (release)
        self.executor.open_gripper()

        # 4. Lift clear
        if not self.executor.move_to_pose(tx, ty, tz + POST_GRASP_Z_LIFT):
            return False

        self.get_logger().info("Place sequence complete.")
        return True

    def _do_inspect(self, ox: float, oy: float, oz: float) -> bool:
        """Move to inspection pose above object."""
        return self.executor.move_to_pose(ox, oy, oz + 0.25)

    # ------------------------------------------------------------------
    # Result publisher
    # ------------------------------------------------------------------
    def _publish_result(
            self, success: bool, status: str,
            message: str, action: str, elapsed: float):
        msg = ActionResult()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.success             = success
        msg.status              = status
        msg.message             = message
        msg.action_performed    = action
        msg.execution_time_sec  = float(elapsed)
        self.pub_result.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
