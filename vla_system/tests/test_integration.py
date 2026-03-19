#!/usr/bin/env python3
"""
VLA Integration Test — simulates the full pipeline without ROS2.

Wires language → coordinator → action together in-process,
using mock objects instead of live ROS2 topics.

Usage:
  python3 tests/test_integration.py
"""

import sys, os, types, time, json

# ─────────────────────────────────────────────────────────────────────────────
# ROS2 stubs (same as test_language_parser.py)
# ─────────────────────────────────────────────────────────────────────────────

def _make_stub(*names):
    mod = types.ModuleType('stub')
    for n in names:
        setattr(mod, n, type(n, (), {
            '__init__': lambda self, *a, **kw: None,
            '__setattr__': object.__setattr__,
        }))
    return mod

class _Logger:
    def __init__(self, prefix=""):  self.prefix = prefix
    def info(self, m, *a, **kw):   print(f"  [INFO]  {self.prefix}{m}")
    def warn(self, m, *a, **kw):   print(f"  [WARN]  {self.prefix}{m}")
    def error(self, m, *a, **kw):  print(f"  [ERROR] {self.prefix}{m}")

class _NodeBase:
    def __init__(self, name="node"):
        self._name   = name
        self._logger = _Logger(f"[{name}] ")
        self._params = {}
    def get_logger(self):                return self._logger
    def declare_parameter(self, k, v):   self._params[k] = v
    def get_parameter(self, k):
        class _P:
            value = self._params.get(k, "")
        _P.value = self._params.get(k, "")
        return _P()
    def create_publisher(self, *a, **kw):  return None
    def create_subscription(self, *a, **kw): return None
    def create_timer(self, *a, **kw):      return None
    def get_clock(self):                   return self
    def now(self):                         return self
    def to_msg(self):                      return None

_node_mod = types.ModuleType('rclpy.node'); _node_mod.Node = _NodeBase
_qos_mod  = types.ModuleType('rclpy.qos')
_qos_mod.QoSProfile = lambda **kw: None
_qos_mod.ReliabilityPolicy = type('RP', (), {'BEST_EFFORT': 1})()

_rclpy = types.ModuleType('rclpy')
_rclpy.init = lambda *a, **kw: None
_rclpy.shutdown = lambda: None
_rclpy.spin = lambda n: None
_rclpy.time = types.ModuleType('rclpy.time')
_rclpy.node = _node_mod

for _k, _v in [
    ('rclpy',               _rclpy),
    ('rclpy.node',          _node_mod),
    ('rclpy.qos',           _qos_mod),
    ('rclpy.time',          types.ModuleType('rclpy.time')),
    ('rclpy.callback_groups', types.ModuleType('rclpy.callback_groups')),
    ('rclpy.executors',     types.ModuleType('rclpy.executors')),
    ('std_msgs',            types.ModuleType('std_msgs')),
    ('std_msgs.msg',        _make_stub('String', 'Header')),
    ('vla_interfaces',      types.ModuleType('vla_interfaces')),
    ('vla_interfaces.msg',  _make_stub('TaskCommand', 'ObjectDetection', 'ActionResult')),
    ('message_filters',     types.ModuleType('message_filters')),
    ('sensor_msgs',         types.ModuleType('sensor_msgs')),
    ('sensor_msgs.msg',     _make_stub('Image', 'CameraInfo')),
    ('cv_bridge',           types.ModuleType('cv_bridge')),
    ('geometry_msgs',       types.ModuleType('geometry_msgs')),
    ('geometry_msgs.msg',   _make_stub('Pose', 'Point', 'Quaternion')),
]:
    sys.modules.setdefault(_k, _v)

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, 'language_node'))
sys.path.insert(0, os.path.join(_ROOT, 'action_node'))
sys.path.insert(0, os.path.join(_ROOT, 'coordinator_node'))

from language_node.language_node import parse_rule_based
from action_node.action_node import MockExecutor, BIN_POSITIONS


# ─────────────────────────────────────────────────────────────────────────────
# Lightweight in-process pipeline
# ─────────────────────────────────────────────────────────────────────────────

class FakeDetection:
    """Mimics vla_interfaces/ObjectDetection."""
    def __init__(self, label, x, y, z, conf=0.90, w=60.0, h=40.0):
        self.label      = label
        self.x          = x
        self.y          = y
        self.z          = z
        self.confidence = conf
        self.width      = w
        self.height     = h
        self.header     = type('H', (), {'stamp': None, 'frame_id': 'camera_color_optical_frame'})()

class FakeTaskCommand:
    """Mimics vla_interfaces/TaskCommand."""
    def __init__(self, action, object_label, target_location, modifiers=None, raw=""):
        self.action          = action
        self.object_label    = object_label
        self.target_location = target_location
        self.modifiers       = modifiers or []
        self.raw_command     = raw
        self.header          = type('H', (), {'stamp': None, 'frame_id': ''})()

class FakeActionResult:
    """Mimics vla_interfaces/ActionResult."""
    def __init__(self, success, status, message, elapsed=0.0):
        self.success             = success
        self.status              = status
        self.message             = message
        self.execution_time_sec  = elapsed
        self.action_performed    = ""


class InProcessPipeline:
    """
    Wires language → coordinator-logic → action together without any ROS2.
    Used to validate the full control flow in a unit-test context.
    """

    def __init__(self, detections: list[FakeDetection]):
        self._detections = {d.label: d for d in detections}
        self._node       = _NodeBase("test_pipeline")
        self._executor   = MockExecutor(self._node)
        self._log        = _Logger("[pipeline] ")
        self._executed   = []   # list of (action, object_label, target)

    def run(self, command: str) -> dict:
        """
        Run one command through the full pipeline.
        Returns a result dict with keys: success, action, object, target, message.
        """
        self._log.info(f"Command: '{command}'")

        # ── Phase 1: Language ────────────────────────────────────────
        parsed = parse_rule_based(command)
        self._log.info(
            f"Parsed → action={parsed['action']} "
            f"obj={parsed['object_label']} "
            f"target={parsed['target_location']} "
            f"conf={parsed['confidence']:.2f}"
        )

        if parsed['action'] == 'unknown' or not parsed['object_label']:
            return {'success': False, 'reason': 'parse_failed',
                    'action': parsed['action'], 'object': '', 'target': ''}

        # ── Phase 2: Vision lookup ───────────────────────────────────
        detection = self._find_object(parsed['object_label'], parsed.get('modifiers', []))
        if detection is None:
            return {'success': False, 'reason': 'object_not_found',
                    'action': parsed['action'],
                    'object': parsed['object_label'],
                    'target': parsed['target_location']}
        self._log.info(
            f"Found {detection.label} at "
            f"({detection.x:.3f}, {detection.y:.3f}, {detection.z:.3f})"
        )

        # ── Phase 3: Action ──────────────────────────────────────────
        result = self._execute(parsed['action'], detection, parsed['target_location'])
        self._executed.append((parsed['action'], detection.label, parsed['target_location']))
        return result

    def _find_object(self, label: str, modifiers: list):
        candidates = [d for d in self._detections.values()
                      if d.label == label and d.confidence >= 0.6]
        if not candidates:
            return None
        if 'smallest' in modifiers:
            return min(candidates, key=lambda d: d.width * d.height)
        if 'largest' in modifiers:
            return max(candidates, key=lambda d: d.width * d.height)
        if 'nearest' in modifiers:
            return min(candidates, key=lambda d: d.z)
        return max(candidates, key=lambda d: d.confidence)

    def _execute(self, action: str, det: FakeDetection, target: str) -> dict:
        ox, oy, oz = det.x, det.y, det.z
        tx, ty, tz = BIN_POSITIONS.get(target, BIN_POSITIONS[''])

        ok = True
        if action in ('pick', 'pick_and_place'):
            ok = ok and self._executor.move_to_pose(ox, oy, oz + 0.12)
            self._executor.open_gripper()
            ok = ok and self._executor.move_to_pose(ox, oy, oz + 0.005)
            self._executor.close_gripper()
            ok = ok and self._executor.move_to_pose(ox, oy, oz + 0.15)
        if action == 'pick_and_place' and ok:
            ok = ok and self._executor.move_to_pose(tx, ty, tz + 0.12)
            ok = ok and self._executor.move_to_pose(tx, ty, tz + 0.025)
            self._executor.open_gripper()
            ok = ok and self._executor.move_to_pose(tx, ty, tz + 0.15)
        if action == 'inspect':
            ok = self._executor.move_to_pose(ox, oy, oz + 0.25)

        self._executor.go_home()

        return {
            'success': ok,
            'reason': 'completed' if ok else 'execution_failed',
            'action': action,
            'object': det.label,
            'target': target,
        }


# ─────────────────────────────────────────────────────────────────────────────
# Test cases
# ─────────────────────────────────────────────────────────────────────────────

SCENE = [
    FakeDetection("screw",  x=0.30, y=0.10, z=0.48, conf=0.91, w=60, h=40),
    FakeDetection("screw",  x=0.35, y=0.05, z=0.48, conf=0.85, w=40, h=25),  # smaller
    FakeDetection("rivet",  x=0.38, y=-0.12, z=0.52, conf=0.87, w=50, h=35),
    FakeDetection("washer", x=0.32, y=-0.02, z=0.50, conf=0.78, w=80, h=80),
    FakeDetection("bolt",   x=0.48, y=0.10, z=0.46, conf=0.83, w=65, h=45),
]

_passed = 0
_failed = 0


def run_test(name: str, command: str, expect_success: bool,
             expect_action=None, expect_object=None, expect_target=None):
    global _passed, _failed
    print(f"\n  ▶  {name}")
    print(f"     cmd: \"{command}\"")

    pipe = InProcessPipeline(SCENE)
    result = pipe.run(command)

    ok = True
    if result['success'] != expect_success:
        print(f"     ✗ success: expected {expect_success}, got {result['success']}")
        ok = False
    if expect_action and result.get('action') != expect_action:
        print(f"     ✗ action: expected '{expect_action}', got '{result.get('action')}'")
        ok = False
    if expect_object and result.get('object') != expect_object:
        print(f"     ✗ object: expected '{expect_object}', got '{result.get('object')}'")
        ok = False
    if expect_target and result.get('target') != expect_target:
        print(f"     ✗ target: expected '{expect_target}', got '{result.get('target')}'")
        ok = False

    if ok:
        print(f"     ✓ PASS  (result: {result['reason']})")
        _passed += 1
    else:
        print(f"     ✗ FAIL")
        _failed += 1
    return ok


def test_modifier_smallest():
    """Smallest-modifier should pick the second screw (w=40×h=25=1000 vs 60×40=2400)."""
    print(f"\n  ▶  smallest modifier selects correct object")
    print(f'     cmd: "Pick the smallest screw"')
    pipe = InProcessPipeline(SCENE)
    parsed = parse_rule_based("Pick the smallest screw")
    det = pipe._find_object("screw", parsed.get("modifiers", []))
    ok = det is not None and det.width * det.height < 1500
    status = "✓ PASS" if ok else "✗ FAIL"
    print(f"     {status}  (selected screw area={det.width * det.height:.0f}px²)")
    global _passed, _failed
    if ok: _passed += 1
    else:  _failed += 1


def test_object_not_in_scene():
    print(f"\n  ▶  object not in scene → object_not_found")
    print(f'     cmd: "Pick the nut"')
    pipe = InProcessPipeline(SCENE)   # no nut in scene
    result = pipe.run("Pick the nut")
    ok = not result['success'] and result['reason'] == 'object_not_found'
    status = "✓ PASS" if ok else "✗ FAIL"
    print(f"     {status}  (reason: {result['reason']})")
    global _passed, _failed
    if ok: _passed += 1
    else:  _failed += 1


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    print()
    print("═" * 58)
    print("  VLA Integration Test — Full Pipeline (no ROS2)")
    print("═" * 58)
    print(f"\n  Scene: {len(SCENE)} objects  "
          f"({', '.join(d.label for d in SCENE)})")

    print("\n\n── Scenario 1: Basic pick ──────────────────────────────")
    run_test("Pick single object",
             "Pick the screw",
             expect_success=True, expect_action="pick", expect_object="screw")

    print("\n\n── Scenario 2: Pick and place ──────────────────────────")
    run_test("Pick and place to bin A",
             "Pick the screw and place it in bin A",
             expect_success=True, expect_action="pick_and_place",
             expect_object="screw", expect_target="bin_A")

    run_test("Put rivet in bin C",
             "Put the rivet in bin C",
             expect_success=True, expect_action="pick_and_place",
             expect_object="rivet", expect_target="bin_C")

    print("\n\n── Scenario 3: Modifier-based selection ────────────────")
    test_modifier_smallest()

    run_test("Nearest screw",
             "Pick the nearest screw",
             expect_success=True, expect_action="pick", expect_object="screw")

    print("\n\n── Scenario 4: Inspect ─────────────────────────────────")
    run_test("Inspect washer",
             "Inspect the washer",
             expect_success=True, expect_action="inspect", expect_object="washer")

    print("\n\n── Scenario 5: Error handling ──────────────────────────")
    test_object_not_in_scene()

    run_test("Completely ambiguous command",
             "do the thing",
             expect_success=False)

    # ── Summary ──────────────────────────────────────────────────────
    total = _passed + _failed
    print()
    print("═" * 58)
    if _failed == 0:
        print(f"  ✓  All {total} integration tests passed.")
    else:
        print(f"  ✗  {_passed}/{total} passed — {_failed} FAILED")
    print("═" * 58)
    print()
    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
