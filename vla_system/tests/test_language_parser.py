#!/usr/bin/env python3
"""
VLA Language Parser — Test Suite
Runs standalone with no ROS2 installation required.

Usage:
  python3 tests/test_language_parser.py          # from project root
  python3 -m pytest tests/test_language_parser.py -v
"""

import sys, os, types

# ─────────────────────────────────────────────────────────────────────────────
# ROS2 stubs — lets us import the language node without a ROS install
# ─────────────────────────────────────────────────────────────────────────────

def _make_msg_stub(*names):
    mod = types.ModuleType('msg_stub')
    for n in names:
        setattr(mod, n, type(n, (), {'__init__': lambda self, **kw: None}))
    return mod

class _Node:
    def __init__(self, *a, **kw): pass
    def get_logger(self): return self
    def info(self, *a, **kw): pass
    def warn(self, *a, **kw): pass
    def error(self, *a, **kw): pass
    def declare_parameter(self, *a, **kw): pass
    def get_parameter(self, name):
        class _P:
            value = ""
        return _P()
    def create_publisher(self, *a, **kw): return None
    def create_subscription(self, *a, **kw): return None
    def create_timer(self, *a, **kw): return None
    def get_clock(self): return self
    def now(self): return self
    def to_msg(self): return None

_node_mod = types.ModuleType('rclpy.node'); _node_mod.Node = _Node
_qos_mod  = types.ModuleType('rclpy.qos')
_qos_mod.QoSProfile = lambda **kw: None
_qos_mod.ReliabilityPolicy = type('RP', (), {'BEST_EFFORT': 1})()

_rclpy = types.ModuleType('rclpy')
_rclpy.init = lambda *a, **kw: None
_rclpy.shutdown = lambda: None
_rclpy.spin = lambda n: None
_rclpy.node = _node_mod

for _k, _v in [
    ('rclpy',                  _rclpy),
    ('rclpy.node',             _node_mod),
    ('rclpy.qos',              _qos_mod),
    ('rclpy.time',             types.ModuleType('rclpy.time')),
    ('rclpy.callback_groups',  types.ModuleType('rclpy.callback_groups')),
    ('rclpy.executors',        types.ModuleType('rclpy.executors')),
    ('std_msgs',               types.ModuleType('std_msgs')),
    ('std_msgs.msg',           _make_msg_stub('String', 'Header', 'Bool')),
    ('vla_interfaces',         types.ModuleType('vla_interfaces')),
    ('vla_interfaces.msg',     _make_msg_stub('TaskCommand', 'ObjectDetection', 'ActionResult')),
    ('message_filters',        types.ModuleType('message_filters')),
    ('sensor_msgs',            types.ModuleType('sensor_msgs')),
    ('sensor_msgs.msg',        _make_msg_stub('Image', 'CameraInfo')),
    ('cv_bridge',              types.ModuleType('cv_bridge')),
    ('geometry_msgs',          types.ModuleType('geometry_msgs')),
    ('geometry_msgs.msg',      _make_msg_stub('Pose', 'Point', 'Quaternion')),
]:
    sys.modules.setdefault(_k, _v)

# Resolve language_node package path relative to this test file
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, 'language_node'))

from language_node.language_node import parse_rule_based  # noqa: E402

# ─────────────────────────────────────────────────────────────────────────────
# Test helpers
# ─────────────────────────────────────────────────────────────────────────────

_results = []


def check(description, text, expected_action,
          expected_object, expected_target="", expected_mods=None):
    result  = parse_rule_based(text)
    ok      = True
    reasons = []

    if result["action"] != expected_action:
        reasons.append(
            f"action: got '{result['action']}', expected '{expected_action}'")
        ok = False

    if result["object_label"] != expected_object:
        reasons.append(
            f"object: got '{result['object_label']}', expected '{expected_object}'")
        ok = False

    if expected_target and result["target_location"] != expected_target:
        reasons.append(
            f"target: got '{result['target_location']}', expected '{expected_target}'")
        ok = False

    if expected_mods:
        for mod in expected_mods:
            if mod not in result["modifiers"]:
                reasons.append(
                    f"modifier '{mod}' missing — got {result['modifiers']}")
                ok = False

    icon = "✓" if ok else "✗"
    print(f"  {icon}  {description}")
    for r in reasons:
        print(f"       └─ {r}")

    _results.append(ok)
    return ok


# pytest-compatible wrappers (each is a standalone test_ function)
def _assert(description, text, exp_action, exp_obj, exp_target="", exp_mods=None):
    result = parse_rule_based(text)
    assert result["action"] == exp_action, \
        f"{description}: action got '{result['action']}'"
    assert result["object_label"] == exp_obj, \
        f"{description}: object got '{result['object_label']}'"
    if exp_target:
        assert result["target_location"] == exp_target, \
            f"{description}: target got '{result['target_location']}'"
    if exp_mods:
        for mod in exp_mods:
            assert mod in result["modifiers"], \
                f"{description}: modifier '{mod}' missing"


# ─────────────────────────────────────────────────────────────────────────────
# Tests — basic pick
# ─────────────────────────────────────────────────────────────────────────────

def test_simple_pick():           _assert("simple pick",      "Pick the screw",       "pick", "screw")
def test_pick_up():               _assert("pick up",          "Pick up the rivet",    "pick", "rivet")
def test_grab_synonym():          _assert("grab",             "Grab the bolt",        "pick", "bolt")
def test_take_synonym():          _assert("take",             "Take the washer",      "pick", "washer")
def test_plural_form():           _assert("plural",           "Pick the screws",      "pick", "screw")
def test_grasp_synonym():         _assert("grasp",            "Grasp the nut",        "pick", "nut")

# ─────────────────────────────────────────────────────────────────────────────
# Tests — pick and place
# ─────────────────────────────────────────────────────────────────────────────

def test_pick_and_place_basic():
    _assert("canonical", "Pick the screw and place it in bin A",
            "pick_and_place", "screw", "bin_A")

def test_put_in_bin():
    _assert("put in bin", "Put the bolt in bin B",
            "pick_and_place", "bolt", "bin_B")

def test_move_to_bin():
    _assert("move to", "Move the rivet to bin C",
            "pick_and_place", "rivet", "bin_C")

def test_grab_and_place():
    _assert("grab and place", "Grab the nut and place it in bin A",
            "pick_and_place", "nut", "bin_A")

def test_transfer_to():
    _assert("transfer to", "Transfer the washer to bin B",
            "pick_and_place", "washer", "bin_B")

def test_bin_number_alias():
    _assert("bin 1 = bin_A", "Pick the screw and put it in bin 1",
            "pick_and_place", "screw", "bin_A")

def test_bin_2_alias():
    _assert("bin 2 = bin_B", "Put the bolt in bin 2",
            "pick_and_place", "bolt", "bin_B")

# ─────────────────────────────────────────────────────────────────────────────
# Tests — modifiers
# ─────────────────────────────────────────────────────────────────────────────

def test_smallest_modifier():
    _assert("smallest", "Pick the smallest screw",
            "pick", "screw", expected_mods=["smallest"])

def test_largest_modifier():
    _assert("largest/biggest", "Pick the biggest bolt",
            "pick", "bolt", expected_mods=["largest"])

def test_leftmost_modifier():
    _assert("leftmost", "Pick the leftmost rivet",
            "pick", "rivet", expected_mods=["leftmost"])

def test_nearest_modifier():
    _assert("nearest/closest", "Pick the closest screw",
            "pick", "screw", expected_mods=["nearest"])

def test_modifier_with_target():
    _assert("modifier + target",
            "Pick the smallest screw and place it in bin A",
            "pick_and_place", "screw", "bin_A", expected_mods=["smallest"])

def test_rightmost_modifier():
    _assert("rightmost", "Pick the rightmost bolt",
            "pick", "bolt", expected_mods=["rightmost"])

# ─────────────────────────────────────────────────────────────────────────────
# Tests — inspect / count
# ─────────────────────────────────────────────────────────────────────────────

def test_inspect():        _assert("inspect",  "Inspect the washer",  "inspect", "washer")
def test_check_synonym():  _assert("check",    "Check the screw",     "inspect", "screw")
def test_examine_synonym(): _assert("examine", "Examine the bolt",    "inspect", "bolt")
def test_how_many():       _assert("how many", "How many screws are there?", "count", "screw")
def test_count_explicit(): _assert("count",    "Count the rivets",    "count",   "rivet")

# ─────────────────────────────────────────────────────────────────────────────
# Tests — edge cases
# ─────────────────────────────────────────────────────────────────────────────

def test_lowercase():      _assert("lowercase", "pick the screw",     "pick", "screw")
def test_uppercase():      _assert("uppercase", "PICK THE SCREW AND PLACE IN BIN A",
                                   "pick_and_place", "screw", "bin_A")
def test_extra_whitespace(): _assert("whitespace", "  Pick   the   screw  ", "pick", "screw")

def test_unknown_object():
    result = parse_rule_based("Pick the foobar")
    assert result["object_label"] == "", \
        f"Expected empty object_label, got '{result['object_label']}'"


# ─────────────────────────────────────────────────────────────────────────────
# Confidence score sanity checks
# ─────────────────────────────────────────────────────────────────────────────

def test_confidence_high_for_full_command():
    result = parse_rule_based("Pick the screw and place it in bin A")
    assert result["confidence"] >= 0.9, \
        f"Expected conf >= 0.9, got {result['confidence']}"

def test_confidence_zero_for_garbage():
    result = parse_rule_based("the quick brown fox")
    assert result["confidence"] < 0.5, \
        f"Expected conf < 0.5 for nonsense, got {result['confidence']}"

def test_confidence_partial_for_pick_only():
    result = parse_rule_based("Pick the screw")
    assert 0.5 <= result["confidence"] <= 0.95, \
        f"Expected mid-range conf, got {result['confidence']}"


# ─────────────────────────────────────────────────────────────────────────────
# Standalone runner (without pytest)
# ─────────────────────────────────────────────────────────────────────────────

def _run_all_as_cli():
    sections = [
        ("Basic pick",          [
            ("simple pick",         "Pick the screw",       "pick", "screw"),
            ("pick up",             "Pick up the rivet",    "pick", "rivet"),
            ("grab synonym",        "Grab the bolt",        "pick", "bolt"),
            ("take synonym",        "Take the washer",      "pick", "washer"),
            ("plural form",         "Pick the screws",      "pick", "screw"),
            ("grasp synonym",       "Grasp the nut",        "pick", "nut"),
        ]),
        ("Pick and place",      [
            ("canonical",           "Pick the screw and place it in bin A",     "pick_and_place", "screw",  "bin_A"),
            ("put in bin",          "Put the bolt in bin B",                    "pick_and_place", "bolt",   "bin_B"),
            ("move to",             "Move the rivet to bin C",                  "pick_and_place", "rivet",  "bin_C"),
            ("grab and place",      "Grab the nut and place it in bin A",       "pick_and_place", "nut",    "bin_A"),
            ("transfer to",         "Transfer the washer to bin B",             "pick_and_place", "washer", "bin_B"),
            ("bin 1 = bin_A",       "Pick the screw and put it in bin 1",       "pick_and_place", "screw",  "bin_A"),
        ]),
        ("Modifiers",           [
            ("smallest",        "Pick the smallest screw",             "pick", "screw",  "", ["smallest"]),
            ("largest/biggest", "Pick the biggest bolt",               "pick", "bolt",   "", ["largest"]),
            ("leftmost",        "Pick the leftmost rivet",             "pick", "rivet",  "", ["leftmost"]),
            ("nearest/closest", "Pick the closest screw",              "pick", "screw",  "", ["nearest"]),
            ("mod + target",    "Pick the smallest screw and place it in bin A",
                                                        "pick_and_place", "screw", "bin_A", ["smallest"]),
        ]),
        ("Inspect / count",     [
            ("inspect",         "Inspect the washer",        "inspect", "washer"),
            ("check",           "Check the screw",           "inspect", "screw"),
            ("examine",         "Examine the bolt",          "inspect", "bolt"),
            ("how many",        "How many screws are there?","count",   "screw"),
            ("count",           "Count the rivets",          "count",   "rivet"),
        ]),
        ("Edge cases",          [
            ("lowercase",       "pick the screw",                           "pick",           "screw"),
            ("uppercase",       "PICK THE SCREW AND PLACE IN BIN A",        "pick_and_place", "screw", "bin_A"),
            ("whitespace",      "  Pick   the   screw  ",                   "pick",           "screw"),
            ("unknown object",  "Pick the foobar",                          "pick",           ""),
        ]),
    ]

    print()
    print("═" * 58)
    print("  VLA Language Parser — Test Suite")
    print("═" * 58)

    for section_name, cases in sections:
        print(f"\n── {section_name} ──")
        for case in cases:
            desc, text = case[0], case[1]
            exp_action, exp_obj = case[2], case[3]
            exp_target = case[4] if len(case) > 4 else ""
            exp_mods   = case[5] if len(case) > 5 else None
            check(desc, text, exp_action, exp_obj, exp_target, exp_mods)

    total  = len(_results)
    passed = sum(_results)
    failed = total - passed

    print()
    print("═" * 58)
    if failed == 0:
        print(f"  ✓  All {total} tests passed.")
    else:
        print(f"  ✗  {passed}/{total} passed — {failed} FAILED")
    print("═" * 58)
    print()
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(_run_all_as_cli())
