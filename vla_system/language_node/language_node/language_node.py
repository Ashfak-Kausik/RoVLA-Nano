#!/usr/bin/env python3
"""
VLA Language Node
-----------------
Converts natural language commands into structured TaskCommand messages.

Two parsing modes (selected by parameter):
  - "rule"  : Deterministic regex/keyword parser (fast, no API key needed)
  - "llm"   : Claude API for complex / ambiguous commands (requires ANTHROPIC_API_KEY)

Subscribes:  /raw_command  (std_msgs/String)
Publishes:   /task_command (vla_interfaces/TaskCommand)

Example input → output:
  "Pick the screw and place it in bin A"
  → { action: pick_and_place, object_label: screw, target_location: bin_A }
"""

import re
import os
import json
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header
from vla_interfaces.msg import TaskCommand


# ---------------------------------------------------------------------------
# Rule-based parser
# ---------------------------------------------------------------------------
OBJECT_VOCAB = {
    "screw": "screw", "screws": "screw",
    "rivet": "rivet", "rivets": "rivet",
    "bolt": "bolt",   "bolts": "bolt",
    "nut": "nut",     "nuts": "nut",
    "washer": "washer", "washers": "washer",
    "fastener": "fastener",
}

BIN_VOCAB = {
    "bin a": "bin_A", "bin_a": "bin_A", "bin 1": "bin_A",
    "bin b": "bin_B", "bin_b": "bin_B", "bin 2": "bin_B",
    "bin c": "bin_C", "bin_c": "bin_C", "bin 3": "bin_C",
    "tray": "tray",
}

ACTION_PATTERNS = {
    "pick_and_place": [
        r"pick.*(?:and|then|&).*place",
        r"pick.*(?:and|then|&).*put",
        r"grab.*(?:and|then|&).*place",
        r"move.*to\s+bin",
        r"transfer.*to",
        r"put.*in(?:to)?\s+bin",
    ],
    "pick": [
        r"^pick(?:\s+up)?\s+",
        r"^grab\s+",
        r"^grasp\s+",
        r"^take\s+",
    ],
    "place": [
        r"^place\s+",
        r"^put\s+.+\s+in(?:to)?\s+",
        r"^drop\s+",
    ],
    "inspect": [
        r"^inspect\s+",
        r"^check\s+",
        r"^look at\s+",
        r"^examine\s+",
    ],
    "count": [
        r"how many",
        r"count\s+",
        r"^list\s+",
    ],
}

MODIFIER_PATTERNS = {
    "smallest":  r"\bsmallest\b",
    "largest":   r"\blargest\b|\bbiggest\b",
    "leftmost":  r"\bleftmost\b|\bleft\s*most\b|\bfar\s+left\b",
    "rightmost": r"\brightmost\b|\bright\s*most\b|\bfar\s+right\b",
    "nearest":   r"\bnearest\b|\bclosest\b",
    "first":     r"\bfirst\b",
}


def parse_rule_based(text: str) -> dict:
    """
    Parse a natural language command into a structured dict.
    Returns:
        action, object_label, target_location, modifiers, confidence
    """
    txt = text.lower().strip()
    result = {
        "action": "unknown",
        "object_label": "",
        "target_location": "",
        "modifiers": [],
        "confidence": 0.0,
    }

    # ---- Detect action ----
    for action, patterns in ACTION_PATTERNS.items():
        for pat in patterns:
            if re.search(pat, txt):
                result["action"] = action
                result["confidence"] += 0.4
                break
        if result["action"] != "unknown":
            break

    # ---- Detect object ----
    for word, canonical in OBJECT_VOCAB.items():
        if re.search(r'\b' + word + r'\b', txt):
            result["object_label"] = canonical
            result["confidence"] += 0.4
            break

    # ---- Detect bin / target ----
    for phrase, canonical in BIN_VOCAB.items():
        if phrase in txt:
            result["target_location"] = canonical
            result["confidence"] += 0.2
            break

    # ---- Detect modifiers ----
    for mod, pat in MODIFIER_PATTERNS.items():
        if re.search(pat, txt):
            result["modifiers"].append(mod)

    # ---- Clamp confidence ----
    result["confidence"] = min(result["confidence"], 1.0)

    return result


# ---------------------------------------------------------------------------
# LLM-based parser (optional upgrade)
# ---------------------------------------------------------------------------
SYSTEM_PROMPT = """You are a robot command parser. Convert natural language instructions into JSON.

Return ONLY a JSON object with these fields (no markdown, no explanation):
{
  "action": "pick" | "place" | "pick_and_place" | "inspect" | "count" | "unknown",
  "object_label": "<object name, singular lowercase, e.g. screw>",
  "target_location": "<destination, e.g. bin_A or empty string>",
  "modifiers": ["<modifier>", ...],
  "confidence": <0.0-1.0>
}

Examples:
  Input: "Pick the smallest screw and place it in bin A"
  Output: {"action":"pick_and_place","object_label":"screw","target_location":"bin_A","modifiers":["smallest"],"confidence":0.97}

  Input: "How many rivets are there?"
  Output: {"action":"count","object_label":"rivet","target_location":"","modifiers":[],"confidence":0.95}
"""


def parse_with_llm(text: str, logger=None) -> dict:
    """Call Claude API to parse the command. Falls back to rule-based on error."""
    try:
        import anthropic
        client = anthropic.Anthropic()
        response = client.messages.create(
            model="claude-3-sonnet-20240229",
            max_tokens=256,
            system=SYSTEM_PROMPT,
            messages=[{"role": "user", "content": text}],
        )
        raw = response.content[0].text.strip()
        parsed = json.loads(raw)
        # Ensure all fields present
        parsed.setdefault("action", "unknown")
        parsed.setdefault("object_label", "")
        parsed.setdefault("target_location", "")
        parsed.setdefault("modifiers", [])
        parsed.setdefault("confidence", 0.8)
        return parsed
    except Exception as e:
        if logger:
            logger.warn(f"LLM parse failed ({e}), falling back to rule-based.")
        return parse_rule_based(text)


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------
class LanguageNode(Node):

    def __init__(self):
        super().__init__("language_node")

        self.declare_parameter("parser_mode", "rule")  # "rule" or "llm"
        self.parser_mode = self.get_parameter("parser_mode").value

        if self.parser_mode == "llm":
            if not os.environ.get("ANTHROPIC_API_KEY"):
                self.get_logger().warn(
                    "parser_mode=llm but ANTHROPIC_API_KEY not set. "
                    "Falling back to rule-based."
                )
                self.parser_mode = "rule"
            else:
                self.get_logger().info("Language node using LLM (Claude) parser.")
        else:
            self.get_logger().info("Language node using rule-based parser.")

        # ---- Subscribers ----
        self.sub = self.create_subscription(
            String, "/raw_command", self._command_callback, 10)

        # ---- Publishers ----
        self.pub = self.create_publisher(
            TaskCommand, "/task_command", 10)

        self.get_logger().info("LanguageNode ready. Send commands on /raw_command.")

    def _command_callback(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"Received command: '{text}'")

        # ---- Parse ----
        if self.parser_mode == "llm":
            parsed = parse_with_llm(text, self.get_logger())
        else:
            parsed = parse_rule_based(text)

        # ---- Build TaskCommand ----
        cmd = TaskCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.action          = parsed["action"]
        cmd.object_label    = parsed["object_label"]
        cmd.target_location = parsed["target_location"]
        cmd.raw_command     = text
        cmd.modifiers       = parsed.get("modifiers", [])

        self.get_logger().info(
            f"Parsed → action={cmd.action} "
            f"object={cmd.object_label} "
            f"target={cmd.target_location} "
            f"mods={list(cmd.modifiers)} "
            f"conf={parsed.get('confidence', 0):.2f}"
        )

        if parsed["action"] == "unknown" or not parsed["object_label"]:
            self.get_logger().warn(
                "Could not parse command confidently. Check your phrasing."
            )

        self.pub.publish(cmd)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = LanguageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
