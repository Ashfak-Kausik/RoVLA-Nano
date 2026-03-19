# VLA System — Vision–Language–Action Pipeline

A minimal but complete VLA pipeline for the Franka Panda robot.

```
[ Camera ] → [ YOLOv11 ] → [ Object Poses ]
                                   ↓
[ User Input ] → [ Language Node ] → [ Task Command ]
                                           ↓
                                  [ Task Coordinator ]
                                           ↓
                                    [ Action Node ]
                                           ↓
                                  [ MoveIt2 / Franka ]
```

---

## Package Structure

```
vla_system/
├── interfaces/          # Custom ROS2 msgs: ObjectDetection, TaskCommand, ActionResult
├── vision_node/         # YOLOv11 object detection + depth estimation
├── language_node/       # Natural language → structured command parser
├── coordinator_node/    # Task orchestration (the brain)
├── action_node/         # MoveIt2 motion execution
├── launch/              # Launch files
├── web_ui/              # Browser control panel
└── vla_cli.py           # Interactive terminal client
```

---

## Prerequisites

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# Python deps
pip install ultralytics opencv-python cv-bridge

# Download YOLO model (for real vision mode)
python3 scripts/download_yolo_model.py

# Optional: real MoveIt2
sudo apt install ros-humble-moveit2

# Optional: LLM parser
pip install anthropic
export ANTHROPIC_API_KEY=sk-ant-...
```

---

## Build

```bash
cd ~/ros2_ws/src
cp -r /path/to/vla_system .
cd ~/ros2_ws
colcon build --packages-select vla_interfaces
source install/setup.bash
colcon build
source install/setup.bash
```

---

## Run

### Full mock mode (no hardware needed — great for testing)
```bash
ros2 launch vla_system vla_full.launch.py
```

### In a second terminal — send commands
```bash
# Option A: CLI client
python3 vla_cli.py

# Option B: raw ROS2 topic
ros2 topic pub /raw_command std_msgs/String "data: 'Pick the screw'"

# Option C: Web UI
# Open web_ui/index.html in a browser
# Also start rosbridge:
sudo apt install ros-humble-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### With LLM parser (Phase 2 upgrade)
```bash
export ANTHROPIC_API_KEY=sk-ant-...
ros2 launch vla_system vla_full.launch.py parser_mode:=llm
```

### With real camera
```bash
# Start RealSense first
ros2 launch realsense2_camera rs_launch.py
# Then launch VLA with real vision
ros2 launch vla_system vla_full.launch.py use_mock_vision:=false
```

### With real robot + MoveIt2
```bash
ros2 launch vla_system vla_full.launch.py \
  use_mock_vision:=false \
  use_mock_action:=false
```

---

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/raw_command` | std_msgs/String | User → Language |
| `/task_command` | vla_interfaces/TaskCommand | Language → Coordinator |
| `/detected_objects` | vla_interfaces/ObjectDetection | Vision → Coordinator |
| `/execute_action` | vla_interfaces/TaskCommand | Coordinator → Action |
| `/action_result` | vla_interfaces/ActionResult | Action → Coordinator |
| `/system_status` | std_msgs/String | Coordinator → All |
| `/vision_debug/image` | sensor_msgs/Image | Vision → RViz |

---

## Example Commands

```
Pick the screw
Pick the screw and place it in bin A
Pick the smallest rivet and place it in bin B
Grab the bolt and put it in bin C
Inspect the washer
Pick the nearest screw
```

---

## Phase Roadmap

| Phase | Status | Description |
|---|---|---|
| 1 — Perception | ✅ | YOLOv11 + depth → /detected_objects |
| 2 — Language | ✅ | Rule-based parser (+ LLM upgrade ready) |
| 3 — Action | ✅ | MoveIt2 pick-and-place (+ mock mode) |
| 4 — Integration | ✅ | Full pipeline via coordinator |
| Bonus A | ✅ | Modifier-based selection (smallest, nearest…) |
| Bonus B | ✅ | Sequential support (via repeated commands) |
| Bonus C | ✅ | Error handling + status reporting |

---

## Architecture Notes

**Why a separate coordinator?**
The coordinator decouples the language and vision timing — it buffers detections
and retries up to 5× if the object hasn't been detected yet when the command arrives.
This is the "VLA" moment: vision and language meet here before action.

**Extending the parser**
Set `parser_mode:=llm` to upgrade to the Claude-based parser. It handles
complex/ambiguous inputs like "take the second screw from the left" that
rule-based parsing can't handle.

**Bin positions**
Edit `BIN_POSITIONS` in `action_node.py` to match your Gazebo or real setup.
Positions are in the robot `base_link` frame.
