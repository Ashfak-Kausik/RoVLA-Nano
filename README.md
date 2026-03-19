# RoVLA-Nano

**ROS-based Vision–Language–Action System for Industrial Manipulation**

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green)](https://python.org)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)
[![IRL-VLA](https://img.shields.io/badge/IRL--VLA-Level%201-orange)]()
[![Tests](https://img.shields.io/badge/Tests-34%20passing-brightgreen)]()

> *A minimal, modular VLA pipeline targeting the industrial deployment gap — perception → reasoning → action on real robotic hardware, zero training required.*

---

## Why RoVLA-Nano?

Existing VLA systems (RT-2, OpenVLA, TinyVLA) are evaluated on server-class GPUs and tabletop benchmarks. Our on-going systematic review paper based on 100+ VLA models (and published research papers) confirmed a precise, unaddressed gap:

- **No published VLA system has been characterised for worst-case execution time (WCET) on industrial edge hardware** (even if deployed somewhere, not reported yet or disclosed)
- WCET characterisation is absent from the entire reviewed corpus.
- The field sits at IRL 1 at best — collaborative industrial deployment requires IRL 3 minimum (IRL means Industry Readiness Level, a term defined by our systematic review work which is yet to be published).
- No VLA system has been evaluated against ISO 10218 or ISO/TS 15066 functional safety standards (report can be found in our systematic review paper).

RoVLA-Nano is the foundation for closing that gap — a clean, ROS2-native architecture that can be incrementally hardened toward real industrial deployment criteria.

---

## Architecture
```
User Input  →  Language Node  →  Task Coordinator  →  Action Node
                                        ↑
                               Vision Node (YOLOv11n)
                               RGB-D Camera
```

Four independent ROS2 nodes — each replaceable, each testable in isolation:

| Node | Function | Technology |
|------|----------|------------|
| Vision | Object detection + 3D pose estimation | YOLOv11n (2.6M params) |
| Language | NL → structured command | Rule-based (15µs) or LLM API |
| Coordinator | Task orchestration + object selection | Python + ROS2 |
| Action | Motion execution | MoveIt2 + Franka Panda |

---

## Key Specifications

| Metric | Value |
|--------|-------|
| Parameters (core) | ~2.6M (vision backbone only) |
| Perception latency | 4.3 ms (YOLO + depth + ROS2, GPU) |
| Language latency | 15 µs (rule-based mode) |
| Language throughput | 66,000 parses/sec |
| End-to-end cycle | ~7 s (dominated by robot motion) |
| Training required | None — zero-shot deployment |
| Test suite | 34 tests passing (no ROS2 install needed) |
| Mock mode | Full pipeline runs on any laptop |

---

## IRL-VLA Status

This project uses the **Industrial Readiness Level for VLAs (IRL-VLA)** framework introduced in our companion review paper — a five-level taxonomy for assessing VLA deployment maturity against industrial constraints.

| Level | Label | Status |
|-------|-------|--------|
| IRL 0 | Lab Proof of Concept | ✅ Complete |
| IRL 1 | Edge-Aware | ✅ Complete (mock mode validated) |
| IRL 2 | Real-Time Candidate | 🔄 In Progress (Jetson WCET characterisation) |
| IRL 3 | Safety-Aware | ⬜ Planned (ISO/TS 15066 safety layer) |
| IRL 4 | Industrially Certifiable | ⬜ Future |

**IRL 2 is the primary research target** — providing the first WCET characterisation of a VLA system on industrial edge hardware, a gap confirmed absent across 83 reviewed papers.

---

## Quick Start
```bash
# Clone
git clone https://github.com/Ashfak-Kausik/RoVLA-Nano.git
cd RoVLA-Nano/vla_system

# Zero hardware — runs on any laptop
ros2 launch vla_system vla_full.launch.py

# Send a command
ros2 topic pub /raw_command std_msgs/String "data: 'Pick the screw'"

# With LLM language mode
export ANTHROPIC_API_KEY=sk-ant-...
ros2 launch vla_system vla_full.launch.py parser_mode:=llm

# Run all tests (no ROS2 install needed)
python3 tests/run_all_tests.py
```

---

## Project Structure
```
vla_system/
├── vision_node/         # YOLOv11n + RGB-D depth estimation
├── language_node/       # Rule-based parser + LLM upgrade path
├── coordinator_node/    # Task orchestration + modifier-based selection
├── action_node/         # MoveIt2 pick-and-place executor
├── interfaces/          # Custom ROS2 message types
├── launch/              # Launch files (full system + Gazebo)
├── config/              # All parameters in vla_params.yaml
├── gazebo/              # Simulation world (workbench + bins + fasteners)
├── tests/               # 34 tests, runs without ROS2
├── scripts/             # CLI tools, topic monitor, command injector
└── web_ui/              # Browser control panel (rosbridge compatible)
```

---

## What Makes This Different

| Feature | RoVLA-Nano | OpenVLA | TinyVLA | RT-2 |
|---------|-----------|---------|---------|------|
| Parameters | 2.6M | 7B | <1B | 55B |
| Training required | None | Yes | Yes | Yes |
| ROS2 native | ✅ | ❌ | ❌ | ❌ |
| Laptop-ready | ✅ | ❌ | ❌ | ❌ |
| WCET characterised | 🔄 | ❌ | ❌ | ❌ |
| ISO safety layer | 🔄 | ❌ | ❌ | ❌ |

---

## Roadmap

- [ ] **IRL 2**: WCET characterisation on Jetson Orin NX — first in VLA literature
- [ ] Custom YOLOv11 fine-tuning on industrial fastener dataset
- [ ] Safety layer architecture compatible with ISO/TS 15066
- [ ] Sequential multi-step command planning
- [ ] Force-torque grasp feedback integration
- [ ] Standardised industrial manipulation benchmark

---

## Companion Review Paper

This project is developed alongside a systematic review:

> **"Lightweight Vision-Language-Action Models for Real-Time Collaborative Industrial Manipulation: System Constraints, Deployment Strategies, and Research Directions"**
> Ashfakul Karim Kausik, Tahsin Ahmed Refat, Saif Alvi.
> Military Institute of Science and Technology / Khulna University of Engineering and Technology.
> *Preprint coming soon.*

The review introduces the IRL-VLA framework and identifies WCET characterisation as the primary unaddressed gap across 100+ existing VLA models and their reports.

---

## Requirements
```
ROS2 Humble — Ubuntu 22.04
Python 3.10+
pip install ultralytics opencv-python cv-bridge

Optional:
  sudo apt install ros-humble-moveit    # real robot execution
  pip install anthropic                  # LLM language mode
```

---

## Author

**Ashfakul Karim Kausik**
Department of Industrial Engineering
Military Institute of Science and Technology (MIST), Dhaka, Bangladesh

Interested in collaboration on industrial VLA deployment, edge inference characterisation, or safety-verifiable robot policies — open an issue or reach out directly.

---

## License

MIT — see [LICENSE](LICENSE) for details.
