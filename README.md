# Robotics VLA Torque Sentinel — C++ ISO 10218-1 Safety & Rust Trajectory Engine 🦾

> **C++ ISO 10218-1 torque safety controller and Rust minimum-jerk 5th-order polynomial trajectory planner.**

[![C++](https://img.shields.io/badge/C++-17-00599C)]()
[![Rust](https://img.shields.io/badge/Rust-Safety%20Critical-orange)]()
[![Python](https://img.shields.io/badge/Python-3.9+-blue)]()
[![Domain](https://img.shields.io/badge/Domain-Robotics%20VLA-red)]()

---

## 🎯 For Recruiters & Hiring Managers

This repository implements the **Robotics VLA Torque Sentinel** — providing ISO 10218-1 compliant torque limiting and smooth trajectory planning for 7-DOF collaborative robotic arms. It demonstrates:

- **C++ ISO 10218-1 torque limiting** with derivative-based collision detection and thermal derating
- **Rust minimum-jerk trajectory planner** calculating 5th-order quintic polynomials for smooth motion
- **Real-time joint protection** capping torque commands at 80% of rated limits to prevent actuator damage
- **Python simulation test harness** verifying safety stops and collision detection

**Why this matters**: Vision-Language-Action (VLA) robotics requires hard real-time safety controllers to ensure physical robot arms never exceed safety boundaries during AI-guided execution.

---

## 🔬 For Engineers & Technical Reviewers

### Core Components

| Component | Language | Purpose |
|---|---|---|
| `src/torque_controller.cpp` | C++ | ISO 10218-1 safety controller with collision detection |
| `src/trajectory_planner.rs` | Rust | 5th-order polynomial minimum-jerk trajectory generator |
| `tests/` | Python | Robotic arm motion simulation with safety verification |

---

## 🤖 ML/AI & Programmatic Mesh Integration

- **MCP Tool**: `robot_safety_status()` — joint state and safety margin queryable by VLA agents
- **Mastermind Sidecar**: Telemetry bridge to APEX Highway mesh
- **SHA-256 Integrity**: Tracked in `.integrity/file_hashes.json`

---

## ⚡ Quick Start

```bash
python3 tests/test_torque_sentinel.py
```
