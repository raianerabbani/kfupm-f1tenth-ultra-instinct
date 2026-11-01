# KFUPM F1TENTH - Team Ultra Instinct

Official submission package for the **CIE-KFUPM F1TENTH Autonomous Racing Competition**.

## Controller Submission Information
- **Team Name:** Ultra Instinct
- **Package Name:** `kfupm_f1tenth_ultra_instinct`
- **Node Name:** `controller`
- **Execution Command:** `ros2 run kfupm_f1tenth_ultra_instinct controller`
- **Target Stage:** Stage 1 (Clean-lap qualification on Levine track)
- **Max Requested Speed:** 2.2 m/s
- **Target Qualification Requirement:** 3 completed timed laps with 0 recorded collisions

---

## Overview & Algorithm
This package implements an autonomous racing controller engineered specifically for **Stage 1 clean-lap qualification** on the Levine course.
- **Pure Pursuit Tracking:** Global closest-waypoint search on the Levine centerline with responsive lookahead ($1.15\,\text{m}$).
- **Curvature Anticipation & Aggressive Braking:** Uses a forward lookahead preview point ($\sim 2.2\,\text{m}$ ahead). When approaching a corner, the controller begins braking *early* on the straightaway and uses an aggressive non-linear square-root braking curve to slow down to $0.9\,\text{m/s}$ at the apex, completely preventing lateral drift or wall clipping.
- **Active LiDAR Safety Layer:** Continuously samples `/scan` across the forward sector. Features multi-stage emergency braking: aggressive slowdown if a wall is within $1.1\,\text{m}$ head-on, and an immediate full stop ($0.0\,\text{m/s}$) if within $0.45\,\text{m}$.
- **Zero-Dependency Portability:** Embedded in-memory waypoint coordinates guarantee immediate execution on any machine or evaluation workspace without path resolution issues.

---

## Qualification Test Results
Validated against the official competition `race_monitor`:
```text
============================================================
STAGE 1 QUALIFICATION SUCCESSFUL!
Total Laps Completed: 3
Total Recorded Collisions: 0
  Lap 1: 35.310 s
  Lap 2: 35.270 s
  Lap 3: 35.270 s
Best Lap Time: 35.270 s
============================================================
```

---

## Dependencies
- ROS 2 Jazzy Jalisco
- `rclpy`
- `sensor_msgs`
- `nav_msgs`
- `ackermann_msgs`

---

## Build and Run Instructions (For Organizers)

### 1. Build
```bash
source /opt/ros/jazzy/setup.bash
source <competition-repo>/starter_ws/.venv/bin/activate
source <competition-repo>/starter_ws/install/setup.bash

cd ~/controller_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Run
**Terminal 1 (Simulator & Race Monitor):**
```bash
cd <competition-repo>
./scripts/launch_sim.sh
```

**Terminal 2 (Controller):**
```bash
source /opt/ros/jazzy/setup.bash
source <competition-repo>/starter_ws/.venv/bin/activate
source <competition-repo>/starter_ws/install/setup.bash
source ~/controller_ws/install/setup.bash

ros2 run kfupm_f1tenth_ultra_instinct controller
```

---

## Parameters Summary
| Parameter | Value | Description |
|---|---|---|
| `lookahead` | `1.15 m` | Steering lookahead distance |
| `speed_max` | `2.2 m/s` | Maximum cruising speed on straights |
| `speed_turn` | `0.9 m/s` | Controlled apex speed for corners |
| `max_steer` | `0.38 rad` | Maximum steering angle limit (~22°) |
| `wheelbase` | `0.33 m` | Vehicle wheelbase |
| `preview_distance` | `~2.2 m` | Lookahead anticipation for early braking |
