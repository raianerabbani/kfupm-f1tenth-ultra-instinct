# CIE-KFUPM F1TENTH Controller Submission

## Team Controller Information
- **Package Name:** `cie_controller`
- **Node Name:** `controller`
- **Execution Command:** `ros2 run cie_controller controller`
- **Target Stage:** Stage 1 (Clean-lap qualification on Levine track)

## Overview & Algorithm
This package implements a robust, high-performance controller designed for Stage 1 clean-lap qualification on the Levine course.
- **Trajectory Tracking:** Pure Pursuit path tracker operating on the Levine centerline waypoints with adaptive lookahead calculation.
- **Dynamic Speed Profiling:** Automatically modulates velocity between straight sections (up to 2.2 m/s) and cornering sections (down to 1.3 m/s) to guarantee zero wheel slip and stable adherence to the raceline.
- **LiDAR Safety Layer:** Continuously samples `/scan` across the forward sector. In the event of an obstacle or impending collision (< 0.35m), emergency braking is immediately engaged.
- **Max Permitted Speed:** 2.2 m/s (under conservative speed cap).
- **Target Qualification Requirement:** 3 completed timed laps with 0 recorded collisions.

## Dependencies
- ROS 2 Jazzy Jalisco
- `rclpy`
- `sensor_msgs`
- `nav_msgs`
- `ackermann_msgs`

## Build and Run

### 1. Build
```bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/scratch/CIE-KFUPM-F1tenth-Autonomous-Racing-Competition/starter_ws/.venv/bin/activate
source ~/Documents/scratch/CIE-KFUPM-F1tenth-Autonomous-Racing-Competition/starter_ws/install/setup.bash

cd ~/controller_ws
python /usr/bin/colcon build --symlink-install
source install/setup.bash
```

### 2. Run
In Terminal 1 (Start Simulator & Race Monitor):
```bash
cd ~/Documents/scratch/CIE-KFUPM-F1tenth-Autonomous-Racing-Competition
./scripts/launch_sim.sh
```

In Terminal 2 (Run Controller):
```bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/scratch/CIE-KFUPM-F1tenth-Autonomous-Racing-Competition/starter_ws/.venv/bin/activate
source ~/Documents/scratch/CIE-KFUPM-F1tenth-Autonomous-Racing-Competition/starter_ws/install/setup.bash
source ~/controller_ws/install/setup.bash

ros2 run cie_controller controller
```

## Configurable Parameters
| Parameter | Default | Description |
|---|---|---|
| `lookahead_distance` | `1.25` | Lookahead distance in metres for Pure Pursuit |
| `target_speed` | `2.2` | Maximum speed on straight segments (m/s) |
| `min_speed` | `1.3` | Speed during tight cornering (m/s) |
| `wheelbase` | `0.33` | Effective vehicle wheelbase (m) |
| `steering_gain` | `0.85` | Curvature-to-steering scaling factor |
| `max_steering_angle` | `0.4189` | Maximum steering angle in radians (~24°) |
