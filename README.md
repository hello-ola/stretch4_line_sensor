# Stretch 4 Line Sensor ROS 2 Bridge

ROS 2 package that publishes Stretch base line sensor data from `stretch4_body` using standard messages.

## Prerequisites

1. **robot_server** running with `line_sensor_loop` enabled in server subsystems.
2. **robot_state_publisher** with `stretch4_urdf` (for `base_link` and `line_sensor_*_optical_link` TF).
3. `hello-robot-stretch4-body` installed (same environment as this package).

### Enable line_sensor_loop on robot_server

Add to `stretch_user_yaml`:

```yaml
robot:
  server:
    subsystems:
      - line_sensor_loop
```

Without this, `RobotClient` does not expose `line_sensor_loop` and the node exits with an error.

## Build

From your ROS 2 workspace:

```bash
colcon build --packages-select stretch4_line_sensor
source install/setup.bash
```

## Run

```bash
# Terminal 1: robot_server (with line_sensor_loop enabled)
# Terminal 2: stretch_driver
ros2 launch stretch4_line_sensor line_sensor.launch.py
```

With RViz:

```bash
ros2 launch stretch4_line_sensor line_sensor.launch.py use_rviz:=true
```

## Topics

| Topic | Type | Frame | Description |
|-------|------|-------|-------------|
| `/line_sensor/points` | `sensor_msgs/PointCloud2` | `base_link` | Fused, tare-corrected ground points |
| `/line_sensor/obstacle_points` | `sensor_msgs/PointCloud2` | `base_link` | Cliff and bump points (outside ground band) |
| `/line_sensor/sensor_N/scan` | `sensor_msgs/LaserScan` | `line_sensor_N_optical_link` | Raw per-sensor ranges (no tare); N = 0..5 |

Set `publish_raw_scans:=false` to disable the scan topics.

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/line_sensor/reload_calibration` | `std_srvs/Trigger` | Reload fleet tare YAML from disk |

## Calibration

Record and compute flat-floor calibration with the factory tool:

```bash
REx_line_sensor_calibrate.py --all
```

Then reload without restarting the node:

```bash
ros2 service call /line_sensor/reload_calibration std_srvs/srv/Trigger
```

## Diagnostics

Published on `/diagnostics` via `diagnostic_updater`:

- Overall frame rate and latency
- Frame sync / incomplete frame counters
- Per-sensor staleness
- Calibration tare load status


