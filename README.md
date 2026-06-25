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
| `/line_sensor/cliff_points` | `sensor_msgs/PointCloud2` | `base_link` | Drop-off/cliff points below the floor band after spatial/temporal filtering |
| `/line_sensor/obstacle_points` | `sensor_msgs/PointCloud2` | `base_link` | Above-floor bump/obstacle points after spatial/temporal filtering |
| `/line_sensor/cliff_points_unfiltered` | `sensor_msgs/PointCloud2` | `base_link` | Cliff points after Z-band only (when `publish_unfiltered_obstacles:=true`) |
| `/line_sensor/obstacle_points_unfiltered` | `sensor_msgs/PointCloud2` | `base_link` | Above-floor obstacle points after Z-band only (when `publish_unfiltered_obstacles:=true`) |
| `/line_sensor/sensor_N/scan` | `sensor_msgs/LaserScan` | `line_sensor_N_optical_link` | Raw per-sensor ranges (no tare); N = 0..5 |

Set `publish_raw_scans:=false` to disable the scan topics.

## Obstacle filtering

When `obstacle_filter_enabled:=true` (default), `/line_sensor/cliff_points`
and `/line_sensor/obstacle_points` apply:

1. **Z-band split** — remove floor returns, then split cliffs below the
   cliff threshold from obstacles above the obstacle threshold
2. **Spatial clustering (DBSCAN)** — drop isolated noise points and tiny clusters
3. **Temporal persistence** — require `min_consecutive_frames` consecutive detections before publishing

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `obstacle_filter_enabled` | `true` | Enable spatial + temporal filtering |
| `min_consecutive_frames` | `3` | Frames a cluster must persist (~100 ms @ 30 Hz) |
| `cluster_eps` | `0.03` | DBSCAN neighborhood radius (m) |
| `cluster_min_points` | `10` | Minimum points per cluster |
| `min_cluster_width_m` | `0.01` | Minimum cluster extent (m) |
| `track_match_thresh_m` | `0.10` | Max centroid motion between frames (m) |
| `track_max_age_s` | `1.0` | Drop tracks not seen within this time |
| `publish_unfiltered_obstacles` | `false` | Also publish Z-band-only cloud for debugging |

Set `obstacle_filter_enabled:=false` to publish Z-band obstacles only (previous behavior).

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


