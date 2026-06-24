"""Build per-sensor sensor_msgs/LaserScan from raw line sensor range arrays."""

from __future__ import annotations

import math

import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


def sensor_name_to_optical_frame(sensor_name: str) -> str:
    """Map stretch4_body sensor_N to stretch4_urdf line_sensor_N_optical_link."""
    if not sensor_name.startswith('sensor_'):
        raise ValueError(f'Unexpected sensor name: {sensor_name}')
    index = sensor_name.split('_', 1)[1]
    return f'line_sensor_{index}_optical_link'


def build_raw_laserscan(
    ranges: np.ndarray,
    stamp: Time,
    frame_id: str,
    horizontal_fov_deg: float,
    range_min: float = 0.05,
    range_max: float = 4.0,
) -> LaserScan:
    """
    Convert a Pixart J3 range array to LaserScan in the sensor optical frame.

    Ranges are published as raw slant distances (meters). Invalid readings are inf.
    Angles span the sensor horizontal FOV in the optical frame x-y scan plane.
    """
    scan = LaserScan()
    scan.header = Header(stamp=stamp, frame_id=frame_id)

    n = len(ranges)
    if n == 0:
        scan.ranges = []
        return scan

    fov_rad = math.radians(horizontal_fov_deg)
    scan.angle_min = -fov_rad / 2.0
    scan.angle_max = fov_rad / 2.0
    scan.angle_increment = fov_rad / (n - 1) if n > 1 else 0.0
    scan.time_increment = 0.0
    scan.scan_time = 0.0
    scan.range_min = range_min
    scan.range_max = range_max

    ranges_f = np.asarray(ranges, dtype=np.float64)
    out = np.empty(n, dtype=np.float32)
    invalid = (ranges_f <= 0.0) | (ranges_f >= range_max) | ~np.isfinite(ranges_f)
    out[:] = ranges_f
    out[invalid] = float('inf')
    scan.ranges = out.tolist()

    return scan
