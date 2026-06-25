"""Unit tests for line sensor projection (no hardware)."""

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from stretch4_line_sensor.projector import (
    LineSensorProjector,
    filter_obstacle_points,
    _numpy_to_pointcloud2,
)


@pytest.fixture
def geometry_params():
    return {
        'pixart_report_num': 320,
        'sensor_horizontal_fov_degrees': 103.0,
        'emitter_height_above_floor_mm': 100.67,
        'emitter_pitch_diameter_mm': 404.04,
        'sensor_angle_down_deg': 26.0,
        'sensor_angles_deg': [10.18, 39.64, 80.36, 39.64, 80.36, 39.64],
        'sensor_normals_deg': [0.0, 60.0, 120.0, 180.0, 240.0, 300.0],
    }


@pytest.fixture
def sensor_names():
    return [f'sensor_{i}' for i in range(6)]


def test_filter_obstacle_points_removes_ground_band():
    # Ground band with 10 mm thresholds: z in (-0.01, 0.01) is filtered out.
    points = np.array([
        [0.0, 0.0, 0.0],       # on floor, removed
        [0.1, 0.0, 0.005],     # 5 mm, inside band, removed
        [0.2, 0.0, -0.02],     # cliff below -10 mm, kept
        [0.3, 0.0, 0.05],      # obstacle above +10 mm, kept
    ])
    filtered = filter_obstacle_points(points, thresh_cliff_mm=10, thresh_obstacle_mm=10)
    assert len(filtered) == 2
    assert filtered[0, 2] == -0.02
    assert filtered[1, 2] == 0.05


def test_numpy_to_pointcloud2_empty():
    header = Header(stamp=Time(sec=0, nanosec=0), frame_id='base_link')
    cloud = _numpy_to_pointcloud2(np.zeros((0, 3)), header)
    assert cloud.width == 0
    assert cloud.header.frame_id == 'base_link'


def test_project_status_numpy_ranges(geometry_params, sensor_names):
    projector = LineSensorProjector(
        geometry_params=geometry_params,
        sensor_names=sensor_names,
        thresh_cliff_mm=10,
        thresh_obstacle_mm=10,
    )
    ranges = np.linspace(0.3, 0.5, 320)
    status = {'sensor_0': {'ranges': ranges}}
    fused, obstacle_pts = projector.project_arrays(status=status, apply_tare=None)
    assert fused.ndim == 2
    assert fused.shape[1] == 3
    assert obstacle_pts.ndim == 2


def test_project_status_empty(geometry_params, sensor_names, capsys):
    projector = LineSensorProjector(
        geometry_params=geometry_params,
        sensor_names=sensor_names,
        thresh_cliff_mm=10,
        thresh_obstacle_mm=10,
    )
    status = {'sensor_0': {'ranges': []}}
    header_stamp = Time(sec=1, nanosec=0)
    points, obstacles = projector.project_status(
        status=status,
        apply_tare=None,
        stamp=header_stamp,
        frame_id='base_link',
    )
    assert points.width == 0
    assert obstacles.width == 0
