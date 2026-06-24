"""Unit tests for raw LaserScan conversion."""

import math

import numpy as np
import pytest
from builtin_interfaces.msg import Time

from stretch4_line_sensor.raw_scan import (
    build_raw_laserscan,
    sensor_name_to_optical_frame,
)


def test_sensor_name_to_optical_frame():
    assert sensor_name_to_optical_frame('sensor_0') == 'line_sensor_0_optical_link'
    assert sensor_name_to_optical_frame('sensor_5') == 'line_sensor_5_optical_link'


def test_build_raw_laserscan_valid_ranges():
    ranges = np.linspace(0.3, 0.5, 320)
    stamp = Time(sec=1, nanosec=0)
    scan = build_raw_laserscan(
        ranges=ranges,
        stamp=stamp,
        frame_id='line_sensor_0_optical_link',
        horizontal_fov_deg=103.0,
        range_min=0.05,
        range_max=4.0,
    )
    assert scan.header.frame_id == 'line_sensor_0_optical_link'
    assert len(scan.ranges) == 320
    assert scan.ranges[0] == pytest.approx(0.3)
    assert math.isfinite(scan.ranges[-1])
    fov_rad = math.radians(103.0)
    assert scan.angle_min == pytest.approx(-fov_rad / 2.0)
    assert scan.angle_max == pytest.approx(fov_rad / 2.0)


def test_build_raw_laserscan_invalid_ranges_are_inf():
    ranges = np.array([0.0, -1.0, 5.0, 0.25])
    scan = build_raw_laserscan(
        ranges=ranges,
        stamp=Time(sec=0, nanosec=0),
        frame_id='line_sensor_1_optical_link',
        horizontal_fov_deg=103.0,
    )
    assert math.isinf(scan.ranges[0])
    assert math.isinf(scan.ranges[1])
    assert math.isinf(scan.ranges[2])
    assert scan.ranges[3] == pytest.approx(0.25)
