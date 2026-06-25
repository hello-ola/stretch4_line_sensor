"""Unit tests for obstacle spatial/temporal filtering."""

import numpy as np

from stretch4_line_sensor.obstacle_filter import ObstacleFilter


def test_temporal_filter_requires_consecutive_frames():
    filt = ObstacleFilter(
        min_consecutive_frames=3,
        track_match_thresh_m=0.05,
        track_max_age_s=10.0,
    )
    cluster = np.array([[0.2, 0.0, 0.05], [0.21, 0.0, 0.05]])

    assert filt._apply_temporal_filter([cluster]).shape[0] == 0
    assert filt._apply_temporal_filter([cluster]).shape[0] == 0
    out = filt._apply_temporal_filter([cluster])
    assert out.shape[0] == 2


def test_temporal_filter_resets_after_miss():
    filt = ObstacleFilter(
        min_consecutive_frames=2,
        track_match_thresh_m=0.05,
        track_max_age_s=10.0,
    )
    cluster = np.array([[0.2, 0.0, 0.05], [0.21, 0.0, 0.05]])

    filt._apply_temporal_filter([cluster])
    out = filt._apply_temporal_filter([cluster])
    assert out.shape[0] == 2

    assert filt._apply_temporal_filter([]).shape[0] == 0
    assert filt._apply_temporal_filter([cluster]).shape[0] == 0


def test_temporal_filter_matches_moving_cluster():
    filt = ObstacleFilter(
        min_consecutive_frames=2,
        track_match_thresh_m=0.10,
        track_max_age_s=10.0,
    )
    c1 = np.array([[0.20, 0.0, 0.05]])
    c2 = np.array([[0.24, 0.0, 0.05]])

    filt._apply_temporal_filter([c1])
    out = filt._apply_temporal_filter([c2])
    assert out.shape[0] == 1


def test_odom_compensation_keeps_static_track_while_base_frame_moves():
    filt = ObstacleFilter(
        min_consecutive_frames=2,
        track_match_thresh_m=0.05,
        track_max_age_s=10.0,
    )
    cluster1 = np.array([[0.50, 0.0, 0.05]])
    cluster2 = np.array([[0.35, 0.0, 0.05]])

    t1 = np.eye(4)
    t2 = np.eye(4)
    t2[0, 3] = 0.15

    filt._apply_temporal_filter([cluster1], base_to_tracking=t1)
    out = filt._apply_temporal_filter([cluster2], base_to_tracking=t2)
    assert out.shape[0] == 1


def test_without_odom_compensation_loses_track_on_large_motion():
    filt = ObstacleFilter(
        min_consecutive_frames=2,
        track_match_thresh_m= 0.05,
        track_max_age_s=10.0,
    )
    cluster1 = np.array([[0.50, 0.0, 0.05]])
    cluster2 = np.array([[0.35, 0.0, 0.05]])

    filt._apply_temporal_filter([cluster1])
    out = filt._apply_temporal_filter([cluster2])
    assert out.shape[0] == 0
