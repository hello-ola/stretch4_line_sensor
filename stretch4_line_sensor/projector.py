"""Project line sensor ranges to PointCloud2 messages in base_link."""

from __future__ import annotations

import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from stretch4_body.subsystem.line_sensor.line_sensor_utils import LineSensorGeometry
from stretch4_line_sensor.raw_scan import as_range_array


def _numpy_to_pointcloud2(
    points: np.ndarray,
    header: Header,
) -> PointCloud2:
    """Convert Nx3 float array to PointCloud2 with x, y, z, and z_deviation fields."""
    if len(points) == 0:
        return point_cloud2.create_cloud_xyz32(header, [])

    structured = np.zeros(
        len(points),
        dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('z_deviation', np.float32),
        ],
    )
    structured['x'] = points[:, 0]
    structured['y'] = points[:, 1]
    structured['z'] = points[:, 2]
    structured['z_deviation'] = points[:, 2]

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='z_deviation', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    return point_cloud2.create_cloud(header, fields, structured)


def filter_obstacle_points(
    points: np.ndarray,
    thresh_cliff_mm: float,
    thresh_obstacle_mm: float,
) -> np.ndarray:
    """Keep points outside the ground band (cliffs below, obstacles above)."""
    if len(points) == 0:
        return points

    z_min_exclude = -thresh_cliff_mm / 1000.0
    z_max_exclude = thresh_obstacle_mm / 1000.0
    mask_ground = (points[:, 2] > z_min_exclude) & (points[:, 2] < z_max_exclude)
    return points[~mask_ground]


def split_cliff_obstacle_points(
    points: np.ndarray,
    thresh_cliff_mm: float,
    thresh_obstacle_mm: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Split Z-band hazard points into cliffs below and obstacles above floor."""
    if len(points) == 0:
        empty = np.zeros((0, 3))
        return empty, empty

    z_min_exclude = -thresh_cliff_mm / 1000.0
    z_max_exclude = thresh_obstacle_mm / 1000.0
    cliff_pts = points[points[:, 2] <= z_min_exclude]
    obstacle_pts = points[points[:, 2] >= z_max_exclude]
    return cliff_pts, obstacle_pts


class LineSensorProjector:
    """Fuse per-sensor ranges into base_link point clouds."""

    def __init__(
        self,
        geometry_params: dict,
        sensor_names: list[str],
        thresh_cliff_mm: float,
        thresh_obstacle_mm: float,
        max_range: float = 4.0,
    ):
        self.geometry = LineSensorGeometry(geometry_params)
        self.sensor_names = sensor_names
        self.thresh_cliff_mm = thresh_cliff_mm
        self.thresh_obstacle_mm = thresh_obstacle_mm
        self.max_range = max_range

    def project_status(
        self,
        status: dict,
        apply_tare,
        stamp: Time,
        frame_id: str,
    ) -> tuple[PointCloud2, PointCloud2]:
        """Build fused and obstacle-filtered clouds from line_sensor_loop status."""
        fused, obstacle_pts = self.project_arrays(status, apply_tare)
        header = Header(stamp=stamp, frame_id=frame_id)
        return (
            _numpy_to_pointcloud2(fused, header),
            _numpy_to_pointcloud2(obstacle_pts, header),
        )

    def project_arrays(
        self,
        status: dict,
        apply_tare,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Project status to fused and Z-band obstacle numpy arrays."""
        fused, _, _ = self.project_arrays_split(status, apply_tare)
        hazard_pts = filter_obstacle_points(
            fused,
            self.thresh_cliff_mm,
            self.thresh_obstacle_mm,
        )
        return fused, hazard_pts

    def project_arrays_split(
        self,
        status: dict,
        apply_tare,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Project status to fused, cliff, and above-floor obstacle arrays."""
        all_points = []
        for sensor_idx, sensor_name in enumerate(self.sensor_names):
            sensor_status = status.get(sensor_name, {})
            if not isinstance(sensor_status, dict):
                continue
            ranges_arr = as_range_array(sensor_status.get('ranges'))
            if ranges_arr.size == 0:
                continue

            if apply_tare is not None:
                ranges_arr = apply_tare(ranges_arr, sensor_name)

            pts = self.geometry.get_sensor_points_in_robot_frame(sensor_idx, ranges_arr)
            if len(pts) == 0:
                continue

            if self.max_range < float('inf'):
                pts = pts[pts[:, 2] < self.max_range]
            if len(pts) > 0:
                all_points.append(pts)

        if not all_points:
            empty = np.zeros((0, 3))
            return empty, empty, empty

        fused = np.vstack(all_points)
        cliff_pts, obstacle_pts = split_cliff_obstacle_points(
            fused,
            self.thresh_cliff_mm,
            self.thresh_obstacle_mm,
        )
        return fused, cliff_pts, obstacle_pts
