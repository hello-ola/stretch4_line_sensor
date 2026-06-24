#!/usr/bin/env python3
"""ROS 2 node bridging Stretch line sensors from stretch4_body RobotClient."""

from __future__ import annotations

import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from stretch4_body.core.robot_params import RobotParams
from stretch4_body.robot.robot_client import RobotClient
from stretch4_body.subsystem.line_sensor.line_sensor_utils import LineSensorCalibration

from stretch4_line_sensor.diagnostics import LineSensorDiagnostics
from stretch4_line_sensor.obstacle_filter import ObstacleFilter
from stretch4_line_sensor.projector import LineSensorProjector, _numpy_to_pointcloud2
from stretch4_line_sensor.raw_scan import build_raw_laserscan, sensor_name_to_optical_frame


class _CalibrationParamsHolder:
    """Minimal holder so LineSensorCalibration can load fleet tare files."""

    def __init__(self, params: dict):
        self.params = params


class LineSensorNode(Node):
    def __init__(self):
        super().__init__('line_sensor_node')

        _, robot_params = RobotParams.get_params()
        ls_params = robot_params.get('line_sensor_loop', {})
        cluster_params = ls_params.get('line_sensor_cluster_tracker', {})
        geometry_params = ls_params.get('line_sensor_geometry', {})
        sensor_names = ls_params.get('sensor_names', [])

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('apply_tare', True)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter(
            'thresh_cliff_mm',
            float(cluster_params.get('thresh_cliff_mm', 10)),
        )
        self.declare_parameter(
            'thresh_obstacle_mm',
            float(cluster_params.get('thresh_obstacle_mm', 10)),
        )
        self.declare_parameter('robot_client_ip', '127.0.0.1')
        self.declare_parameter('publish_raw_scans', True)
        self.declare_parameter('scan_range_min', 0.05)
        self.declare_parameter('scan_range_max', 4.0)
        self.declare_parameter('obstacle_filter_enabled', True)
        self.declare_parameter('publish_unfiltered_obstacles', False)
        self.declare_parameter(
            'min_consecutive_frames',
            int(cluster_params.get('min_consecutive_frames', 3)),
        )
        self.declare_parameter(
            'cluster_eps',
            float(cluster_params.get('cluster_eps', 0.03)),
        )
        self.declare_parameter(
            'cluster_min_points',
            int(cluster_params.get('cluster_min_points', 10)),
        )
        self.declare_parameter(
            'min_cluster_width_m',
            float(cluster_params.get('min_width', 0.01)),
        )
        self.declare_parameter(
            'track_match_thresh_m',
            float(cluster_params.get('match_thresh_m', 0.10)),
        )
        self.declare_parameter(
            'track_max_age_s',
            float(cluster_params.get('max_age_s', 1.0)),
        )

        self._base_frame = self.get_parameter('base_frame').value
        self._apply_tare = self.get_parameter('apply_tare').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        max_range = float(self.get_parameter('max_range').value)
        thresh_cliff_mm = float(self.get_parameter('thresh_cliff_mm').value)
        thresh_obstacle_mm = float(self.get_parameter('thresh_obstacle_mm').value)
        robot_client_ip = self.get_parameter('robot_client_ip').value
        self._publish_raw_scans = self.get_parameter('publish_raw_scans').value
        scan_range_min = float(self.get_parameter('scan_range_min').value)
        scan_range_max = float(self.get_parameter('scan_range_max').value)
        self._horizontal_fov_deg = float(
            geometry_params.get('sensor_horizontal_fov_degrees', 103.0)
        )
        self._obstacle_filter_enabled = self.get_parameter('obstacle_filter_enabled').value
        self._publish_unfiltered_obstacles = (
            self.get_parameter('publish_unfiltered_obstacles').value
        )

        self._robot_client = RobotClient(ip_address=robot_client_ip)
        if not self._robot_client.startup():
            self.get_logger().error('Failed to start RobotClient. Is robot_server running?')
            raise RuntimeError('RobotClient startup failed')

        if not hasattr(self._robot_client, 'line_sensor_loop'):
            self.get_logger().error(
                'line_sensor_loop is not enabled on robot_server. '
                'Add line_sensor_loop to robot.server.subsystems in stretch_user_yaml.'
            )
            self._robot_client.stop()
            raise RuntimeError('line_sensor_loop not available on robot_server')

        self._calibration = LineSensorCalibration(_CalibrationParamsHolder(ls_params))
        self._reload_calibration()

        self._projector = LineSensorProjector(
            geometry_params=geometry_params,
            sensor_names=sensor_names,
            thresh_cliff_mm=thresh_cliff_mm,
            thresh_obstacle_mm=thresh_obstacle_mm,
            max_range=max_range,
        )

        self._sensor_names = sensor_names
        self._scan_range_min = scan_range_min
        self._scan_range_max = scan_range_max

        self._diagnostics = LineSensorDiagnostics(self, sensor_names)

        self._points_pub = self.create_publisher(PointCloud2, '/line_sensor/points', 10)
        self._obstacle_pub = self.create_publisher(
            PointCloud2, '/line_sensor/obstacle_points', 10,
        )
        self._obstacle_unfiltered_pub = None
        if self._publish_unfiltered_obstacles:
            self._obstacle_unfiltered_pub = self.create_publisher(
                PointCloud2, '/line_sensor/obstacle_points_unfiltered', 10,
            )
        self._obstacle_filter = None
        if self._obstacle_filter_enabled:
            self._obstacle_filter = ObstacleFilter(
                cluster_eps=float(self.get_parameter('cluster_eps').value),
                cluster_min_points=int(self.get_parameter('cluster_min_points').value),
                min_cluster_width_m=float(self.get_parameter('min_cluster_width_m').value),
                track_match_thresh_m=float(self.get_parameter('track_match_thresh_m').value),
                track_max_age_s=float(self.get_parameter('track_max_age_s').value),
                min_consecutive_frames=int(self.get_parameter('min_consecutive_frames').value),
            )
        self._scan_pubs: dict[str, object] = {}
        if self._publish_raw_scans:
            for name in sensor_names:
                topic = f'/line_sensor/{name}/scan'
                self._scan_pubs[name] = self.create_publisher(LaserScan, topic, 10)
        self._reload_srv = self.create_service(
            Trigger,
            '/line_sensor/reload_calibration',
            self._reload_calibration_callback,
        )

        timer_period = 1.0 / max(publish_rate, 1.0)
        self._timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(
            f'line_sensor_node started ({len(sensor_names)} sensors, '
            f'{publish_rate:.1f} Hz, raw_scans={self._publish_raw_scans}, '
            f'obstacle_filter={self._obstacle_filter_enabled})'
        )

    def _reload_calibration(self) -> None:
        self._calibration.load_latest_tare()
        tare_loaded = {
            name: name in self._calibration.tare_offsets
            for name in self._projector.sensor_names
        }
        self._diagnostics.set_tare_loaded(tare_loaded)
        loaded = sum(tare_loaded.values())
        self.get_logger().info(
            f'Loaded tare for {loaded}/{len(tare_loaded)} sensors'
        )

    def _reload_calibration_callback(self, request, response):
        del request
        try:
            self._reload_calibration()
            response.success = True
            response.message = 'Calibration tare reloaded from fleet directory'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f'Failed to reload calibration: {exc}')
        return response

    def _apply_tare_fn(self, ranges: np.ndarray, sensor_name: str) -> np.ndarray:
        return self._calibration.apply_tare(ranges, sensor_name)

    def _timer_callback(self) -> None:
        self._robot_client.pull_status()
        status = self._robot_client.line_sensor_loop.status
        stamp = self.get_clock().now().to_msg()

        if self._publish_raw_scans:
            for sensor_name in self._sensor_names:
                sensor_status = status.get(sensor_name, {})
                ranges = sensor_status.get('ranges', [])
                if not ranges:
                    continue
                frame_id = sensor_name_to_optical_frame(sensor_name)
                scan_msg = build_raw_laserscan(
                    ranges=np.asarray(ranges),
                    stamp=stamp,
                    frame_id=frame_id,
                    horizontal_fov_deg=self._horizontal_fov_deg,
                    range_min=self._scan_range_min,
                    range_max=self._scan_range_max,
                )
                self._scan_pubs[sensor_name].publish(scan_msg)

        apply_tare = self._apply_tare_fn if self._apply_tare else None
        fused, obstacle_pts = self._projector.project_arrays(
            status=status,
            apply_tare=apply_tare,
        )
        header = Header(stamp=stamp, frame_id=self._base_frame)
        self._points_pub.publish(_numpy_to_pointcloud2(fused, header))

        if self._publish_unfiltered_obstacles and self._obstacle_unfiltered_pub is not None:
            self._obstacle_unfiltered_pub.publish(
                _numpy_to_pointcloud2(obstacle_pts, header),
            )

        if self._obstacle_filter is not None:
            obstacle_pts = self._obstacle_filter.filter(obstacle_pts)

        self._obstacle_pub.publish(_numpy_to_pointcloud2(obstacle_pts, header))
        self._diagnostics.update_status(status)

    def destroy_node(self) -> bool:
        if hasattr(self, '_robot_client') and self._robot_client is not None:
            self._robot_client.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LineSensorNode()
        rclpy.spin(node)
    except RuntimeError as exc:
        print(f'line_sensor_node failed to start: {exc}', file=sys.stderr)
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
