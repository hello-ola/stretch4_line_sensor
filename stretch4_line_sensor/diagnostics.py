"""Diagnostic reporting for the line sensor bridge node."""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater


class LineSensorDiagnostics:
    """Wrap diagnostic_updater tasks for line_sensor_loop status."""

    def __init__(self, node, sensor_names: list[str]):
        self._node = node
        self._sensor_names = sensor_names
        self._status: dict = {}
        self._tare_loaded: dict[str, bool] = {
            name: False for name in sensor_names
        }
        self._last_frame_advance_err = 0
        self._last_not_six_sensors_err = 0

        self._updater = Updater(node)
        self._updater.setHardwareID('stretch4_line_sensor')
        self._updater.add('Line Sensor', self._check_overall)
        for name in sensor_names:
            self._updater.add(
                f'Line Sensor {name}',
                self._make_sensor_check(name),
            )

    @property
    def updater(self) -> Updater:
        return self._updater

    def set_tare_loaded(self, tare_loaded: dict[str, bool]) -> None:
        self._tare_loaded = dict(tare_loaded)

    def update_status(self, status: dict) -> None:
        self._status = status
        self._updater.update()

    def _check_overall(self, stat: DiagnosticStatus) -> DiagnosticStatus:
        status = self._status
        now = time.time()

        rate_hz = status.get('rate_hz', 0.0)
        stat.add('frame_rate_hz', f'{rate_hz:.1f}')

        last_frame_time = status.get('last_frame_time', 0.0)
        if last_frame_time > 0:
            frame_age_s = now - last_frame_time
        else:
            frame_age_s = float('inf')
        stat.add('last_frame_age_ms', f'{frame_age_s * 1000.0:.1f}')

        frame_advance_err = status.get('frame_advance_err', 0)
        not_six_sensors_err = status.get('not_six_sensors_err', 0)
        stat.add('frame_advance_err', str(frame_advance_err))
        stat.add('not_six_sensors_err', str(not_six_sensors_err))

        missing_tare = [
            n for n, loaded in self._tare_loaded.items() if not loaded
        ]
        if missing_tare:
            cal_text = f'missing: {missing_tare}'
        else:
            cal_text = 'ok'
        stat.add('calibration', cal_text)

        level = DiagnosticStatus.OK
        msg = 'Line sensor operating normally'

        if rate_hz < 10.0 or frame_age_s > 0.5:
            level = DiagnosticStatus.ERROR
            msg = 'Line sensor data stale or rate too low'
        elif rate_hz < 25.0 or frame_age_s > 0.1:
            level = DiagnosticStatus.WARN
            msg = 'Line sensor rate or latency degraded'

        if frame_advance_err > self._last_frame_advance_err:
            if level < DiagnosticStatus.WARN:
                level = DiagnosticStatus.WARN
                msg = 'Frame sync errors detected'
        if not_six_sensors_err > self._last_not_six_sensors_err:
            if level < DiagnosticStatus.WARN:
                level = DiagnosticStatus.WARN
                msg = 'Incomplete frames detected'

        if missing_tare:
            if level < DiagnosticStatus.WARN:
                level = DiagnosticStatus.WARN
                msg = 'Calibration tare missing for one or more sensors'

        self._last_frame_advance_err = frame_advance_err
        self._last_not_six_sensors_err = not_six_sensors_err

        stat.summary(level, msg)
        return stat

    def _make_sensor_check(self, sensor_name: str):
        def check(stat: DiagnosticStatus) -> DiagnosticStatus:
            sensor_status = self._status.get(sensor_name, {})
            if not isinstance(sensor_status, dict):
                sensor_status = {}
            ts_last_read = sensor_status.get('ts_last_read', 0.0)
            frame_id = sensor_status.get('frame_id', 0)
            rate_hz = sensor_status.get('rate_hz', 0.0)

            if ts_last_read > 0:
                age_s = time.time() - ts_last_read
            else:
                age_s = float('inf')
            stat.add('ts_last_read_age_ms', f'{age_s * 1000.0:.1f}')
            stat.add('frame_id', str(frame_id))
            stat.add('rate_hz', f'{rate_hz:.1f}')
            loaded = self._tare_loaded.get(sensor_name, False)
            stat.add('tare_loaded', str(loaded))

            if age_s > 0.2:
                stat.summary(
                    DiagnosticStatus.ERROR,
                    f'{sensor_name} data stale',
                )
            else:
                stat.summary(
                    DiagnosticStatus.OK,
                    f'{sensor_name} ok',
                )
            return stat

        return check
