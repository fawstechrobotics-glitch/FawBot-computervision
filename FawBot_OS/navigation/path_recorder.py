"""Path Recorder for Teach/Record Mode with intelligent point throttling."""
import time
import math
import logging
from typing import Optional, List
from PyQt5.QtCore import QObject, pyqtSignal

import config.settings as settings
from models.path import Path, PathPoint, Command
from models.pose import Pose, normalize_angle_deg

logger = logging.getLogger(__name__)


class PathRecorder(QObject):
    """
    Captures manual robot driving trajectories while intelligently suppressing
    redundant/duplicate coordinates based on distance, angle, and time thresholds.
    """
    point_recorded = pyqtSignal(object)           # Emits PathPoint
    recording_state_changed = pyqtSignal(bool)    # True when recording starts, False when stops

    def __init__(
        self,
        min_distance_cm: float = settings.PATH_RECORD_DISTANCE_THRESHOLD,
        min_angle_deg: float = settings.PATH_RECORD_ANGLE_THRESHOLD,
        min_interval_sec: float = settings.PATH_RECORD_TIME_INTERVAL
    ):
        super().__init__()
        self.min_distance_cm = min_distance_cm
        self.min_angle_deg = min_angle_deg
        self.min_interval_sec = min_interval_sec

        self._is_recording = False
        self._path = Path()
        self._last_recorded_point: Optional[PathPoint] = None
        self._start_time: float = 0.0

    @property
    def is_recording(self) -> bool:
        return self._is_recording

    @property
    def path(self) -> Path:
        return self._path

    def start_recording(self, current_pose: Optional[Pose] = None):
        """Begin capturing path trajectory."""
        self._path.clear()
        self._is_recording = True
        self._start_time = time.time()
        self._last_recorded_point = None

        logger.info("Path recording started.")
        self.recording_state_changed.emit(True)

        if current_pose is not None:
            initial_pt = PathPoint(
                x=current_pose.x,
                y=current_pose.y,
                heading=current_pose.heading,
                timestamp=0.0,
                action="START",
                distance=0.0,
                rotation=0.0
            )
            self._path.append(initial_pt)
            self._last_recorded_point = initial_pt
            self.point_recorded.emit(initial_pt)

    def stop_recording(self) -> Path:
        """Finish capturing path trajectory and return recorded Path."""
        self._is_recording = False
        logger.info(f"Path recording stopped. Captured {len(self._path.points)} points.")
        self.recording_state_changed.emit(False)
        return self._path

    def clear(self):
        """Clear all recorded points."""
        self._path.clear()
        self._last_recorded_point = None

    def record_manual_event(
        self,
        x: float,
        y: float,
        heading: float,
        action: str = "MOVE",
        distance: float = 0.0,
        rotation: float = 0.0
    ) -> bool:
        """
        Evaluate candidate pose against criteria and record if thresholds are met.
        Returns True if the point was recorded, False if throttled.
        """
        if not self._is_recording:
            return False

        norm_heading = normalize_angle_deg(heading)
        elapsed = time.time() - self._start_time

        if self._last_recorded_point is None:
            pt = PathPoint(
                x=x,
                y=y,
                heading=norm_heading,
                timestamp=elapsed,
                action=action,
                distance=distance,
                rotation=rotation
            )
            self._path.append(pt)
            self._last_recorded_point = pt
            self.point_recorded.emit(pt)
            return True

        # Compute delta from last recorded point
        dx = x - self._last_recorded_point.x
        dy = y - self._last_recorded_point.y
        dist = math.hypot(dx, dy)

        dh = abs(norm_heading - self._last_recorded_point.heading)
        while dh > 180.0: dh -= 360.0
        dh = abs(dh)

        dt = elapsed - self._last_recorded_point.timestamp

        # Check threshold criteria
        should_record = (
            dist >= self.min_distance_cm or
            dh >= self.min_angle_deg or
            (dt >= self.min_interval_sec and (dist > 0.1 or dh > 0.2)) or
            action in ("START", "GOAL", "WAYPOINT")
        )

        if should_record:
            pt = PathPoint(
                x=x,
                y=y,
                heading=norm_heading,
                timestamp=elapsed,
                action=action,
                distance=dist if distance == 0.0 else distance,
                rotation=dh if rotation == 0.0 else rotation
            )
            self._path.append(pt)
            self._last_recorded_point = pt
            self.point_recorded.emit(pt)
            return True

        return False
