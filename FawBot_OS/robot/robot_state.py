"""Centralized observable robot state model."""
from typing import Optional, List, Tuple
from PyQt5.QtCore import QObject, pyqtSignal

from models.pose import Pose, normalize_angle_deg


class RobotState(QObject):
    """Observable central state for the robot across the entire application."""
    pose_changed = pyqtSignal(float, float, float)       # x, y, heading
    flags_changed = pyqtSignal()                         # any status flag changed
    safety_halt_triggered = pyqtSignal()                 # hardware halt signal
    path_history_updated = pyqtSignal()                  # new trail point added

    def __init__(self):
        super().__init__()
        self._pose: Optional[Pose] = None
        self._initial_pose: Optional[Pose] = None
        self._current_target: Optional[Tuple[float, float]] = None

        # Status flags
        self._is_connected: bool = False
        self._is_executing: bool = False
        self._is_paused: bool = False
        self._is_returning_home: bool = False
        self._is_backtracking: bool = False
        self._safety_halted: bool = False
        self._is_recording: bool = False

        # Path history strokes
        self.path_history: List[List[Tuple[float, float]]] = []
        self.current_stroke: Optional[List[Tuple[float, float]]] = None

    @property
    def pose(self) -> Optional[Pose]:
        return self._pose

    @property
    def x(self) -> Optional[float]:
        return self._pose.x if self._pose else None

    @property
    def y(self) -> Optional[float]:
        return self._pose.y if self._pose else None

    @property
    def heading(self) -> float:
        return self._pose.heading if self._pose else 0.0

    @property
    def initial_pose(self) -> Optional[Pose]:
        return self._initial_pose

    @property
    def current_target(self) -> Optional[Tuple[float, float]]:
        return self._current_target

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_executing(self) -> bool:
        return self._is_executing

    @property
    def is_paused(self) -> bool:
        return self._is_paused

    @property
    def is_returning_home(self) -> bool:
        return self._is_returning_home

    @property
    def is_backtracking(self) -> bool:
        return self._is_backtracking

    @property
    def safety_halted(self) -> bool:
        return self._safety_halted

    @property
    def is_recording(self) -> bool:
        return self._is_recording

    # --- SETTERS WITH AUTOMATIC SIGNALS ---

    def set_pose(self, x: float, y: float, heading: float, set_initial_if_unset: bool = True):
        norm_heading = normalize_angle_deg(heading)
        if self._pose is None:
            self._pose = Pose(x=x, y=y, heading=norm_heading)
            if set_initial_if_unset and self._initial_pose is None:
                self._initial_pose = Pose(x=x, y=y, heading=norm_heading)
        else:
            self._pose.x = float(x)
            self._pose.y = float(y)
            self._pose.heading = norm_heading

        self.pose_changed.emit(self._pose.x, self._pose.y, self._pose.heading)

    def set_heading(self, heading: float):
        if self._pose:
            self._pose.heading = normalize_angle_deg(heading)
            self.pose_changed.emit(self._pose.x, self._pose.y, self._pose.heading)

    def set_initial_pose(self, pose: Pose):
        self._initial_pose = Pose(x=pose.x, y=pose.y, heading=normalize_angle_deg(pose.heading))

    def set_current_target(self, target: Optional[Tuple[float, float]]):
        self._current_target = target
        self.flags_changed.emit()

    def set_connected(self, connected: bool):
        if self._is_connected != connected:
            self._is_connected = connected
            self.flags_changed.emit()

    def set_executing(self, executing: bool):
        if self._is_executing != executing:
            self._is_executing = executing
            self.flags_changed.emit()

    def set_paused(self, paused: bool):
        if self._is_paused != paused:
            self._is_paused = paused
            self.flags_changed.emit()

    def set_returning_home(self, returning: bool):
        if self._is_returning_home != returning:
            self._is_returning_home = returning
            self.flags_changed.emit()

    def set_backtracking(self, backtracking: bool):
        if self._is_backtracking != backtracking:
            self._is_backtracking = backtracking
            self.flags_changed.emit()

    def set_safety_halted(self, halted: bool):
        self._safety_halted = halted
        if halted:
            self._is_executing = False
            self._is_paused = False
            self.safety_halt_triggered.emit()
        self.flags_changed.emit()

    def set_recording(self, recording: bool):
        if self._is_recording != recording:
            self._is_recording = recording
            self.flags_changed.emit()

    # --- TRAIL MANAGEMENT ---

    def start_new_stroke(self, start_pt: Tuple[float, float]):
        self.current_stroke = [start_pt]
        self.path_history.append(self.current_stroke)
        self.path_history_updated.emit()

    def append_trail_point(self, pt: Tuple[float, float]):
        if self.current_stroke is not None:
            self.current_stroke.append(pt)
            self.path_history_updated.emit()

    def clear_trails(self):
        self.path_history.clear()
        self.current_stroke = None
        self.path_history_updated.emit()

    def reset_world(self):
        """Reset robot pose, history, and status flags to spawn state."""
        self._pose = None
        self._initial_pose = None
        self._current_target = None
        self._is_executing = False
        self._is_paused = False
        self._is_returning_home = False
        self._is_backtracking = False
        self._safety_halted = False
        self._is_recording = False
        self.clear_trails()
        self.flags_changed.emit()
