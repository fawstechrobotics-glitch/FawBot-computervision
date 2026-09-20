"""Central Robot Controller managing physical commands, telemetry, live interpolation, and watchdog recovery."""
import math
import logging
from typing import Optional, Tuple
from PyQt5.QtCore import QObject, QTimer, pyqtSignal

import config.settings as settings
from robot.udp_communication import UDPCommunication
from robot.robot_state import RobotState
from models.pose import normalize_angle_deg

logger = logging.getLogger(__name__)


class RobotController(QObject):
    """Central interface between UI/Navigation and the physical FawBot hardware."""
    motion_started = pyqtSignal(str)           # "MOVE" or "TURN"
    motion_completed = pyqtSignal(str)         # "MOVE" or "TURN"
    safety_halt_received = pyqtSignal()
    status_message_updated = pyqtSignal(str)

    def __init__(self, comm: UDPCommunication, state: RobotState):
        super().__init__()
        self.comm = comm
        self.state = state

        # Connect low-level UDP feedback
        self.comm.feedback_received.connect(self._handle_feedback)
        self.comm.connection_status_changed.connect(self.state.set_connected)

        # Animation & Motion tracking (~30 FPS)
        self.anim_timer = QTimer()
        self.anim_timer.setInterval(settings.ANIMATION_INTERVAL_MS)
        self.anim_timer.timeout.connect(self._update_live_animation)

        # Command Watchdog Timer to prevent lockup if UDP feedback packet is dropped
        self.watchdog_timer = QTimer()
        self.watchdog_timer.setSingleShot(True)
        self.watchdog_timer.timeout.connect(self._on_command_timeout)

        self.anim_mode: Optional[str] = None  # "MOVE" or "TURN"
        self.start_pos: Tuple[float, float] = (0.0, 0.0)
        self.end_pos: Tuple[float, float] = (0.0, 0.0)
        self.start_angle: float = 0.0
        self.target_angle: float = 0.0
        self.anim_progress: float = 0.0
        self.step_increment: float = 0.01

        self.pending_distance: float = 0.0
        self.pending_turn: float = 0.0

    def _start_watchdog(self, duration_sec: float):
        """Arm the watchdog timer based on expected motion duration with a safety padding."""
        # Minimum timeout of 3.5s; padding of 1.5x + 2.0s for stepper acceleration ramping
        timeout_sec = max(duration_sec * 1.5 + 2.0, 3.5)
        self.watchdog_timer.start(int(timeout_sec * 1000))

    def teleop_move(self, distance_cm: float):
        """Execute linear movement in current heading direction by distance_cm."""
        # Auto-spawn if not yet placed on grid
        if self.state.pose is None:
            self.state.set_pose(20.0, 20.0, 0.0, set_initial_if_unset=True)
            self.status_message_updated.emit("Robot auto-spawned at (20.0, 20.0) | Heading 0.0°")

        # If previous motion is still executing, auto-complete it so new manual command runs immediately
        if self.state.is_executing and not self.state.is_paused:
            logger.info("New teleop move requested while executing - auto-resolving previous command.")
            self._on_command_timeout()

        rad = self.state.pose.heading_rad
        dx = distance_cm * math.cos(rad)
        dy = distance_cm * math.sin(rad)

        target_x = self.state.x + dx
        target_y = self.state.y + dy

        self.start_pos = (self.state.x, self.state.y)
        self.end_pos = (target_x, target_y)
        self.pending_distance = abs(distance_cm)

        # Kinematic timing
        total_steps = abs(distance_cm) * settings.STEPS_PER_CM
        duration_sec = (total_steps * settings.STEP_DELAY_US) / 1000000.0
        frames_total = (duration_sec * 1000.0) / float(settings.ANIMATION_INTERVAL_MS)
        self.step_increment = 1.0 / max(1.0, frames_total)

        self.state.set_executing(True)
        self.state.set_safety_halted(False)
        self.state.start_new_stroke(self.start_pos)

        msg = f"TELEOP MOVE | Distance: {distance_cm:+.1f} cm"
        self.status_message_updated.emit(msg)
        logger.info(msg)

        self.anim_progress = 0.0
        self.anim_mode = "MOVE"
        self.anim_timer.start()
        self._start_watchdog(duration_sec)

        direction_flag = 1.0 if distance_cm >= 0 else -1.0
        self.comm.send_command(f"MOVE:{abs(distance_cm):.1f}:{direction_flag}")
        self.motion_started.emit("MOVE")

    def rotate_robot_delta(self, delta_angle: float):
        """Rotate robot in place by delta_angle degrees."""
        # Auto-spawn if not yet placed on grid
        if self.state.pose is None:
            self.state.set_pose(20.0, 20.0, 0.0, set_initial_if_unset=True)
            self.status_message_updated.emit("Robot auto-spawned at (20.0, 20.0) | Heading 0.0°")

        # If previous motion is still executing, auto-complete it so new manual command runs immediately
        if self.state.is_executing and not self.state.is_paused:
            logger.info("New teleop turn requested while executing - auto-resolving previous command.")
            self._on_command_timeout()

        self.pending_turn = delta_angle
        self.state.set_executing(True)
        self.state.set_safety_halted(False)

        # Kinematic timing
        arc_length_cm = (math.pi * settings.WHEEL_BASE_CM) * (abs(self.pending_turn) / 360.0)
        total_steps = arc_length_cm * settings.STEPS_PER_CM
        duration_sec = (total_steps * settings.STEP_DELAY_US) / 1000000.0
        frames_total = (duration_sec * 1000.0) / float(settings.ANIMATION_INTERVAL_MS)
        self.step_increment = 1.0 / max(1.0, frames_total)

        msg = f"TELEOP TURN | Delta: {self.pending_turn:+.1f}°"
        self.status_message_updated.emit(msg)
        logger.info(msg)

        self.start_angle = self.state.heading
        self.target_angle = normalize_angle_deg(self.state.heading + self.pending_turn)

        self.anim_progress = 0.0
        self.anim_mode = "TURN"
        self.anim_timer.start()
        self._start_watchdog(duration_sec)

        self.comm.send_command(f"TURN:{self.pending_turn:.1f}")
        self.motion_started.emit("TURN")

    def execute_move(self, distance_cm: float, target_pos: Tuple[float, float], direction: float = 1.0):
        """Start a planned linear move segment."""
        self.pending_distance = abs(distance_cm)
        self.start_pos = (self.state.x, self.state.y)
        self.end_pos = target_pos

        total_steps = self.pending_distance * settings.STEPS_PER_CM
        duration_sec = (total_steps * settings.STEP_DELAY_US) / 1000000.0
        frames_total = (duration_sec * 1000.0) / float(settings.ANIMATION_INTERVAL_MS)
        self.step_increment = 1.0 / max(1.0, frames_total)

        self.state.set_executing(True)
        self.state.start_new_stroke(self.start_pos)
        self.anim_progress = 0.0
        self.anim_mode = "MOVE"
        self.anim_timer.start()
        self._start_watchdog(duration_sec)

        self.comm.send_command(f"MOVE:{self.pending_distance:.1f}:{direction:.1f}")
        self.motion_started.emit("MOVE")

    def execute_turn(self, turn_angle_deg: float):
        """Start a planned in-place turn segment."""
        self.pending_turn = turn_angle_deg

        arc_length_cm = (math.pi * settings.WHEEL_BASE_CM) * (abs(self.pending_turn) / 360.0)
        total_steps = arc_length_cm * settings.STEPS_PER_CM
        duration_sec = (total_steps * settings.STEP_DELAY_US) / 1000000.0
        frames_total = (duration_sec * 1000.0) / float(settings.ANIMATION_INTERVAL_MS)
        self.step_increment = 1.0 / max(1.0, frames_total)

        self.state.set_executing(True)
        self.start_angle = self.state.heading
        self.target_angle = normalize_angle_deg(self.state.heading + self.pending_turn)

        self.anim_progress = 0.0
        self.anim_mode = "TURN"
        self.anim_timer.start()
        self._start_watchdog(duration_sec)

        self.comm.send_command(f"TURN:{self.pending_turn:.1f}")
        self.motion_started.emit("TURN")

    def stop(self):
        """Halt live motion interpolation, disarm watchdog, and immediately unfreeze controls."""
        self.watchdog_timer.stop()
        self.anim_timer.stop()
        self.anim_mode = None
        self.state.set_executing(False)
        self.state.set_paused(False)
        self.pending_distance = 0.0
        self.pending_turn = 0.0
        self.status_message_updated.emit("STOPPED | Controls unlocked")
        logger.info("Robot controller stopped and controls unlocked.")

    def _update_live_animation(self):
        """Interpolate robot position and heading at 30 FPS for visual smoothness."""
        if not self.anim_mode or self.state.is_paused:
            return

        if self.anim_progress < 1.0:
            self.anim_progress += self.step_increment

        factor = min(1.0, self.anim_progress)

        if self.anim_mode == "TURN":
            # Shortest arc angular interpolation
            delta = self.target_angle - self.start_angle
            while delta > 180.0: delta -= 360.0
            while delta < -180.0: delta += 360.0
            curr_angle = normalize_angle_deg(self.start_angle + delta * factor)
            self.state.set_pose(self.state.x, self.state.y, curr_angle, set_initial_if_unset=False)

        elif self.anim_mode == "MOVE":
            curr_x = self.start_pos[0] + (self.end_pos[0] - self.start_pos[0]) * factor
            curr_y = self.start_pos[1] + (self.end_pos[1] - self.start_pos[1]) * factor
            self.state.set_pose(curr_x, curr_y, self.state.heading, set_initial_if_unset=False)
            self.state.append_trail_point((curr_x, curr_y))

    def _handle_feedback(self, msg: str):
        """Process robot telemetry and command completion responses."""
        logger.info(f"[Robot Telemetry Feedback]: {msg}")

        if msg.startswith("COMPLETED:TURN"):
            self.watchdog_timer.stop()
            self.anim_timer.stop()
            self.anim_mode = None
            if self.state.pose:
                self.state.set_pose(self.state.x, self.state.y, self.target_angle, set_initial_if_unset=False)
            self.pending_turn = 0.0
            if not self.state.is_returning_home and not self.state.is_backtracking and self.state.current_target is None:
                self.state.set_executing(False)
            self.motion_completed.emit("TURN")

        elif msg.startswith("COMPLETED:MOVE"):
            self.watchdog_timer.stop()
            self.anim_timer.stop()
            self.anim_mode = None
            if self.state.pose:
                self.state.set_pose(self.end_pos[0], self.end_pos[1], self.state.heading, set_initial_if_unset=False)
                self.state.append_trail_point(self.end_pos)
            self.pending_distance = 0.0
            if not self.state.is_returning_home and not self.state.is_backtracking and self.state.current_target is None:
                self.state.set_executing(False)
            self.motion_completed.emit("MOVE")

        elif msg.startswith("ALERT:SAFETY_HALT"):
            self.watchdog_timer.stop()
            self.anim_timer.stop()
            self.anim_mode = None
            self.state.set_safety_halted(True)
            self.status_message_updated.emit("SAFETY HALT DETECTED | Hardware sensors interrupted movement")
            self.safety_halt_received.emit()

    def _on_command_timeout(self):
        """Fires if no UDP completion packet was received within timeout window (network packet drop recovery)."""
        self.watchdog_timer.stop()
        self.anim_timer.stop()

        mode = self.anim_mode
        self.anim_mode = None

        if mode == "TURN":
            if self.state.pose:
                self.state.set_pose(self.state.x, self.state.y, self.target_angle, set_initial_if_unset=False)
            self.pending_turn = 0.0
            if not self.state.is_returning_home and not self.state.is_backtracking and self.state.current_target is None:
                self.state.set_executing(False)
            logger.warning("[Watchdog] TURN feedback timed out; auto-recovered controller state.")
            self.status_message_updated.emit("READY | Turn completed (network feedback auto-recovered)")
            self.motion_completed.emit("TURN")

        elif mode == "MOVE":
            if self.state.pose:
                self.state.set_pose(self.end_pos[0], self.end_pos[1], self.state.heading, set_initial_if_unset=False)
                self.state.append_trail_point(self.end_pos)
            self.pending_distance = 0.0
            if not self.state.is_returning_home and not self.state.is_backtracking and self.state.current_target is None:
                self.state.set_executing(False)
            logger.warning("[Watchdog] MOVE feedback timed out; auto-recovered controller state.")
            self.status_message_updated.emit("READY | Move completed (network feedback auto-recovered)")
            self.motion_completed.emit("MOVE")
        else:
            self.state.set_executing(False)
            self.status_message_updated.emit("READY | Controls unlocked")
