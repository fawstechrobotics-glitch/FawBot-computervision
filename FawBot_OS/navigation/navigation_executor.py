"""Navigation Executor managing non-blocking, asynchronous mission execution."""
import math
import logging
from typing import List, Optional
from PyQt5.QtCore import QObject, pyqtSignal

from models.path import Command
from robot.robot_controller import RobotController
from robot.robot_state import RobotState

logger = logging.getLogger(__name__)


class NavigationExecutor(QObject):
    """
    Executes a sequence of hardware robot commands (MOVE, TURN) sequentially.
    Handles asynchronous completion feedback, pausing, resuming, stopping, and safety halts.
    """
    started = pyqtSignal()
    step_started = pyqtSignal(int, int, str)    # current_step, total_steps, description
    step_completed = pyqtSignal(int, int)       # current_step, total_steps
    paused = pyqtSignal()
    resumed = pyqtSignal()
    completed = pyqtSignal()
    stopped = pyqtSignal()
    safety_halted = pyqtSignal()
    progress_updated = pyqtSignal(float)       # 0.0 to 1.0

    def __init__(self, controller: RobotController, state: RobotState):
        super().__init__()
        self.controller = controller
        self.state = state

        self._commands: List[Command] = []
        self._current_step: int = 0
        self._is_running: bool = False
        self._is_paused: bool = False

        # Connect controller feedback signals
        self.controller.motion_completed.connect(self._handle_motion_completed)
        self.controller.safety_halt_received.connect(self._handle_safety_halt)

    @property
    def is_running(self) -> bool:
        return self._is_running

    @property
    def is_paused(self) -> bool:
        return self._is_paused

    @property
    def current_step(self) -> int:
        return self._current_step

    @property
    def total_steps(self) -> int:
        return len(self._commands)

    def set_commands(self, commands: List[Command]):
        """Load the list of commands to be executed."""
        self._commands = list(commands)
        self._current_step = 0

    def start(self):
        """Start or restart execution from the beginning."""
        if not self._commands:
            logger.warning("Cannot start mission execution: command list is empty.")
            return

        if self.state.safety_halted:
            logger.warning("Cannot start mission execution: Robot is in SAFETY HALT condition.")
            return

        self._is_running = True
        self._is_paused = False
        self._current_step = 0
        self.state.set_executing(True)
        self.state.set_paused(False)

        logger.info(f"Mission execution started with {len(self._commands)} commands.")
        self.started.emit()
        self._dispatch_current_step()

    def pause(self):
        """Pause mission execution, preserving the current step index."""
        if not self._is_running or self._is_paused:
            return

        self._is_paused = True
        self.state.set_paused(True)
        self.controller.stop()
        logger.info(f"Mission paused at step {self._current_step + 1}/{len(self._commands)}")
        self.paused.emit()

    def resume(self):
        """Resume mission execution from the paused step."""
        if not self._is_running or not self._is_paused:
            return

        if self.state.safety_halted:
            logger.warning("Cannot resume: Robot is in SAFETY HALT condition.")
            return

        self._is_paused = False
        self.state.set_paused(False)
        self.state.set_executing(True)
        logger.info(f"Mission resumed at step {self._current_step + 1}/{len(self._commands)}")
        self.resumed.emit()
        self._dispatch_current_step()

    def stop(self):
        """Stop and abort mission execution."""
        if not self._is_running:
            return

        self._is_running = False
        self._is_paused = False
        self.state.set_executing(False)
        self.state.set_paused(False)
        self.controller.stop()
        logger.info("Mission execution stopped by user.")
        self.stopped.emit()

    def _dispatch_current_step(self):
        """Send the active step command to the controller."""
        if not self._is_running or self._is_paused:
            return

        if self._current_step >= len(self._commands):
            self._finish_execution()
            return

        cmd = self._commands[self._current_step]
        desc = f"{cmd.type} {cmd.value:.1f}{'cm' if cmd.type == 'MOVE' else '°'}"
        self.step_started.emit(self._current_step + 1, len(self._commands), desc)
        logger.info(f"Executing step {self._current_step + 1}/{len(self._commands)}: {desc}")

        if cmd.type == "TURN":
            self.controller.execute_turn(cmd.value)
        elif cmd.type == "MOVE":
            # Calculate target end coordinates from current robot state
            curr_pos = (self.state.x, self.state.y)
            rad = self.state.pose.heading_rad
            dx = cmd.value * cmd.direction * math.cos(rad)
            dy = cmd.value * cmd.direction * math.sin(rad)
            target = (curr_pos[0] + dx, curr_pos[1] + dy)
            self.controller.execute_move(cmd.value, target, cmd.direction)
        else:
            logger.error(f"Unknown command type: {cmd.type}")
            self._advance_step()

    def _handle_motion_completed(self, mode: str):
        """Slot invoked when controller finishes physical move or turn."""
        if not self._is_running or self._is_paused:
            return

        self.step_completed.emit(self._current_step + 1, len(self._commands))
        progress = (self._current_step + 1) / float(len(self._commands))
        self.progress_updated.emit(min(1.0, progress))

        self._advance_step()

    def _advance_step(self):
        """Move pointer to next step and dispatch."""
        self._current_step += 1
        if self._current_step >= len(self._commands):
            self._finish_execution()
        else:
            self._dispatch_current_step()

    def _finish_execution(self):
        """All commands executed successfully."""
        self._is_running = False
        self._is_paused = False
        self.state.set_executing(False)
        self.state.set_paused(False)
        self.progress_updated.emit(1.0)
        logger.info("All mission commands completed successfully.")
        self.completed.emit()

    def _handle_safety_halt(self):
        """Hardware halt detected: abort execution immediately."""
        self._is_running = False
        self._is_paused = False
        self.state.set_executing(False)
        logger.warning("Execution aborted due to SAFETY HALT feedback from robot hardware.")
        self.safety_halted.emit()
