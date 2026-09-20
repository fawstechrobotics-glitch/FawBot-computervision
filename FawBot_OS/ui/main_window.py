"""Main Window hosting Navigation tabs between Manual Control and Mission Path Planning."""
import logging
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QLabel, QFrame, QApplication, QLineEdit, QDoubleSpinBox, QSpinBox
)
from PyQt5.QtCore import Qt, QEvent

import config.settings as settings
from ui.styles import GAZEBO_DARK_STYLESHEET
from ui.manual_control_page import ManualControlPage
from ui.mission_planner_page import MissionPlannerPage
from ui.swarming_page import SwarmingPage
from robot.udp_communication import UDPCommunication
from robot.robot_state import RobotState
from robot.robot_controller import RobotController
from robot.fleet import RobotRoster, RobotSpec
from robot.fleet_controller import RobotFleet
from navigation.mission_manager import MissionManager

logger = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    """Primary application window hosting tabbed pages, global telemetry, and universal keyboard teleoperation."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle(settings.WINDOW_TITLE)
        self.resize(settings.DEFAULT_WINDOW_WIDTH, settings.DEFAULT_WINDOW_HEIGHT)
        self.setStyleSheet(GAZEBO_DARK_STYLESHEET)
        self.setFocusPolicy(Qt.StrongFocus)

        # 1. Initialize Central Architecture Layers
        roster = RobotRoster.from_file()
        if not roster.robots:
            roster = RobotRoster([RobotSpec("default", settings.ROBOT_HOST)])
        self.fleet = RobotFleet(roster)
        self.comm = self.fleet.comms[self.fleet.primary_id]
        self.state = self.fleet.primary_state
        self.controller = self.fleet.primary_controller
        self.mission_manager = MissionManager(self.controller, self.state, roster)
        self.mission_manager.set_fleet(self.fleet)

        # 2. Build UI
        self.init_ui()
        self._connect_header_telemetry()

        # 3. Install application-wide event filter so Arrow keys and WASD work reliably everywhere
        app = QApplication.instance()
        if app:
            app.installEventFilter(self)

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(4)

        # --- GLOBAL TOP BAR ---
        top_bar = QFrame()
        top_bar.setStyleSheet("background-color: #12161f; border: 1px solid #283344; border-radius: 4px; padding: 2px;")
        top_layout = QHBoxLayout(top_bar)
        top_layout.setContentsMargins(8, 4, 8, 4)

        lbl_app_title = QLabel("🤖 FAWBOT OS")
        lbl_app_title.setStyleSheet("font-size: 13px; font-weight: bold; color: #00d2ff;")
        top_layout.addWidget(lbl_app_title)

        top_layout.addStretch()

        self.lbl_global_conn = QLabel(f"UDP: {settings.ROBOT_HOST}:{settings.UDP_PORT} (READY)")
        self.lbl_global_conn.setStyleSheet("color: #00ffaa; font-family: monospace; font-weight: bold;")
        top_layout.addWidget(self.lbl_global_conn)

        top_layout.addSpacing(20)

        self.lbl_global_pose = QLabel("Pose: X: -- | Y: -- | Heading: 0.0°")
        self.lbl_global_pose.setStyleSheet("color: #ffcc00; font-family: monospace; font-weight: bold;")
        top_layout.addWidget(self.lbl_global_pose)

        top_layout.addSpacing(20)

        self.lbl_global_safety = QLabel("SAFETY: OK")
        self.lbl_global_safety.setStyleSheet("color: #00ffaa; font-weight: bold; padding: 2px 6px; border-radius: 3px; background-color: #0a2e14;")
        top_layout.addWidget(self.lbl_global_safety)

        main_layout.addWidget(top_bar)

        # --- TAB NAVIGATION ---
        self.tabs = QTabWidget()
        self.tabs.setFocusPolicy(Qt.NoFocus)

        self.manual_page = ManualControlPage(
            self.controller,
            self.state,
            home_callback=self.mission_manager.return_all_robots_home,
            backtrack_callback=self.mission_manager.backtrack_all_robots,
        )
        self.mission_page = MissionPlannerPage(self.mission_manager, self.controller, self.state)
        self.swarming_page = SwarmingPage(self.fleet)

        # Keep every configured robot visible on both maps. The primary robot
        # continues to drive the existing single-robot controls.
        for robot_id, robot_state in self.fleet.states.items():
            robot_state.pose_changed.connect(
                lambda x, y, h, robot_id=robot_id: self._update_robot_on_maps(robot_id, x, y, h)
            )
            robot_state.path_history_updated.connect(
                lambda robot_id=robot_id: self._update_robot_trail(robot_id)
            )
            if robot_state.pose:
                self._update_robot_on_maps(
                    robot_id, robot_state.pose.x, robot_state.pose.y, robot_state.pose.heading
                )

        self.tabs.addTab(self.manual_page, "🎮 Manual Control")
        self.tabs.addTab(self.mission_page, "🗺️ Mission Path Planning")
        self.tabs.addTab(self.swarming_page, "💃 Swarming")

        main_layout.addWidget(self.tabs)

    def _update_robot_on_maps(self, robot_id: str, x: float, y: float, heading: float):
        self.manual_page.map_widget.set_named_robot_pose(robot_id, x, y, heading)
        self.mission_page.map_widget.set_named_robot_pose(robot_id, x, y, heading)
        state = self.fleet.states[robot_id]
        self.manual_page.map_widget.set_named_path_history(robot_id, state.path_history)
        self.mission_page.map_widget.set_named_path_history(robot_id, state.path_history)

    def _update_robot_trail(self, robot_id: str):
        state = self.fleet.states[robot_id]
        self.manual_page.map_widget.set_named_path_history(robot_id, state.path_history)
        self.mission_page.map_widget.set_named_path_history(robot_id, state.path_history)
        if self.manual_page.backtrack_callback:
            has_trail = any(robot_state.path_history for robot_state in self.fleet.states.values())
            any_running = any(robot_state.is_executing for robot_state in self.fleet.states.values())
            self.manual_page.btn_backtrack.setEnabled(has_trail and not any_running)

    def _connect_header_telemetry(self):
        # Update connection status
        self.comm.connection_status_changed.connect(self._on_conn_changed)

        # Update global coordinates
        self.state.pose_changed.connect(
            lambda x, y, h: self.lbl_global_pose.setText(f"Pose: X: {x:.1f} cm | Y: {y:.1f} cm | Heading: {h:.1f}°")
        )

        # Update safety banner
        self.state.safety_halt_triggered.connect(self._on_safety_alert)
        self.state.flags_changed.connect(self._on_flags_changed)

    def _on_conn_changed(self, connected: bool):
        if connected:
            self.lbl_global_conn.setText(f"UDP: {settings.ROBOT_HOST}:{settings.UDP_PORT} (ONLINE)")
            self.lbl_global_conn.setStyleSheet("color: #00ffaa; font-family: monospace; font-weight: bold;")
        else:
            self.lbl_global_conn.setText(f"UDP: {settings.ROBOT_HOST}:{settings.UDP_PORT} (OFFLINE)")
            self.lbl_global_conn.setStyleSheet("color: #ff3366; font-family: monospace; font-weight: bold;")

    def _on_safety_alert(self):
        self.lbl_global_safety.setText("SAFETY: HALTED")
        self.lbl_global_safety.setStyleSheet("color: #ffffff; font-weight: bold; padding: 2px 6px; border-radius: 3px; background-color: #800a20;")

    def _on_flags_changed(self):
        if not self.state.safety_halted:
            if self.state.is_executing:
                self.lbl_global_safety.setText("STATUS: RUNNING")
                self.lbl_global_safety.setStyleSheet("color: #00d2ff; font-weight: bold; padding: 2px 6px; border-radius: 3px; background-color: #0d2838;")
            elif self.state.is_recording:
                self.lbl_global_safety.setText("STATUS: RECORDING")
                self.lbl_global_safety.setStyleSheet("color: #ff9900; font-weight: bold; padding: 2px 6px; border-radius: 3px; background-color: #38240d;")
            else:
                self.lbl_global_safety.setText("SAFETY: OK")
                self.lbl_global_safety.setStyleSheet("color: #00ffaa; font-weight: bold; padding: 2px 6px; border-radius: 3px; background-color: #0a2e14;")

    def eventFilter(self, watched, event):
        """Intercept application-wide key presses so Arrow keys and WASD always drive the robot."""
        if event.type() == QEvent.KeyPress:
            focused = QApplication.focusWidget()
            # Allow text entry widgets to handle input normally
            if isinstance(focused, (QLineEdit, QDoubleSpinBox, QSpinBox)):
                return super().eventFilter(watched, event)

            key = event.key()
            if key in (Qt.Key_Up, Qt.Key_Down, Qt.Key_Left, Qt.Key_Right,
                       Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D, Qt.Key_Space):
                self._handle_global_teleop_key(key)
                return True  # Consume event to prevent Qt from moving button focus

        return super().eventFilter(watched, event)

    def _handle_global_teleop_key(self, key):
        """Dispatches keyboard driving commands globally."""
        # Ensure robot is placed/spawned
        if self.state.pose is None:
            self.state.set_pose(20.0, 20.0, 0.0, set_initial_if_unset=True)
            self.manual_page.map_widget.set_initial_pos((20.0, 20.0))
            self.mission_page.map_widget.set_initial_pos((20.0, 20.0))

        if key in (Qt.Key_Up, Qt.Key_W):
            self.controller.teleop_move(10.0)
            if self.mission_manager.is_recording:
                self.mission_manager.record_drive_step(
                    self.state.x, self.state.y, self.state.heading, action="MOVE", distance=10.0
                )
        elif key in (Qt.Key_Down, Qt.Key_S):
            self.controller.teleop_move(-10.0)
            if self.mission_manager.is_recording:
                self.mission_manager.record_drive_step(
                    self.state.x, self.state.y, self.state.heading, action="MOVE", distance=10.0
                )
        elif key in (Qt.Key_Left, Qt.Key_A):
            self.controller.rotate_robot_delta(15.0)
            if self.mission_manager.is_recording:
                self.mission_manager.record_drive_step(
                    self.state.x, self.state.y, self.state.heading, action="TURN", rotation=15.0
                )
        elif key in (Qt.Key_Right, Qt.Key_D):
            self.controller.rotate_robot_delta(-15.0)
            if self.mission_manager.is_recording:
                self.mission_manager.record_drive_step(
                    self.state.x, self.state.y, self.state.heading, action="TURN", rotation=15.0
                )
        elif key == Qt.Key_Space:
            if self.tabs.currentIndex() == 0:
                self.manual_page.toggle_mission_execution()
            else:
                if self.mission_manager.executor.is_running:
                    if self.mission_manager.executor.is_paused:
                        self.mission_manager.resume_mission()
                    else:
                        self.mission_manager.pause_mission()

    def closeEvent(self, event):
        """Clean up background threads and sockets on window close."""
        logger.info("Application closing...")
        self.controller.stop()
        self.mission_page.preview_timer.stop()
        self.swarming_page.close()
        self.fleet.close()
        event.accept()
