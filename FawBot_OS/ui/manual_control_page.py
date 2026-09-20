"""Manual Control Page preserving all original Turtlesim operations and teleop controls."""
import math
import logging
from typing import Callable, List, Tuple, Optional
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QFrame, QGroupBox, QDoubleSpinBox, QScrollArea, QInputDialog, QSpinBox, QLineEdit
)
from PyQt5.QtCore import Qt

import config.settings as settings
from ui.map_widget import MapWidget
from robot.robot_controller import RobotController
from robot.robot_state import RobotState
from models.pose import normalize_angle_deg

logger = logging.getLogger(__name__)


class ManualControlPage(QWidget):
    """Refactored Manual Control page preserving complete original Turtlesim workflow."""

    def __init__(
        self,
        controller: RobotController,
        state: RobotState,
        home_callback: Optional[Callable[[], None]] = None,
        backtrack_callback: Optional[Callable[[], None]] = None,
        parent=None
    ):
        super().__init__(parent)
        self.controller = controller
        self.state = state
        self.home_callback = home_callback
        self.backtrack_callback = backtrack_callback

        # Local waypoint queue and backtracking state
        self.waypoint_queue: List[Tuple[float, float]] = []
        self.current_target: Optional[Tuple[float, float]] = None
        self.backtrack_waypoints: List[Tuple[float, float]] = []
        self.is_returning_home = False
        self.home_phase: Optional[str] = None  # 'TURN_TO_HOME', 'MOVE_TO_HOME', 'ALIGN_HOME'

        self.init_ui()
        self._connect_signals()

    def init_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(6)

        # --- LEFT PANEL: VIEWPORT ---
        view_container = QWidget()
        view_layout = QVBoxLayout(view_container)
        view_layout.setContentsMargins(0, 0, 0, 0)
        view_layout.setSpacing(4)

        # Status Header
        status_card = QFrame()
        status_card.setObjectName("statusCard")
        status_card_layout = QVBoxLayout(status_card)
        status_card_layout.setContentsMargins(6, 6, 6, 6)
        self.status_label = QLabel(
            "SYSTEM READY | Click grid to spawn robot or queue multi-point targets [Spacebar = Start/Pause]"
        )
        self.status_label.setObjectName("telemetryText")
        status_card_layout.addWidget(self.status_label)
        view_layout.addWidget(status_card)

        # Map Canvas
        self.map_widget = MapWidget()
        view_layout.addWidget(self.map_widget)
        main_layout.addWidget(view_container, stretch=3)

        # --- RIGHT PANEL: TELEMETRY & CONTROL ---
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        control_layout.setContentsMargins(4, 4, 8, 4)
        control_layout.setSpacing(6)

        # Cursor Tracker
        cursor_box = QGroupBox("CURSOR TRACKER")
        cur_layout = QVBoxLayout(cursor_box)
        cur_layout.setContentsMargins(6, 6, 6, 6)
        self.lbl_mouse_pos = QLabel("X: -- | Y: -- (cm)")
        self.lbl_mouse_pos.setObjectName("mouseCoordText")
        cur_layout.addWidget(self.lbl_mouse_pos)
        control_layout.addWidget(cursor_box)

        # Telemetry
        telemetry_box = QGroupBox("TURTLESIM TELEMETRY")
        tel_layout = QVBoxLayout(telemetry_box)
        tel_layout.setContentsMargins(6, 6, 6, 6)
        self.lbl_pos = QLabel("Position: (N/A, N/A)")
        self.lbl_head = QLabel("Heading: 0.0°")
        self.lbl_waypoints = QLabel("Queued Waypoints: 0")
        self.lbl_target = QLabel("Current Target: (N/A, N/A)")
        self.lbl_grid_res = QLabel(f"Grid Step: {settings.INITIAL_GRID_STEP_CM:.1f} cm")
        tel_layout.addWidget(self.lbl_pos)
        tel_layout.addWidget(self.lbl_head)
        tel_layout.addWidget(self.lbl_waypoints)
        tel_layout.addWidget(self.lbl_target)
        tel_layout.addWidget(self.lbl_grid_res)
        control_layout.addWidget(telemetry_box)

        # Teleop Controls
        rot_box = QGroupBox("TELEOP & ROTATION CONTROLS")
        rot_layout = QVBoxLayout(rot_box)
        rot_layout.setContentsMargins(6, 6, 6, 6)

        teleop_layout = QVBoxLayout()
        teleop_layout.setSpacing(4)
        self.btn_fwd = QPushButton("▲ Linear Move (+10cm)")
        self.btn_fwd.clicked.connect(lambda: (self.ensure_robot_spawned(), self.controller.teleop_move(10.0)))

        btn_turn_row = QHBoxLayout()
        btn_turn_row.setSpacing(4)
        self.btn_left = QPushButton("◄ Turn Left (+15°)")
        self.btn_right = QPushButton("Turn Right (-15°) ►")
        self.btn_left.clicked.connect(lambda: (self.ensure_robot_spawned(), self.controller.rotate_robot_delta(15.0)))
        self.btn_right.clicked.connect(lambda: (self.ensure_robot_spawned(), self.controller.rotate_robot_delta(-15.0)))
        btn_turn_row.addWidget(self.btn_left)
        btn_turn_row.addWidget(self.btn_right)

        self.btn_back = QPushButton("▼ Linear Back (-10cm)")
        self.btn_back.clicked.connect(lambda: (self.ensure_robot_spawned(), self.controller.teleop_move(-10.0)))

        teleop_layout.addWidget(self.btn_fwd)
        teleop_layout.addLayout(btn_turn_row)
        teleop_layout.addWidget(self.btn_back)
        rot_layout.addLayout(teleop_layout)

        custom_rot_layout = QHBoxLayout()
        custom_rot_layout.setSpacing(4)
        self.spin_angle = QDoubleSpinBox()
        self.spin_angle.setRange(-360.0, 360.0)
        self.spin_angle.setValue(45.0)
        self.spin_angle.setSuffix("°")

        self.btn_apply_spin = QPushButton("Rotate Angle")
        self.btn_apply_spin.clicked.connect(lambda: (self.ensure_robot_spawned(), self.controller.rotate_robot_delta(self.spin_angle.value())))
        custom_rot_layout.addWidget(QLabel("Angle:"))
        custom_rot_layout.addWidget(self.spin_angle)
        custom_rot_layout.addWidget(self.btn_apply_spin)
        rot_layout.addLayout(custom_rot_layout)

        control_layout.addWidget(rot_box)

        # Services & Actions
        ops_box = QGroupBox("TURTLESIM SERVICES & ACTIONS")
        ops_layout = QVBoxLayout(ops_box)
        ops_layout.setContentsMargins(6, 6, 6, 6)
        ops_layout.setSpacing(4)

        self.btn_orient = QPushButton("Set Robot Heading")
        self.btn_orient.clicked.connect(self.prompt_robot_orientation)
        self.btn_orient.setEnabled(True)
        ops_layout.addWidget(self.btn_orient)

        self.btn_start = QPushButton("Execute Mission Path [SPACE]")
        self.btn_start.setObjectName("primaryBtn")
        self.btn_start.clicked.connect(self.toggle_mission_execution)
        self.btn_start.setEnabled(False)
        ops_layout.addWidget(self.btn_start)

        self.btn_clear_waypoints = QPushButton("Clear Queued Waypoints")
        self.btn_clear_waypoints.clicked.connect(self.clear_waypoints)
        ops_layout.addWidget(self.btn_clear_waypoints)

        home_label = "🏠 Return All Robots Home" if self.home_callback else "🏠 Return Direct to Start"
        self.btn_return_home = QPushButton(home_label)
        self.btn_return_home.setObjectName("warningBtn")
        self.btn_return_home.clicked.connect(self.return_to_home)
        self.btn_return_home.setEnabled(False)
        ops_layout.addWidget(self.btn_return_home)

        backtrack_label = "↩ Backtrack All Robot Paths" if self.backtrack_callback else "↩ Backtrack via Path"
        self.btn_backtrack = QPushButton(backtrack_label)
        self.btn_backtrack.setStyleSheet("background-color: #61380b; color: #ffcc00; font-weight: bold;")
        self.btn_backtrack.clicked.connect(self.start_backtrack)
        self.btn_backtrack.setEnabled(False)
        ops_layout.addWidget(self.btn_backtrack)

        self.btn_clear_path = QPushButton("Clear Path Trails")
        self.btn_clear_path.clicked.connect(self.clear_trails)
        ops_layout.addWidget(self.btn_clear_path)

        self.btn_reset_view = QPushButton("Recenter Viewport")
        self.btn_reset_view.clicked.connect(self.map_widget.reset_view)
        ops_layout.addWidget(self.btn_reset_view)

        self.btn_reset = QPushButton("Reset World (Kill/Spawn)")
        self.btn_reset.setObjectName("dangerBtn")
        self.btn_reset.clicked.connect(self.reset_world)
        ops_layout.addWidget(self.btn_reset)

        control_layout.addWidget(ops_box)

        # Set NoFocus on all buttons so Arrow Keys are not stolen for focus navigation
        for btn in [self.btn_fwd, self.btn_back, self.btn_left, self.btn_right,
                    self.btn_apply_spin, self.btn_orient, self.btn_start,
                    self.btn_clear_waypoints, self.btn_return_home,
                    self.btn_backtrack, self.btn_clear_path, self.btn_reset_view, self.btn_reset]:
            btn.setFocusPolicy(Qt.NoFocus)

        # Controls Guide
        guide_box = QGroupBox("CONTROLS GUIDE")
        guide_layout = QVBoxLayout(guide_box)
        guide_layout.setContentsMargins(6, 6, 6, 6)
        guide_layout.setSpacing(2)
        guide_layout.addWidget(QLabel("• Spacebar: Start / Pause / Resume Path"))
        guide_layout.addWidget(QLabel("• Left Click: Spawn Robot or Add Waypoints"))
        guide_layout.addWidget(QLabel("• WASD / Arrow Keys: Drive Teleop"))
        guide_layout.addWidget(QLabel("• Scroll Wheel: Smooth Zoom"))
        guide_layout.addWidget(QLabel("• Scroll Click + Drag: Pan View"))
        guide_layout.addWidget(QLabel("• Orange Marker: World Origin (0,0)"))
        control_layout.addWidget(guide_box)

        control_layout.addStretch()
        scroll_area.setWidget(control_panel)
        main_layout.addWidget(scroll_area, stretch=1)

        self.set_teleop_controls_enabled(True)

    def ensure_robot_spawned(self):
        """Ensure a robot pose exists before teleoperating."""
        if self.state.pose is None:
            self.state.set_pose(20.0, 20.0, 0.0, set_initial_if_unset=True)
            self.map_widget.set_initial_pos((20.0, 20.0))
            self.status_label.setText("Robot auto-spawned at (20.0, 20.0) | Heading 0.0°")

    def _connect_signals(self):
        # Map signals
        self.map_widget.grid_clicked.connect(self.on_grid_clicked)
        self.map_widget.mouse_moved.connect(self.on_mouse_moved)
        self.map_widget.grid_step_changed.connect(
            lambda step: self.lbl_grid_res.setText(f"Grid Step: {step:.2f} cm")
        )

        # State signals
        self.state.pose_changed.connect(self._on_pose_changed)
        self.state.flags_changed.connect(self._update_ui_state)
        self.state.safety_halt_triggered.connect(self._on_safety_halt)

        # Controller signals
        self.controller.status_message_updated.connect(self.status_label.setText)
        self.controller.motion_completed.connect(self._on_motion_completed)

    def _on_pose_changed(self, x: float, y: float, heading: float):
        self.map_widget.set_robot_pose(x, y, heading)
        self.lbl_pos.setText(f"Position: ({x:.1f}, {y:.1f}) cm")
        self.lbl_head.setText(f"Heading: {heading:.1f}°")

    def _update_ui_state(self):
        has_pos = (self.state.pose is not None)
        is_executing = self.state.is_executing

        # Keep teleop direction buttons enabled
        self.set_teleop_controls_enabled(True)
        self.btn_orient.setEnabled(True)

        if has_pos and self.state.initial_pose and not is_executing:
            self.btn_return_home.setEnabled(True)
            self.btn_backtrack.setEnabled(len(self.state.path_history) > 0)
        else:
            self.btn_return_home.setEnabled(False)
            self.btn_backtrack.setEnabled(False)

        if self.waypoint_queue or self.backtrack_waypoints:
            self.btn_start.setEnabled(True)
        elif not is_executing:
            self.btn_start.setEnabled(False)

    def set_teleop_controls_enabled(self, enabled: bool):
        self.btn_fwd.setEnabled(enabled)
        self.btn_back.setEnabled(enabled)
        self.btn_left.setEnabled(enabled)
        self.btn_right.setEnabled(enabled)
        self.btn_apply_spin.setEnabled(enabled)
        self.spin_angle.setEnabled(enabled)

    def on_mouse_moved(self, x: float, y: float):
        self.lbl_mouse_pos.setText(f"X: {x:.2f} | Y: {y:.2f} cm")

    def on_grid_clicked(self, x_snap: float, y_snap: float):
        if self.state.pose is None:
            self.state.set_pose(x_snap, y_snap, 0.0, set_initial_if_unset=True)
            self.map_widget.set_initial_pos((x_snap, y_snap))
            self.prompt_robot_orientation()
        else:
            self.waypoint_queue.append((x_snap, y_snap))
            self.lbl_waypoints.setText(f"Queued Waypoints: {len(self.waypoint_queue)}")
            self.map_widget.set_waypoint_queue(self.waypoint_queue)

            if not self.state.is_executing:
                self.btn_start.setEnabled(True)
                self.btn_start.setText("Execute Mission Path [SPACE]")
                self.status_label.setText(
                    f"WAYPOINT QUEUED: P{len(self.waypoint_queue)} at ({x_snap:.1f}, {y_snap:.1f}) | Press SPACE to run"
                )
            else:
                self.status_label.setText(
                    f"WAYPOINT ADDED MID-FLIGHT: P{len(self.waypoint_queue)} at ({x_snap:.1f}, {y_snap:.1f})"
                )

    def prompt_robot_orientation(self):
        if self.state.pose is None or self.state.is_executing:
            return

        angle, ok = QInputDialog.getDouble(
            self,
            "Set Initial Heading",
            "Enter heading angle (0° = East, 90° = North, 180° = West, -90° = South):",
            self.state.heading, -360.0, 360.0, 1
        )
        if ok:
            norm_angle = normalize_angle_deg(angle)
            self.state.set_heading(norm_angle)
            if self.state.initial_pose:
                self.state.initial_pose.heading = norm_angle
            self.status_label.setText(
                f"ROBOT SPAWNED | Pos: ({self.state.x:.1f}, {self.state.y:.1f}) cm | Heading: {norm_angle:.1f}°"
            )

    def toggle_mission_execution(self):
        if self.state.pose is None or (not self.waypoint_queue and not self.state.is_executing and not self.state.is_backtracking):
            return

        if not self.state.is_executing:
            # START MISSION
            self.state.set_paused(False)
            if self.state.is_backtracking:
                self.execute_next_backtrack_point()
            else:
                self.execute_next_waypoint()
        else:
            # PAUSE OR RESUME MISSION
            if not self.state.is_paused:
                self.state.set_paused(True)
                self.controller.anim_timer.stop()
                self.btn_start.setText("Resume Mission Path [SPACE]")
                self.btn_start.setStyleSheet("background-color: #8a5300; color: white;")
                self.status_label.setText("PAUSED | Mission paused by user. Press Spacebar to Resume.")
            else:
                self.state.set_paused(False)
                self.controller.anim_timer.start()
                self.btn_start.setText("Pause Mission Path [SPACE]")
                self.btn_start.setStyleSheet("background-color: #b52a2a; color: white;")
                target_str = f"({self.current_target[0]:.1f}, {self.current_target[1]:.1f})" if self.current_target else "N/A"
                self.status_label.setText(f"RESUMED | Heading to target {target_str}")

    def execute_next_waypoint(self):
        if not self.waypoint_queue:
            self.finish_navigation()
            return

        self.current_target = self.waypoint_queue.pop(0)
        self.state.set_current_target(self.current_target)
        self.map_widget.set_current_target(self.current_target)
        self.lbl_waypoints.setText(f"Queued Waypoints: {len(self.waypoint_queue)}")
        self.map_widget.set_waypoint_queue(self.waypoint_queue)

        dx = self.current_target[0] - self.state.x
        dy = self.current_target[1] - self.state.y

        dist = math.hypot(dx, dy)
        calc_target_angle = math.degrees(math.atan2(dy, dx))
        turn_angle_deg = normalize_angle_deg(calc_target_angle - self.state.heading)

        self.state.set_executing(True)
        self.btn_start.setEnabled(True)
        self.btn_start.setText("Pause Mission Path [SPACE]")
        self.btn_start.setStyleSheet("background-color: #b52a2a; color: white;")

        if abs(turn_angle_deg) > 1.0:
            self.status_label.setText(
                f"EXECUTING TURN | Delta: {turn_angle_deg:.1f}° | Target: ({self.current_target[0]:.1f}, {self.current_target[1]:.1f})"
            )
            self.controller.execute_turn(turn_angle_deg)
        else:
            self.start_distance_move()

    def start_distance_move(self):
        if not self.current_target:
            self.on_waypoint_reached()
            return

        dx = self.current_target[0] - self.state.x
        dy = self.current_target[1] - self.state.y
        dist = math.hypot(dx, dy)

        if dist > 0.1:
            self.status_label.setText(
                f"EXECUTING LINEAR MOVE | Distance: {dist:.1f} cm | Target: ({self.current_target[0]:.1f}, {self.current_target[1]:.1f})"
            )
            self.controller.execute_move(dist, self.current_target, direction=1.0)
        else:
            self.on_waypoint_reached()

    def on_waypoint_reached(self):
        self.current_target = None
        self.state.set_current_target(None)
        self.map_widget.set_current_target(None)
        if self.waypoint_queue:
            self.execute_next_waypoint()
        else:
            self.finish_navigation()

    def finish_navigation(self):
        self.state.set_executing(False)
        self.state.set_paused(False)
        self.state.set_backtracking(False)
        self.current_target = None
        self.state.set_current_target(None)
        self.map_widget.set_current_target(None)

        self.btn_start.setText("Execute Mission Path [SPACE]")
        self.btn_start.setStyleSheet("background-color: #1e5c2b; color: white;")
        self.btn_start.setEnabled(False)
        self.status_label.setText("MISSION COMPLETED | All queued waypoints reached successfully!")
        self._update_ui_state()

    def return_to_home(self):
        if self.home_callback:
            self.home_callback()
            self.status_label.setText("RETURN HOME REQUESTED | All configured robots are returning.")
            return

        if self.state.pose is None or self.state.initial_pose is None or self.state.is_executing:
            return

        self.is_returning_home = True
        self.state.set_returning_home(True)
        self.current_target = self.state.initial_pose.as_tuple
        self.home_phase = 'TURN_TO_HOME'

        dx = self.current_target[0] - self.state.x
        dy = self.current_target[1] - self.state.y
        dist_to_home = math.hypot(dx, dy)

        if dist_to_home < 0.5:
            self.home_phase = 'ALIGN_HOME'
            self.align_to_home_orientation()
        else:
            calc_target_angle = math.degrees(math.atan2(dy, dx))
            turn_angle_deg = normalize_angle_deg(calc_target_angle - self.state.heading)
            if abs(turn_angle_deg) > 1.0:
                self.status_label.setText(f"RETURNING HOME: Orienting toward Start Point ({turn_angle_deg:+.1f}°)")
                self.controller.execute_turn(turn_angle_deg)
            else:
                self.start_home_move()

    def start_home_move(self):
        self.home_phase = 'MOVE_TO_HOME'
        dx = self.state.initial_pose.x - self.state.x
        dy = self.state.initial_pose.y - self.state.y
        dist = math.hypot(dx, dy)

        if dist > 0.1:
            self.status_label.setText(f"RETURNING HOME: Traversing back to Start ({dist:.1f} cm)")
            self.controller.execute_move(dist, self.state.initial_pose.as_tuple, direction=1.0)
        else:
            self.align_to_home_orientation()

    def align_to_home_orientation(self):
        self.home_phase = 'ALIGN_HOME'
        turn_angle_deg = normalize_angle_deg(self.state.initial_pose.heading - self.state.heading)
        if abs(turn_angle_deg) > 1.0:
            self.status_label.setText(f"RETURNING HOME: Re-aligning to Start Heading ({turn_angle_deg:+.1f}°)")
            self.controller.execute_turn(turn_angle_deg)
        else:
            self.finish_home_routine()

    def finish_home_routine(self):
        self.is_returning_home = False
        self.state.set_returning_home(False)
        self.state.set_executing(False)
        self.state.set_backtracking(False)
        self.current_target = None
        self.home_phase = None

        self.status_label.setText("SUCCESS: Robot returned to Start Point with Original Heading!")
        self.btn_start.setText("Execute Mission Path [SPACE]")
        self.btn_start.setStyleSheet("background-color: #1e5c2b; color: white;")
        if self.waypoint_queue:
            self.btn_start.setEnabled(True)
        self._update_ui_state()

    def start_backtrack(self):
        if self.backtrack_callback:
            self.backtrack_callback()
            self.status_label.setText("BACKTRACK REQUESTED | All robots are following their recorded paths.")
            return

        if self.state.pose is None or self.state.is_executing or not self.state.path_history:
            return

        raw_nodes = []
        for stroke in self.state.path_history:
            for pt in stroke:
                if not raw_nodes or math.hypot(pt[0] - raw_nodes[-1][0], pt[1] - raw_nodes[-1][1]) > 0.5:
                    raw_nodes.append(pt)

        if not raw_nodes:
            return

        self.backtrack_waypoints = list(reversed(raw_nodes))
        if self.backtrack_waypoints and math.hypot(self.state.x - self.backtrack_waypoints[0][0], self.state.y - self.backtrack_waypoints[0][1]) < 0.5:
            self.backtrack_waypoints.pop(0)

        self.state.set_backtracking(True)
        self.state.set_executing(True)
        self.btn_start.setEnabled(True)
        self.btn_start.setText("Pause Backtrack [SPACE]")
        self.btn_start.setStyleSheet("background-color: #b52a2a; color: white;")
        self.execute_next_backtrack_point()

    def execute_next_backtrack_point(self):
        if not self.backtrack_waypoints:
            self.align_to_home_orientation()
            return

        self.current_target = self.backtrack_waypoints.pop(0)
        self.state.set_current_target(self.current_target)
        self.map_widget.set_current_target(self.current_target)

        dx = self.current_target[0] - self.state.x
        dy = self.current_target[1] - self.state.y
        calc_target_angle = math.degrees(math.atan2(dy, dx))
        turn_angle_deg = normalize_angle_deg(calc_target_angle - self.state.heading)

        if abs(turn_angle_deg) > 1.0:
            self.status_label.setText(
                f"BACKTRACKING: Turning ({turn_angle_deg:.1f}°) towards node ({self.current_target[0]:.1f}, {self.current_target[1]:.1f})"
            )
            self.controller.execute_turn(turn_angle_deg)
        else:
            self.start_distance_move()

    def _on_motion_completed(self, mode: str):
        if mode == "TURN":
            if self.is_returning_home:
                if self.home_phase == 'TURN_TO_HOME':
                    self.start_home_move()
                elif self.home_phase == 'ALIGN_HOME':
                    self.finish_home_routine()
            elif self.current_target is not None:
                self.start_distance_move()
        elif mode == "MOVE":
            if self.is_returning_home and self.home_phase == 'MOVE_TO_HOME':
                self.align_to_home_orientation()
            elif self.state.is_backtracking:
                self.execute_next_backtrack_point()
            elif self.current_target is not None:
                self.on_waypoint_reached()

    def _on_safety_halt(self):
        self.is_returning_home = False
        self.home_phase = None
        self.current_target = None
        self.btn_start.setText("Execute Mission Path [SPACE]")
        self.btn_start.setStyleSheet("background-color: #1e5c2b; color: white;")
        self.status_label.setText("SAFETY HALT DETECTED | Hardware sensors interrupted movement")
        self._update_ui_state()

    def clear_waypoints(self):
        self.waypoint_queue.clear()
        self.map_widget.set_waypoint_queue([])
        self.lbl_waypoints.setText("Queued Waypoints: 0")
        if not self.state.is_executing:
            self.btn_start.setEnabled(False)
            self.btn_start.setText("Execute Mission Path [SPACE]")
            self.btn_start.setStyleSheet("background-color: #1e5c2b; color: white;")

    def clear_trails(self):
        self.state.clear_trails()
        self.map_widget.clear_trails()

    def reset_world(self):
        self.controller.anim_timer.stop()
        self.clear_waypoints()
        self.backtrack_waypoints.clear()
        self.is_returning_home = False
        self.home_phase = None
        self.state.reset_world()
        self.map_widget.set_initial_pos(None)
        self.map_widget.set_current_target(None)
        self.lbl_pos.setText("Position: (N/A, N/A)")
        self.lbl_head.setText("Heading: 0.0°")
        self.status_label.setText("WORLD RESET | Click grid to spawn robot")
        self.map_widget.reset_view()
        self._update_ui_state()

    def keyPressEvent(self, event):
        # Don't trigger movement if focused on an input widget
        focused = self.focusWidget()
        if isinstance(focused, (QLineEdit, QDoubleSpinBox, QSpinBox)):
            super().keyPressEvent(event)
            return

        key = event.key()
        if key == Qt.Key_Space:
            self.toggle_mission_execution()
            return

        if self.state.pose is None or self.state.is_executing:
            super().keyPressEvent(event)
            return

        if key in (Qt.Key_W, Qt.Key_Up):
            self.controller.teleop_move(10.0)
        elif key in (Qt.Key_S, Qt.Key_Down):
            self.controller.teleop_move(-10.0)
        elif key in (Qt.Key_A, Qt.Key_Left):
            self.controller.rotate_robot_delta(15.0)
        elif key in (Qt.Key_D, Qt.Key_Right):
            self.controller.rotate_robot_delta(-15.0)
        else:
            super().keyPressEvent(event)
