"""Dedicated Mission Path Planning page with Teach Mode, Boundary/Obstacle Editor, Validation, and Execution."""
import math
import logging
from typing import Optional, List, Tuple
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QFrame, QGroupBox, QDoubleSpinBox, QScrollArea, QInputDialog,
    QMessageBox, QFileDialog, QListWidget, QListWidgetItem,
    QProgressBar, QCheckBox, QDialog, QDialogButtonBox, QFormLayout, QLineEdit, QComboBox
)
from PyQt5.QtCore import Qt, QTimer

import config.settings as settings
from ui.map_widget import MapWidget
from models.mission import Mission
from models.obstacle import Boundary, RestrictedArea
from models.pose import Pose, normalize_angle_deg
from navigation.mission_manager import MissionManager
from robot.robot_controller import RobotController
from robot.robot_state import RobotState

logger = logging.getLogger(__name__)


class AddRestrictedAreaDialog(QDialog):
    """Dialog to define a new restricted no-go area."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Add Restricted / No-Go Area")
        self.setMinimumWidth(320)

        layout = QVBoxLayout(self)
        form = QFormLayout()

        self.name_edit = QLineEdit("Obstacle_1")
        form.addRow("Area Name:", self.name_edit)

        self.min_x = QDoubleSpinBox()
        self.min_x.setRange(-500.0, 500.0)
        self.min_x.setValue(40.0)
        self.min_x.setSuffix(" cm")
        form.addRow("Min X:", self.min_x)

        self.min_y = QDoubleSpinBox()
        self.min_y.setRange(-500.0, 500.0)
        self.min_y.setValue(40.0)
        self.min_y.setSuffix(" cm")
        form.addRow("Min Y:", self.min_y)

        self.width_spin = QDoubleSpinBox()
        self.width_spin.setRange(1.0, 500.0)
        self.width_spin.setValue(30.0)
        self.width_spin.setSuffix(" cm")
        form.addRow("Width (X):", self.width_spin)

        self.height_spin = QDoubleSpinBox()
        self.height_spin.setRange(1.0, 500.0)
        self.height_spin.setValue(30.0)
        self.height_spin.setSuffix(" cm")
        form.addRow("Height (Y):", self.height_spin)

        layout.addLayout(form)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def get_restricted_area(self) -> RestrictedArea:
        name = self.name_edit.text().strip() or "restricted_area"
        x1 = self.min_x.value()
        y1 = self.min_y.value()
        w = self.width_spin.value()
        h = self.height_spin.value()

        # Rectangular polygon vertices in counter-clockwise order
        polygon = [
            (x1, y1),
            (x1 + w, y1),
            (x1 + w, y1 + h),
            (x1, y1 + h)
        ]
        area_id = name.lower().replace(" ", "_")
        return RestrictedArea(id=area_id, name=name, polygon=polygon)


class MissionPlannerPage(QWidget):
    """Mission Path Planning Page supporting manual recording, boundaries, obstacles, validation, and execution."""

    def __init__(
        self,
        manager: MissionManager,
        controller: RobotController,
        state: RobotState,
        parent=None
    ):
        super().__init__(parent)
        self.manager = manager
        self.controller = controller
        self.state = state

        # Preview animation state
        self.preview_timer = QTimer()
        self.preview_timer.setInterval(40)  # 25 FPS
        self.preview_timer.timeout.connect(self._update_preview_step)
        self.preview_points: List[Tuple[float, float, float]] = []
        self.preview_index: int = 0

        self.init_ui()
        self._connect_signals()

    def init_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(6)

        # --- LEFT PANEL: VIEWPORT & STATUS HEADER ---
        view_container = QWidget()
        view_layout = QVBoxLayout(view_container)
        view_layout.setContentsMargins(0, 0, 0, 0)
        view_layout.setSpacing(4)

        # Status Header Card
        status_card = QFrame()
        status_card.setObjectName("statusCard")
        status_card_layout = QVBoxLayout(status_card)
        status_card_layout.setContentsMargins(6, 6, 6, 6)

        self.status_label = QLabel(
            "MISSION PLANNER READY | Create/load a mission, record paths in Teach Mode, or validate obstacles"
        )
        self.status_label.setObjectName("telemetryText")
        status_card_layout.addWidget(self.status_label)

        # Execution Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setFixedHeight(12)
        status_card_layout.addWidget(self.progress_bar)

        view_layout.addWidget(status_card)

        # Map Canvas
        self.map_widget = MapWidget()
        view_layout.addWidget(self.map_widget)
        main_layout.addWidget(view_container, stretch=3)

        # --- RIGHT PANEL: MISSION CONTROL SIDEBAR ---
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        control_layout.setContentsMargins(4, 4, 8, 4)
        control_layout.setSpacing(6)

        # 1. Mission File Management
        file_box = QGroupBox("MISSION MANAGEMENT")
        file_layout = QVBoxLayout(file_box)
        file_layout.setSpacing(4)

        row_file_btns = QHBoxLayout()
        self.btn_new_mission = QPushButton("New")
        self.btn_new_mission.clicked.connect(self.new_mission)
        self.btn_load_mission = QPushButton("Load")
        self.btn_load_mission.clicked.connect(self.load_mission)
        self.btn_save_mission = QPushButton("Save")
        self.btn_save_mission.clicked.connect(self.save_mission)
        self.btn_refresh = QPushButton("Refresh / Reset")
        self.btn_refresh.setObjectName("warningBtn")
        self.btn_refresh.clicked.connect(self.reset_software)
        row_file_btns.addWidget(self.btn_new_mission)
        row_file_btns.addWidget(self.btn_load_mission)
        row_file_btns.addWidget(self.btn_save_mission)
        row_file_btns.addWidget(self.btn_refresh)
        file_layout.addLayout(row_file_btns)

        self.lbl_mission_info = QLabel("Active: new_mission (0 points)")
        self.lbl_mission_info.setStyleSheet("color: #00ffaa; font-weight: bold;")
        file_layout.addWidget(self.lbl_mission_info)
        control_layout.addWidget(file_box)

        # 1b. Robot Start Pose
        pose_box = QGroupBox("ROBOT START POSE")
        pose_layout = QVBoxLayout(pose_box)
        pose_layout.setSpacing(4)

        pose_row1 = QHBoxLayout()
        pose_row1.addWidget(QLabel("X:"))
        self.spin_start_x = QDoubleSpinBox()
        self.spin_start_x.setRange(-500.0, 500.0)
        self.spin_start_x.setValue(20.0)
        self.spin_start_x.setSuffix(" cm")
        self.spin_start_x.setDecimals(1)
        pose_row1.addWidget(self.spin_start_x)
        pose_row1.addWidget(QLabel("Y:"))
        self.spin_start_y = QDoubleSpinBox()
        self.spin_start_y.setRange(-500.0, 500.0)
        self.spin_start_y.setValue(20.0)
        self.spin_start_y.setSuffix(" cm")
        self.spin_start_y.setDecimals(1)
        pose_row1.addWidget(self.spin_start_y)
        pose_layout.addLayout(pose_row1)

        pose_row2 = QHBoxLayout()
        pose_row2.addWidget(QLabel("Heading:"))
        self.spin_start_heading = QDoubleSpinBox()
        self.spin_start_heading.setRange(-180.0, 180.0)
        self.spin_start_heading.setValue(0.0)
        self.spin_start_heading.setSuffix(" °")
        self.spin_start_heading.setDecimals(1)
        self.spin_start_heading.setToolTip(
            "Robot facing direction.\n"
            "0° = East (right)  |  90° = North (up)\n"
            "180°/-180° = West  |  -90° = South (down)"
        )
        pose_row2.addWidget(self.spin_start_heading)

        self.btn_set_pose = QPushButton("⊕ Set Pose")
        self.btn_set_pose.setObjectName("primaryBtn")
        self.btn_set_pose.clicked.connect(self.apply_start_pose)
        pose_row2.addWidget(self.btn_set_pose)
        pose_layout.addLayout(pose_row2)

        self.lbl_pose_hint = QLabel("Tip: Click map to place robot, then adjust heading here.")
        self.lbl_pose_hint.setStyleSheet("color: #666; font-size: 10px;")
        self.lbl_pose_hint.setWordWrap(True)
        pose_layout.addWidget(self.lbl_pose_hint)
        control_layout.addWidget(pose_box)

        # 1c. Multi-robot destination assignment
        destination_box = QGroupBox("MULTI-ROBOT WAYPOINTS")
        destination_layout = QVBoxLayout(destination_box)
        destination_layout.setSpacing(4)
        destination_layout.addWidget(QLabel("Robot (or automatic nearest):"))
        self.combo_destination_robot = QComboBox()
        self.combo_destination_robot.addItem("AUTO / NEAREST", None)
        for robot_id in self.manager.roster.robots:
            self.combo_destination_robot.addItem(robot_id.robot_id, robot_id.robot_id)
        destination_layout.addWidget(self.combo_destination_robot)

        destination_row = QHBoxLayout()
        self.spin_destination_x = QDoubleSpinBox()
        self.spin_destination_x.setRange(-500.0, 500.0)
        self.spin_destination_x.setSuffix(" cm")
        self.spin_destination_x.setDecimals(1)
        self.spin_destination_y = QDoubleSpinBox()
        self.spin_destination_y.setRange(-500.0, 500.0)
        self.spin_destination_y.setSuffix(" cm")
        self.spin_destination_y.setDecimals(1)
        destination_row.addWidget(QLabel("X:"))
        destination_row.addWidget(self.spin_destination_x)
        destination_row.addWidget(QLabel("Y:"))
        destination_row.addWidget(self.spin_destination_y)
        destination_layout.addLayout(destination_row)

        self.btn_add_destination = QPushButton("Add Waypoint")
        self.btn_add_destination.clicked.connect(self.add_destination)
        destination_layout.addWidget(self.btn_add_destination)
        self.list_destinations = QListWidget()
        self.list_destinations.setFixedHeight(75)
        destination_layout.addWidget(self.list_destinations)
        self.btn_clear_destinations = QPushButton("Clear Destinations")
        self.btn_clear_destinations.clicked.connect(self.clear_destinations)
        destination_layout.addWidget(self.btn_clear_destinations)
        control_layout.addWidget(destination_box)

        # 2. Teach / Record Mode
        record_box = QGroupBox("TEACH / RECORD PATH")
        rec_layout = QVBoxLayout(record_box)
        rec_layout.setSpacing(4)

        row_rec_btns = QHBoxLayout()
        self.btn_start_record = QPushButton("● Start Recording")
        self.btn_start_record.setObjectName("dangerBtn")
        self.btn_start_record.clicked.connect(self.toggle_recording)
        self.btn_clear_path = QPushButton("Clear Path")
        self.btn_clear_path.clicked.connect(self.clear_mission_path)
        row_rec_btns.addWidget(self.btn_start_record)
        row_rec_btns.addWidget(self.btn_clear_path)
        rec_layout.addLayout(row_rec_btns)

        self.lbl_record_status = QLabel("Recording: INACTIVE")
        self.lbl_record_status.setStyleSheet("color: #888888; font-weight: bold;")
        rec_layout.addWidget(self.lbl_record_status)

        # On-screen D-Pad for manual drive during teach mode
        dpad_box = QGroupBox("MANUAL DRIVING PAD (TEACH MODE)")
        dpad_layout = QVBoxLayout(dpad_box)
        dpad_layout.setSpacing(3)

        btn_fwd = QPushButton("▲ FORWARD")
        btn_fwd.clicked.connect(lambda: self.drive_manual_step(10.0, 0.0))

        mid_row = QHBoxLayout()
        btn_left = QPushButton("◄ LEFT")
        btn_left.clicked.connect(lambda: self.drive_manual_step(0.0, 15.0))
        btn_stop = QPushButton("■ STOP")
        btn_stop.clicked.connect(self.controller.stop)
        btn_right = QPushButton("RIGHT ►")
        btn_right.clicked.connect(lambda: self.drive_manual_step(0.0, -15.0))
        mid_row.addWidget(btn_left)
        mid_row.addWidget(btn_stop)
        mid_row.addWidget(btn_right)

        btn_back = QPushButton("▼ BACKWARD")
        btn_back.clicked.connect(lambda: self.drive_manual_step(-10.0, 0.0))

        dpad_layout.addWidget(btn_fwd)
        dpad_layout.addLayout(mid_row)
        dpad_layout.addWidget(btn_back)
        rec_layout.addWidget(dpad_box)

        control_layout.addWidget(record_box)

        # 3. Environment & Restricted Areas
        env_box = QGroupBox("RESTRICTED / NO-GO AREAS")
        env_layout = QVBoxLayout(env_box)
        env_layout.setSpacing(4)

        self.list_restricted = QListWidget()
        self.list_restricted.setFixedHeight(85)
        env_layout.addWidget(self.list_restricted)

        row_env_btns = QHBoxLayout()
        self.btn_add_area = QPushButton("+ Add Area")
        self.btn_add_area.clicked.connect(self.add_restricted_area)
        self.btn_remove_area = QPushButton("- Remove")
        self.btn_remove_area.clicked.connect(self.remove_restricted_area)
        row_env_btns.addWidget(self.btn_add_area)
        row_env_btns.addWidget(self.btn_remove_area)
        env_layout.addLayout(row_env_btns)
        control_layout.addWidget(env_box)

        # 4. Robot Footprint & Safety Margin
        footprint_box = QGroupBox("ROBOT FOOTPRINT & SAFETY")
        fp_layout = QVBoxLayout(footprint_box)
        fp_layout.setSpacing(2)

        self.lbl_fp_specs = QLabel(
            f"Chassis: {settings.ROBOT_LENGTH_CM:.1f} × {settings.ROBOT_WIDTH_CM:.1f} cm | "
            f"Margin: {settings.ROBOT_SAFETY_MARGIN_CM:.1f} cm"
        )
        fp_layout.addWidget(self.lbl_fp_specs)

        self.chk_show_footprint = QCheckBox("Show Footprint + Safety Margin")
        self.chk_show_footprint.setChecked(True)
        self.chk_show_footprint.toggled.connect(self._toggle_footprint_margin)
        fp_layout.addWidget(self.chk_show_footprint)
        control_layout.addWidget(footprint_box)

        # 5. Mission Validation
        val_box = QGroupBox("PATH VALIDATION")
        val_layout = QVBoxLayout(val_box)
        val_layout.setSpacing(4)

        self.btn_validate = QPushButton("Validate Mission Path")
        self.btn_validate.clicked.connect(self.validate_mission)
        val_layout.addWidget(self.btn_validate)

        self.lbl_val_status = QLabel("Validation: Not Checked")
        self.lbl_val_status.setWordWrap(True)
        self.lbl_val_status.setStyleSheet("color: #ffaa00; font-weight: bold;")
        val_layout.addWidget(self.lbl_val_status)
        control_layout.addWidget(val_box)

        # 6. Mission Execution & Preview
        exec_box = QGroupBox("EXECUTION & PLAYBACK")
        exec_layout = QVBoxLayout(exec_box)
        exec_layout.setSpacing(4)

        self.btn_preview = QPushButton("▶ Preview Mission (Dry Run)")
        self.btn_preview.clicked.connect(self.toggle_preview)
        exec_layout.addWidget(self.btn_preview)

        self.btn_execute = QPushButton("⚡ Execute Mission on Real Robot")
        self.btn_execute.setObjectName("primaryBtn")
        self.btn_execute.clicked.connect(self.execute_mission)
        exec_layout.addWidget(self.btn_execute)

        self.chk_auto_return_home = QCheckBox("Auto-return robots home after destination")
        self.chk_auto_return_home.setChecked(True)
        self.chk_auto_return_home.toggled.connect(self.manager.set_auto_return_home)
        exec_layout.addWidget(self.chk_auto_return_home)

        self.btn_return_all_home = QPushButton("Return All Robots Home")
        self.btn_return_all_home.setObjectName("warningBtn")
        self.btn_return_all_home.clicked.connect(self.manager.return_all_robots_home)
        exec_layout.addWidget(self.btn_return_all_home)

        row_exec_controls = QHBoxLayout()
        self.btn_pause = QPushButton("Pause")
        self.btn_pause.setObjectName("warningBtn")
        self.btn_pause.clicked.connect(self.manager.pause_mission)
        self.btn_resume = QPushButton("Resume")
        self.btn_resume.clicked.connect(self.manager.resume_mission)
        self.btn_stop = QPushButton("Stop")
        self.btn_stop.setObjectName("dangerBtn")
        self.btn_stop.clicked.connect(self.manager.stop_mission)
        row_exec_controls.addWidget(self.btn_pause)
        row_exec_controls.addWidget(self.btn_resume)
        row_exec_controls.addWidget(self.btn_stop)
        exec_layout.addLayout(row_exec_controls)

        control_layout.addWidget(exec_box)

        # Set NoFocus on all buttons so Arrow Keys are not stolen for focus navigation
        for btn in [self.btn_new_mission, self.btn_load_mission, self.btn_save_mission,
                    self.btn_refresh,
                    self.btn_set_pose,
                    self.btn_add_destination, self.btn_clear_destinations,
                    self.btn_start_record, self.btn_clear_path, btn_fwd, btn_left,
                    btn_stop, btn_right, btn_back, self.btn_add_area, self.btn_remove_area,
                    self.btn_validate, self.btn_preview, self.btn_execute, self.btn_pause,
                    self.btn_resume, self.btn_stop, self.btn_return_all_home]:
            btn.setFocusPolicy(Qt.NoFocus)

        control_layout.addStretch()
        scroll_area.setWidget(control_panel)
        main_layout.addWidget(scroll_area, stretch=1)

        self._refresh_restricted_list()

    def _connect_signals(self):
        # Map signals
        self.map_widget.grid_clicked.connect(self.on_grid_clicked)
        self.map_widget.mouse_moved.connect(self._on_mouse_moved)

        # State signals
        self.state.pose_changed.connect(self._on_pose_changed)
        self.state.safety_halt_triggered.connect(self._on_safety_halt)

        # Manager signals
        self.manager.mission_loaded.connect(self._on_mission_loaded)
        self.manager.validation_completed.connect(self._on_validation_completed)
        self.manager.teach_mode_changed.connect(self._on_teach_mode_changed)
        self.manager.status_message.connect(self.status_label.setText)

        # Executor signals
        self.manager.executor.step_started.connect(self._on_executor_step)
        self.manager.executor.progress_updated.connect(
            lambda p: self.progress_bar.setValue(int(p * 100))
        )
        self.manager.executor.completed.connect(self._on_executor_completed)
        self.manager.executor.safety_halted.connect(self._on_safety_halt)

    def _on_pose_changed(self, x: float, y: float, heading: float):
        self.map_widget.set_robot_pose(x, y, heading)
        if self.manager.is_recording:
            # Continuously feed live positions into path recorder during teach mode
            self.manager.record_drive_step(x, y, heading, action="MOVE")

    def _on_mouse_moved(self, x: float, y: float):
        pass

    def on_grid_clicked(self, x_snap: float, y_snap: float):
        """Set robot initial pose if not placed, or add a target waypoint."""
        heading = self.spin_start_heading.value()
        if self.state.pose is None:
            self.state.set_pose(x_snap, y_snap, heading, set_initial_if_unset=True)
            self.map_widget.set_initial_pos((x_snap, y_snap))
            self.manager.active_mission.start_pose = Pose(x_snap, y_snap, heading)
            # Sync spinboxes to match clicked position
            self.spin_start_x.setValue(x_snap)
            self.spin_start_y.setValue(y_snap)
            self.status_label.setText(f"Robot spawned at ({x_snap:.1f}, {y_snap:.1f}) | Heading {heading:.1f}°")
        elif not self.manager.is_recording:
            if self.manager.is_multi_robot:
                self.spin_destination_x.setValue(x_snap)
                self.spin_destination_y.setValue(y_snap)
                self.add_destination()
                return
            # Append manual waypoint to mission path
            dx = x_snap - self.state.x
            dy = y_snap - self.state.y
            dist = math.hypot(dx, dy)
            target_heading = normalize_angle_deg(math.degrees(math.atan2(dy, dx)))

            from models.path import PathPoint
            pt = PathPoint(
                x=x_snap,
                y=y_snap,
                heading=target_heading,
                action="WAYPOINT",
                distance=dist
            )
            self.manager.active_mission.path.append(pt)
            self._update_mission_path_display()

    def add_destination(self):
        """Add the selected map coordinates to the multi-robot mission."""
        robot_id = self.combo_destination_robot.currentData()
        destination = self.manager.add_destination(
            self.spin_destination_x.value(), self.spin_destination_y.value(), robot_id
        )
        self._refresh_destinations()
        target = robot_id or "nearest available robot"
        self.status_label.setText(
            f"Added {destination.destination_id} at ({destination.x:.1f}, {destination.y:.1f}) for {target}."
        )

    def clear_destinations(self):
        self.manager.active_mission.destinations.clear()
        self._refresh_destinations()
        self.status_label.setText("Multi-robot destinations cleared.")

    def reset_software(self):
        """Restore robot poses, mission data, and visible trails to startup state."""
        if self.preview_timer.isActive():
            self.preview_timer.stop()
        self.map_widget.set_ghost_pose(None)
        self.chk_auto_return_home.setChecked(True)
        self.manager.reset_software()
        self.map_widget.clear_trails()
        self.map_widget.set_planned_path([])
        self.map_widget.set_waypoint_queue([])
        self.status_label.setText("Software reset to initial condition.")

    def _refresh_destinations(self):
        self.list_destinations.clear()
        robot_waypoints = {}
        for destination in self.manager.active_mission.destinations:
            robot_id = destination.robot_id or "AUTO / NEAREST"
            robot_waypoints.setdefault(robot_id, []).append((destination.x, destination.y))
            self.list_destinations.addItem(
                f"{destination.destination_id}: ({destination.x:.1f}, {destination.y:.1f}) -> {robot_id}"
            )
        self.map_widget.set_robot_waypoints(robot_waypoints)

    def ensure_robot_spawned(self):
        """Ensure a robot pose exists before teleoperating or recording."""
        if self.state.pose is None:
            x = self.spin_start_x.value()
            y = self.spin_start_y.value()
            heading = self.spin_start_heading.value()
            self.state.set_pose(x, y, heading, set_initial_if_unset=True)
            self.map_widget.set_initial_pos((x, y))
            self.manager.active_mission.start_pose = Pose(x, y, heading)
            self.status_label.setText(f"Robot auto-spawned at ({x:.1f}, {y:.1f}) | Heading {heading:.1f}°")

    def apply_start_pose(self):
        """Apply X/Y/Heading spinbox values to set or update the robot start pose."""
        x = self.spin_start_x.value()
        y = self.spin_start_y.value()
        heading = self.spin_start_heading.value()
        self.state.set_pose(x, y, heading, set_initial_if_unset=False)
        self.map_widget.set_initial_pos((x, y))
        self.manager.active_mission.start_pose = Pose(x, y, heading)
        self.status_label.setText(f"Start pose set: ({x:.1f}, {y:.1f}) | Heading {heading:.1f}°")

    def drive_manual_step(self, dist_cm: float, turn_deg: float):
        """D-Pad button action that drives robot and records into Teach Mode."""
        self.ensure_robot_spawned()

        if abs(turn_deg) > 0.1:
            self.controller.rotate_robot_delta(turn_deg)
            if self.manager.is_recording:
                self.manager.record_drive_step(
                    self.state.x, self.state.y,
                    normalize_angle_deg(self.state.heading + turn_deg),
                    action="TURN",
                    rotation=turn_deg
                )
        elif abs(dist_cm) > 0.1:
            self.controller.teleop_move(dist_cm)
            if self.manager.is_recording:
                rad = self.state.pose.heading_rad
                nx = self.state.x + dist_cm * math.cos(rad)
                ny = self.state.y + dist_cm * math.sin(rad)
                self.manager.record_drive_step(
                    nx, ny, self.state.heading,
                    action="MOVE",
                    distance=abs(dist_cm)
                )

    def toggle_recording(self):
        """Toggle Teach / Record mode."""
        self.ensure_robot_spawned()
        if not self.manager.is_recording:
            self.manager.start_teach_mode()
        else:
            self.manager.stop_teach_mode()

    def _on_teach_mode_changed(self, is_rec: bool):
        if is_rec:
            self.btn_start_record.setText("■ Stop Recording")
            self.btn_start_record.setStyleSheet("background-color: #631e1e; color: white;")
            self.lbl_record_status.setText("Recording: ACTIVE (Drive to teach)")
            self.lbl_record_status.setStyleSheet("color: #ff0055; font-weight: bold;")
        else:
            self.btn_start_record.setText("● Start Recording")
            self.btn_start_record.setStyleSheet("background-color: #2b2b2b; color: white;")
            self.lbl_record_status.setText("Recording: INACTIVE")
            self.lbl_record_status.setStyleSheet("color: #888888; font-weight: bold;")
            self._update_mission_path_display()

    def _update_mission_path_display(self):
        pts = self.manager.active_mission.path.get_coordinates()
        self.map_widget.set_planned_path(pts)
        self.lbl_mission_info.setText(
            f"Active: {self.manager.active_mission.metadata.name} ({len(pts)} points, {self.manager.active_mission.path.total_distance:.1f}cm)"
        )

    def clear_mission_path(self):
        self.manager.active_mission.path.clear()
        self.manager.active_mission.commands.clear()
        self._update_mission_path_display()
        self.status_label.setText("Mission path cleared.")

    def new_mission(self):
        name, ok = QInputDialog.getText(self, "New Mission", "Enter mission name:", text="warehouse_route_01")
        if ok and name.strip():
            self.manager.new_mission(name.strip())

    def load_mission(self):
        missions = self.manager.storage.list_missions()
        if not missions:
            QMessageBox.information(self, "No Saved Missions", "No saved mission JSON files found in missions/ directory.")
            return

        names = [f"{m['filename']} ({m['name']})" for m in missions]
        item, ok = QInputDialog.getItem(self, "Load Mission", "Select mission to load:", names, 0, False)
        if ok and item:
            filename = item.split(" (")[0]
            try:
                self.manager.load_mission(filename)
            except Exception as e:
                QMessageBox.critical(self, "Load Error", f"Failed to load mission: {e}")

    def save_mission(self):
        try:
            path = self.manager.save_mission()
            QMessageBox.information(self, "Mission Saved", f"Mission successfully saved to:\n{path}")
        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Failed to save mission: {e}")

    def add_restricted_area(self):
        dialog = AddRestrictedAreaDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            ra = dialog.get_restricted_area()
            self.manager.active_mission.environment.restricted_areas.append(ra)
            self._refresh_restricted_list()
            self.map_widget.set_restricted_areas(self.manager.active_mission.environment.restricted_areas)
            self.status_label.setText(f"Added Restricted Area: '{ra.name}'")

    def remove_restricted_area(self):
        selected = self.list_restricted.currentRow()
        if selected >= 0 and selected < len(self.manager.active_mission.environment.restricted_areas):
            removed = self.manager.active_mission.environment.restricted_areas.pop(selected)
            self._refresh_restricted_list()
            self.map_widget.set_restricted_areas(self.manager.active_mission.environment.restricted_areas)
            self.status_label.setText(f"Removed Restricted Area: '{removed.name}'")

    def _refresh_restricted_list(self):
        self.list_restricted.clear()
        for ra in self.manager.active_mission.environment.restricted_areas:
            item = QListWidgetItem(f"🚫 {ra.name} ({len(ra.polygon)} pts)")
            self.list_restricted.addItem(item)
        self.map_widget.set_restricted_areas(self.manager.active_mission.environment.restricted_areas)

    def _toggle_footprint_margin(self, checked: bool):
        self.map_widget.show_footprint_margin = checked
        self.map_widget.draw_grid()

    def validate_mission(self):
        if self.manager.active_mission.destinations and self.manager.is_multi_robot:
            try:
                assignments = self.manager.plan_destination_assignments()
                self.lbl_val_status.setText(f"PASSED: {len(assignments)} destinations assigned without overlap.")
                self.lbl_val_status.setStyleSheet("color: #00ffaa; font-weight: bold;")
            except ValueError as exc:
                self.lbl_val_status.setText(f"FAILED: {exc}")
                self.lbl_val_status.setStyleSheet("color: #ff3366; font-weight: bold;")
            return
        self.manager.validate_mission()

    def _on_validation_completed(self, is_valid: bool, errors: list):
        if is_valid:
            self.lbl_val_status.setText("✅ PASSED: Path is collision-free and inside boundary.")
            self.lbl_val_status.setStyleSheet("color: #00ffaa; font-weight: bold;")
        else:
            summary = "\n• ".join(errors[:3])
            if len(errors) > 3:
                summary += f"\n• ...and {len(errors) - 3} more issue(s)"
            self.lbl_val_status.setText(f"❌ FAILED ({len(errors)} errors):\n• {summary}")
            self.lbl_val_status.setStyleSheet("color: #ff3366; font-weight: bold;")

    def toggle_preview(self):
        """Start or stop dry-run preview animation on map."""
        if self.preview_timer.isActive():
            self.preview_timer.stop()
            self.map_widget.set_ghost_pose(None)
            self.btn_preview.setText("▶ Preview Mission (Dry Run)")
            self.status_label.setText("Mission preview stopped.")
            return

        pts = self.manager.active_mission.path.points
        if not pts:
            QMessageBox.warning(self, "Empty Path", "No path points to preview. Record or load a mission first.")
            return

        # Generate interpolated preview poses
        self.preview_points = []
        for i in range(len(pts) - 1):
            p1 = pts[i]
            p2 = pts[i + 1]
            dist = math.hypot(p2.x - p1.x, p2.y - p1.y)
            steps = max(1, int(dist / 2.0))
            for s in range(steps):
                t = s / steps
                ix = p1.x + (p2.x - p1.x) * t
                iy = p1.y + (p2.y - p1.y) * t
                self.preview_points.append((ix, iy, p2.heading))
        self.preview_points.append((pts[-1].x, pts[-1].y, pts[-1].heading))

        self.preview_index = 0
        self.preview_timer.start()
        self.btn_preview.setText("■ Stop Preview")
        self.status_label.setText("PREVIEW RUNNING | Simulating robot path...")

    def _update_preview_step(self):
        if self.preview_index >= len(self.preview_points):
            self.preview_timer.stop()
            self.map_widget.set_ghost_pose(None)
            self.btn_preview.setText("▶ Preview Mission (Dry Run)")
            self.status_label.setText("Mission preview finished.")
            return

        pose = self.preview_points[self.preview_index]
        self.map_widget.set_ghost_pose(pose)
        self.preview_index += 1

    def execute_mission(self):
        success = self.manager.execute_mission()
        if not success:
            QMessageBox.warning(self, "Execution Blocked", "Mission execution cannot proceed. Check validation status or commands.")

    def _on_executor_step(self, curr: int, total: int, desc: str):
        self.status_label.setText(f"EXECUTING STEP {curr}/{total}: {desc}")

    def _on_executor_completed(self):
        self.status_label.setText("MISSION EXECUTION COMPLETED SUCCESSFULLY!")
        self.progress_bar.setValue(100)
        QMessageBox.information(self, "Mission Done", "All mission commands executed successfully on the robot!")

    def _on_safety_halt(self):
        self.status_label.setText("SAFETY HALT DETECTED | Mission aborted due to hardware safety sensor!")
        self.progress_bar.setStyleSheet("QProgressBar::chunk { background-color: #ff0055; }")

    def _on_mission_loaded(self, mission: Mission):
        self._update_mission_path_display()
        self._refresh_destinations()
        self._refresh_restricted_list()
        self.map_widget.set_boundary(mission.environment.boundary)
        self.map_widget.set_restricted_areas(mission.environment.restricted_areas)
        self.lbl_val_status.setText("Validation: Not Checked")
        self.lbl_val_status.setStyleSheet("color: #ffaa00; font-weight: bold;")
        self.progress_bar.setValue(0)

    def keyPressEvent(self, event):
        focused = self.focusWidget()
        if isinstance(focused, (QLineEdit, QDoubleSpinBox)):
            super().keyPressEvent(event)
            return

        key = event.key()
        if key in (Qt.Key_W, Qt.Key_Up):
            self.drive_manual_step(10.0, 0.0)
        elif key in (Qt.Key_S, Qt.Key_Down):
            self.drive_manual_step(-10.0, 0.0)
        elif key in (Qt.Key_A, Qt.Key_Left):
            self.drive_manual_step(0.0, 15.0)
        elif key in (Qt.Key_D, Qt.Key_Right):
            self.drive_manual_step(0.0, -15.0)
        elif key == Qt.Key_Space:
            if self.manager.executor.is_running:
                if self.manager.executor.is_paused:
                    self.manager.resume_mission()
                else:
                    self.manager.pause_mission()
        else:
            super().keyPressEvent(event)
