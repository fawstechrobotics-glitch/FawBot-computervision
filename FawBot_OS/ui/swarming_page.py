"""Virtual choreography recorder and simultaneous fleet execution page."""
import math
from typing import Dict, List, Tuple

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel, QListWidget,
    QPushButton, QVBoxLayout, QWidget
)

from models.path import Command
from navigation.navigation_executor import NavigationExecutor
from robot.fleet_controller import RobotFleet
from ui.map_widget import MapWidget


class SwarmingPage(QWidget):
    """Record a virtual robot dance and replay identical commands on all robots."""

    def __init__(self, fleet: RobotFleet, parent=None):
        super().__init__(parent)
        self.fleet = fleet
        self.commands: List[Command] = []
        self.virtual_pose = [75.0, 75.0, 0.0]
        self.virtual_path: List[Tuple[float, float]] = [(75.0, 75.0)]
        self.executors: Dict[str, NavigationExecutor] = {}
        self.completed_robot_ids = set()
        self._build_ui()
        self._draw_virtual_path()

    def _build_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        self.map_widget = MapWidget()
        layout.addWidget(self.map_widget, stretch=3)

        panel = QWidget()
        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(4, 4, 8, 4)
        panel_layout.setSpacing(6)

        title = QLabel("SWARMING / ROBOT DANCE")
        title.setStyleSheet("color: #00d2ff; font-size: 14px; font-weight: bold;")
        panel_layout.addWidget(title)
        panel_layout.addWidget(QLabel(
            "Virtually record one choreography, then execute the same commands on every robot."
        ))

        move_box = QGroupBox("VIRTUAL CHOREOGRAPHY")
        move_layout = QVBoxLayout(move_box)
        row = QHBoxLayout()
        row.addWidget(QLabel("Move:"))
        self.move_distance = QDoubleSpinBox()
        self.move_distance.setRange(0.1, 500.0)
        self.move_distance.setValue(10.0)
        self.move_distance.setSuffix(" cm")
        self.move_distance.setDecimals(1)
        row.addWidget(self.move_distance)
        btn_forward = QPushButton("Forward")
        btn_forward.clicked.connect(lambda: self.record_move(1.0))
        btn_backward = QPushButton("Backward")
        btn_backward.clicked.connect(lambda: self.record_move(-1.0))
        row.addWidget(btn_forward)
        row.addWidget(btn_backward)
        move_layout.addLayout(row)

        turn_row = QHBoxLayout()
        turn_row.addWidget(QLabel("Turn:"))
        self.turn_angle = QDoubleSpinBox()
        self.turn_angle.setRange(1.0, 360.0)
        self.turn_angle.setValue(90.0)
        self.turn_angle.setSuffix(" deg")
        self.turn_angle.setDecimals(1)
        turn_row.addWidget(self.turn_angle)
        btn_left = QPushButton("Left")
        btn_left.clicked.connect(lambda: self.record_turn(1.0))
        btn_right = QPushButton("Right")
        btn_right.clicked.connect(lambda: self.record_turn(-1.0))
        turn_row.addWidget(btn_left)
        turn_row.addWidget(btn_right)
        move_layout.addLayout(turn_row)

        panel_layout.addWidget(move_box)

        self.command_list = QListWidget()
        self.command_list.setMinimumHeight(180)
        panel_layout.addWidget(QLabel("Recorded command sequence"))
        panel_layout.addWidget(self.command_list)

        self.lbl_pose = QLabel()
        panel_layout.addWidget(self.lbl_pose)

        record_row = QHBoxLayout()
        btn_clear = QPushButton("Clear Dance")
        btn_clear.clicked.connect(self.clear_dance)
        btn_undo = QPushButton("Undo Last")
        btn_undo.clicked.connect(self.undo_last)
        record_row.addWidget(btn_clear)
        record_row.addWidget(btn_undo)
        panel_layout.addLayout(record_row)

        self.btn_execute = QPushButton("Execute Dance on All Robots")
        self.btn_execute.setObjectName("primaryBtn")
        self.btn_execute.clicked.connect(self.execute_swarm)
        panel_layout.addWidget(self.btn_execute)
        self.btn_stop = QPushButton("Stop All Robots")
        self.btn_stop.setObjectName("dangerBtn")
        self.btn_stop.clicked.connect(self.stop_swarm)
        panel_layout.addWidget(self.btn_stop)

        self.status_label = QLabel("Ready to record choreography.")
        self.status_label.setWordWrap(True)
        panel_layout.addWidget(self.status_label)
        panel_layout.addStretch()
        layout.addWidget(panel, stretch=1)

        for button in [btn_forward, btn_backward, btn_left, btn_right, btn_clear, btn_undo,
                       self.btn_execute, self.btn_stop]:
            button.setFocusPolicy(Qt.NoFocus)

    def record_move(self, direction: float):
        distance = self.move_distance.value()
        self.commands.append(Command("MOVE", distance, direction))
        heading_rad = math.radians(self.virtual_pose[2])
        self.virtual_pose[0] += distance * direction * math.cos(heading_rad)
        self.virtual_pose[1] += distance * direction * math.sin(heading_rad)
        self.virtual_path.append((self.virtual_pose[0], self.virtual_pose[1]))
        self._refresh_recording()

    def record_turn(self, direction: float):
        angle = self.turn_angle.value() * direction
        self.commands.append(Command("TURN", angle))
        self.virtual_pose[2] = (self.virtual_pose[2] + angle + 180.0) % 360.0 - 180.0
        self._refresh_recording()

    def _refresh_recording(self):
        self.command_list.clear()
        for index, command in enumerate(self.commands, 1):
            self.command_list.addItem(f"{index:02d}. {command.to_udp_string()}")
        self._draw_virtual_path()
        self.status_label.setText(f"Recorded {len(self.commands)} command(s).")

    def _draw_virtual_path(self):
        self.map_widget.set_planned_path(self.virtual_path)
        self.map_widget.set_named_robot_pose(
            "virtual_dancer", self.virtual_pose[0], self.virtual_pose[1], self.virtual_pose[2]
        )
        self.lbl_pose.setText(
            f"Virtual pose: X {self.virtual_pose[0]:.1f} | Y {self.virtual_pose[1]:.1f} | "
            f"Heading {self.virtual_pose[2]:.1f} deg"
        )

    def clear_dance(self):
        self.stop_swarm()
        self.commands.clear()
        self.virtual_pose = [75.0, 75.0, 0.0]
        self.virtual_path = [(75.0, 75.0)]
        self._refresh_recording()

    def undo_last(self):
        if not self.commands:
            return
        self.commands.pop()
        self.virtual_pose = [75.0, 75.0, 0.0]
        self.virtual_path = [(75.0, 75.0)]
        replay = list(self.commands)
        self.commands = []
        for command in replay:
            if command.type == "MOVE":
                self.move_distance.setValue(command.value)
                self.record_move(command.direction)
            else:
                self.turn_angle.setValue(abs(command.value))
                self.record_turn(1.0 if command.value >= 0 else -1.0)

    def execute_swarm(self):
        if not self.commands:
            self.status_label.setText("Record at least one dance command first.")
            return
        self.stop_swarm()
        self.completed_robot_ids.clear()
        for robot_id, controller in self.fleet.controllers.items():
            state = self.fleet.states[robot_id]
            executor = NavigationExecutor(controller, state)
            executor.set_commands(self.commands)
            executor.completed.connect(lambda robot_id=robot_id: self._robot_completed(robot_id))
            executor.safety_halted.connect(lambda robot_id=robot_id: self._robot_halted(robot_id))
            self.executors[robot_id] = executor
            executor.start()
        self.status_label.setText(f"Executing identical dance on {len(self.executors)} robot(s).")

    def _robot_completed(self, robot_id: str):
        self.completed_robot_ids.add(robot_id)
        if len(self.completed_robot_ids) == len(self.executors):
            self.status_label.setText("All robots completed the dance.")

    def _robot_halted(self, robot_id: str):
        self.status_label.setText(f"Safety halt received from {robot_id}.")

    def stop_swarm(self):
        for executor in self.executors.values():
            executor.stop()
        self.executors.clear()
        if self.completed_robot_ids:
            self.status_label.setText("Swarm execution stopped.")

    def close(self):
        self.stop_swarm()
