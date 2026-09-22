"""Live VL53L0X continuous LiDAR mapping page."""
import math
import time
import heapq
import urllib.error
import urllib.request
from typing import List

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QDoubleSpinBox, QGroupBox, QFormLayout
)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from PyQt5.QtCore import QUrl
from PyQt5.QtNetwork import QNetworkAccessManager, QNetworkRequest
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon as MplPolygon

from navigation.lidar_protocol import decode_lidar_packet


class LidarEventThread(QThread):
    """Read the firmware's Server-Sent Events stream without blocking Qt."""
    packet_received = pyqtSignal(str)
    connection_changed = pyqtSignal(bool, str)

    def __init__(self, host: str, parent=None):
        super().__init__(parent)
        self.url = f"http://{host}/events"
        self._running = True

    def run(self):
        while self._running:
            try:
                request = urllib.request.Request(
                    self.url, headers={"Accept": "text/event-stream"}
                )
                with urllib.request.urlopen(request, timeout=5) as response:
                    self.connection_changed.emit(True, "SSE connected")
                    event_name = ""
                    for raw_line in response:
                        if not self._running:
                            break
                        line = raw_line.decode("utf-8", errors="replace").strip()
                        if line.startswith("event:"):
                            event_name = line[6:].strip()
                        elif line.startswith("data:") and event_name == "lidar_packet":
                            self.packet_received.emit(line[5:].strip())
                        elif line.startswith("data:") and event_name == "robot_pose":
                            self.packet_received.emit("POSE:" + line[5:].strip())
                        elif not line:
                            event_name = ""
            except (OSError, urllib.error.URLError):
                if self._running:
                    self.connection_changed.emit(False, "SSE unavailable")
                    self.msleep(500)

    def stop(self):
        self._running = False


class LidarMappingPage(QWidget):
    """Accumulate continuous firmware scans in world coordinates."""
    ROBOT_LENGTH_CM = 11.0
    ROBOT_WIDTH_CM = 9.0
    LIDAR_STEP_ANGLE_DEG = 1.0
    ROUTE_TURN_SIGN = -1.0

    def __init__(self, host: str, comm, parent=None):
        super().__init__(parent)
        self.host = host
        self.comm = comm
        self.http = QNetworkAccessManager(self)
        self.max_range_cm = 300.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0
        self.sweep_degrees = 180.0
        self.forward_step_cm = 10.0
        self.last_angle = None
        self.last_packet = ""
        self.last_packet_time = 0.0
        self.last_reached = ""
        self.mapping_active = False
        self.scan_points: List[tuple] = []
        self.scan_points_by_angle = {}
        self.map_points: List[tuple] = []
        self.lidar_rays: List[tuple] = []
        self.current_sweep_walls: List[tuple] = []
        self.last_wall_segments: List[tuple] = []
        self.sweep_segments_finalized = False
        self.free_rays: List[tuple] = []
        self.free_rays_by_angle = {}
        self.robot_trail: List[tuple] = []
        self.home_pose = None
        self.goal_point = None
        self.route_points: List[tuple] = []
        self.route_commands: List[tuple] = []
        self.route_waiting_for = None
        self._is_panning = False
        self._pan_start = None
        self._pan_limits = None
        self._user_view = False

        self.event_thread = LidarEventThread(host, self)
        self.redraw_timer = QTimer(self)
        self.redraw_timer.setInterval(80)
        self.redraw_timer.setSingleShot(True)
        self.redraw_timer.timeout.connect(self._draw_map)
        self.figure, self.axes = plt.subplots(figsize=(8, 6), facecolor="#121212")
        self.canvas = FigureCanvas(self.figure)
        self._build_ui()
        self.canvas.mpl_connect("scroll_event", self._on_scroll)
        self.canvas.mpl_connect("button_press_event", self._on_mouse_press)
        self.canvas.mpl_connect("button_release_event", self._on_mouse_release)
        self.canvas.mpl_connect("motion_notify_event", self._on_mouse_move)
        self.canvas.mpl_connect("button_press_event", self._on_goal_click)
        self._draw_map()
        self.event_thread.packet_received.connect(self.handle_lidar_packet)
        self.event_thread.connection_changed.connect(self._on_connection_changed)
        self.event_thread.start()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        toolbar = QHBoxLayout()

        title = QLabel("LIVE LIDAR MAP")
        title.setStyleSheet("color: #00d2ff; font-weight: bold; font-size: 13px;")
        toolbar.addWidget(title)
        self.status_label = QLabel("Waiting for LiDAR data")
        self.status_label.setStyleSheet("color: #ffcc00; font-family: monospace;")
        toolbar.addWidget(self.status_label)
        toolbar.addStretch()

        controls = QGroupBox("Continuous scan")
        form = QFormLayout(controls)
        self.sweep_spin = QDoubleSpinBox()
        self.sweep_spin.setRange(10.0, 360.0)
        self.sweep_spin.setSingleStep(10.0)
        self.sweep_spin.setValue(self.sweep_degrees)
        self.sweep_spin.setSuffix(" deg")
        form.addRow("Sweep", self.sweep_spin)
        self.step_angle_spin = QDoubleSpinBox()
        self.step_angle_spin.setRange(0.5, 30.0)
        self.step_angle_spin.setSingleStep(0.5)
        self.step_angle_spin.setValue(self.LIDAR_STEP_ANGLE_DEG)
        self.step_angle_spin.setSuffix(" deg")
        self.step_angle_spin.valueChanged.connect(self._set_lidar_step_angle)
        form.addRow("LiDAR step", self.step_angle_spin)
        self.step_spin = QDoubleSpinBox()
        self.step_spin.setRange(0.1, 500.0)
        self.step_spin.setValue(self.forward_step_cm)
        self.step_spin.setSuffix(" cm")
        form.addRow("Forward", self.step_spin)
        self.range_spin = QDoubleSpinBox()
        self.range_spin.setRange(1.0, 10000.0)
        self.range_spin.setValue(self.max_range_cm)
        self.range_spin.setSuffix(" cm")
        self.range_spin.valueChanged.connect(self._set_max_range)
        form.addRow("Max range", self.range_spin)
        toolbar.addWidget(controls)

        clear_button = QPushButton("Clear")
        clear_button.clicked.connect(self.clear_scan)
        toolbar.addWidget(clear_button)
        navigate_button = QPushButton("Navigate to Goal")
        navigate_button.setObjectName("primaryBtn")
        navigate_button.clicked.connect(self._plan_route_to_goal)
        toolbar.addWidget(navigate_button)
        emergency_button = QPushButton("Emergency Stop")
        emergency_button.setObjectName("dangerBtn")
        emergency_button.clicked.connect(self.emergency_stop)
        toolbar.addWidget(emergency_button)
        fit_button = QPushButton("Fit View")
        fit_button.clicked.connect(self.fit_view)
        toolbar.addWidget(fit_button)
        self.scan_button = QPushButton("Start 180 deg / 10 cm")
        self.scan_button.setObjectName("primaryBtn")
        self.scan_button.clicked.connect(self.toggle_scan)
        toolbar.addWidget(self.scan_button)
        layout.addLayout(toolbar)

        self.telemetry_label = QLabel("Map points: 0 | Pose: 0.0, 0.0 cm")
        self.telemetry_label.setObjectName("telemetryText")
        layout.addWidget(self.telemetry_label)
        layout.addWidget(self.canvas)

    def _on_goal_click(self, event):
        if event.button != 1 or event.xdata is None or event.ydata is None:
            return
        self.goal_point = (float(event.xdata), float(event.ydata))
        self._schedule_draw()
        self.status_label.setText(
            f"Goal selected at ({self.goal_point[0]:.1f}, {self.goal_point[1]:.1f}) cm"
        )

    def _set_max_range(self, value: float):
        self.max_range_cm = value
        self._draw_map()

    def _set_lidar_step_angle(self, value: float):
        self.LIDAR_STEP_ANGLE_DEG = float(value)
        self.http.get(QNetworkRequest(QUrl(
            f"http://{self.host}/config?param=lidar_step&val={value:g}"
        )))

    def set_robot_pose(self, x: float, y: float, heading: float):
        if not self.mapping_active and not self.route_waiting_for and not self.route_commands:
            self.robot_x = float(x)
            self.robot_y = float(y)
            if self.home_pose is None:
                self.home_pose = (self.robot_x, self.robot_y)
            if not self.robot_trail:
                self.robot_trail.append((self.robot_x, self.robot_y))
        self.robot_heading = float(heading)
        self._update_telemetry()
        self._draw_map()

    def handle_lidar_packet(self, packet_hex: str):
        if packet_hex.startswith("POSE:"):
            self.handle_udp_message(packet_hex[5:])
            return
        now = time.monotonic()
        if packet_hex == self.last_packet and now - self.last_packet_time < 0.2:
            return
        self.last_packet = packet_hex
        self.last_packet_time = now
        decoded = decode_lidar_packet(packet_hex)
        if decoded is None:
            return
        angle, distance = decoded

        # The firmware starts every cycle at angle 0. Position is deliberately
        # unchanged here; it advances only after REACHED feedback arrives.
        self.last_angle = angle

        half_sweep = self.sweep_degrees / 2.0
        if angle < half_sweep:
            scan_heading = self.robot_heading - (angle + self.LIDAR_STEP_ANGLE_DEG)
        else:
            scan_heading = self.robot_heading + (angle - half_sweep)
        world_angle = math.radians(scan_heading)
        direction_x = math.cos(world_angle)
        direction_y = math.sin(world_angle)
        sensor_offset = self.ROBOT_LENGTH_CM / 2.0
        robot_heading_radians = math.radians(self.robot_heading)
        sensor_origin = (
            self.robot_x + sensor_offset * math.cos(robot_heading_radians),
            self.robot_y + sensor_offset * math.sin(robot_heading_radians),
        )
        ray_key = round(angle, 2)

        if 0.0 < distance <= self.max_range_cm:
            endpoint = (
                sensor_origin[0] + distance * direction_x,
                sensor_origin[1] + distance * direction_y,
            )
            origin = sensor_origin
            ray = (origin, endpoint)
            previous_ray = self.scan_points_by_angle.get(ray_key)
            if previous_ray is not None and previous_ray[0] <= distance:
                return
            if previous_ray is not None:
                self.map_points.remove(previous_ray[1])
                self.lidar_rays.remove(previous_ray[2])
                self.current_sweep_walls = [
                    item for item in self.current_sweep_walls
                    if item[0] != angle
                ]
            free_ray = self.free_rays_by_angle.pop(ray_key, None)
            if free_ray is not None:
                self.free_rays.remove(free_ray)
            self.map_points.append(endpoint)
            self.lidar_rays.append(ray)
            self.scan_points_by_angle[ray_key] = (distance, endpoint, ray)
            self.current_sweep_walls.append((angle, endpoint, ray))
        else:
            # A zero/invalid range means no wall was returned. Extend only
            # that ray to the visible map limit and draw it in white.
            free_endpoint = (
                sensor_origin[0] + self.max_range_cm * direction_x,
                sensor_origin[1] + self.max_range_cm * direction_y,
            )
            free_ray = (sensor_origin, free_endpoint)
            previous_free_ray = self.free_rays_by_angle.get(ray_key)
            if previous_free_ray is not None:
                self.free_rays.remove(previous_free_ray)
            self.free_rays.append(free_ray)
            self.free_rays_by_angle[ray_key] = free_ray

        if not self.sweep_segments_finalized and angle >= self.sweep_degrees - 0.01:
            self._complete_wall_polygon()
            self.sweep_segments_finalized = True
            self.status_label.setText("Sweep complete; wall segments updated before forward move")

        self.status_label.setText(
            f"Receiving {self.sweep_degrees:.0f} deg scan, angle {angle:.0f} deg"
        )
        self.status_label.setStyleSheet("color: #00ffaa; font-family: monospace;")
        self._update_telemetry()
        self._schedule_draw()

    def handle_udp_message(self, message: str):
        """Accept LiDAR packets and confirmed physical pose feedback."""
        if message.startswith("LIDAR_PACKET:"):
            self.handle_lidar_packet(message.split(":", 1)[1])
        elif message.startswith("REACHED:LIDAR:"):
            if message == self.last_reached:
                return
            self.last_reached = message
            fields = message.split(":")
            if len(fields) == 5:
                try:
                    self.robot_x = float(fields[2])
                    self.robot_y = float(fields[3])
                    self.robot_heading = float(fields[4])
                except ValueError:
                    return
                self.robot_trail.append((self.robot_x, self.robot_y))
                self.last_angle = None
                self.scan_points_by_angle.clear()
                self.free_rays.clear()
                self.free_rays_by_angle.clear()
                self.sweep_segments_finalized = False
                if self.route_waiting_for == "MOVE":
                    self.route_waiting_for = None
                    self._send_next_route_command()
                self.status_label.setText("Physical move reached; mapping pose confirmed")
                self._update_telemetry()
                self._schedule_draw()
        elif message.startswith("COMPLETED:TURN") and self.route_waiting_for == "TURN":
            self.robot_heading = getattr(self, "navigation_target_heading", self.robot_heading)
            self.route_waiting_for = None
            self._update_telemetry()
            self._schedule_draw()
            self._send_next_route_command()
        elif message.startswith("COMPLETED:MOVE") and self.route_waiting_for == "MOVE":
            if hasattr(self, "navigation_target"):
                self.robot_x, self.robot_y = self.navigation_target
                self.robot_trail.append((self.robot_x, self.robot_y))
                self._update_telemetry()
                self._schedule_draw()
            self.route_waiting_for = None
            self._send_next_route_command()
        elif message.startswith("ALERT:") and self.route_waiting_for:
            self.route_commands.clear()
            self.route_waiting_for = None
            self.status_label.setText("Route stopped by robot safety alert")

    def toggle_scan(self):
        if self.scan_button.text().startswith("Stop"):
            self.comm.send_command("STOP")
            self.scan_button.setText("Start 180 deg / 10 cm")
            self.status_label.setText("Continuous scan stopping")
            self.mapping_active = False
            return

        self.sweep_degrees = self.sweep_spin.value()
        self.forward_step_cm = self.step_spin.value()
        self.last_angle = None
        self.scan_points_by_angle.clear()
        self.current_sweep_walls.clear()
        self.free_rays.clear()
        self.free_rays_by_angle.clear()
        self.sweep_segments_finalized = False
        self.mapping_active = True
        self.comm.send_command(
            f"LIDAR:{self.sweep_degrees:g}:{self.forward_step_cm:g}:"
            f"{self.robot_x:.2f}:{self.robot_y:.2f}:{self.robot_heading:.2f}"
        )
        self.scan_button.setText("Stop Continuous Scan")
        self.status_label.setText(
            f"Starting {self.sweep_degrees:.0f} deg scan / "
            f"{self.forward_step_cm:.1f} cm step"
        )

    def emergency_stop(self):
        """Immediately stop physical motion and cancel all LiDAR navigation."""
        self.comm.send_command("STOP")
        self.mapping_active = False
        self.route_commands.clear()
        self.route_waiting_for = None
        self.scan_button.setText("Start 180 deg / 10 cm")
        self.status_label.setText("EMERGENCY STOP SENT")
        self.status_label.setStyleSheet("color: #ff3366; font-family: monospace; font-weight: bold;")

    def _plan_route_to_goal(self):
        if self.goal_point is None:
            return
        if self.route_waiting_for or self.route_commands:
            self.status_label.setText("A route is already executing")
            return
        start = (self.robot_x, self.robot_y)
        goal = self.goal_point
        if math.dist(start, goal) < 1.0:
            self.status_label.setText("Goal is already reached")
            return

        if self.mapping_active:
            self.comm.send_command("STOP")
            self.mapping_active = False
            self.scan_button.setText("Start 180 deg / 10 cm")

        route = self._astar_route(start, goal)
        if not route:
            self.status_label.setText("No safe route found to selected goal")
            return
        self.route_points = route
        self.route_commands = self._route_to_commands(route)
        self.status_label.setText("Safe route planned; waiting to execute")
        QTimer.singleShot(350, self._send_next_route_command)

    def _astar_route(self, start, goal):
        grid = 2.0
        clearance = math.hypot(
            self.ROBOT_LENGTH_CM / 2.0,
            self.ROBOT_WIDTH_CM / 2.0,
        ) + 2.0
        occupied = set()
        obstacle_segments = list(self.last_wall_segments)

        def point_segment_distance(point, first, second):
            dx = second[0] - first[0]
            dy = second[1] - first[1]
            length_squared = dx * dx + dy * dy
            if length_squared == 0.0:
                return math.dist(point, first)
            projection = ((point[0] - first[0]) * dx + (point[1] - first[1]) * dy) / length_squared
            projection = max(0.0, min(1.0, projection))
            nearest = (first[0] + projection * dx, first[1] + projection * dy)
            return math.dist(point, nearest)

        def is_blocked(point):
            return any(
                point_segment_distance(point, first, second) <= clearance
                for first, second in obstacle_segments
            )

        def line_is_clear(first, second):
            sample_count = max(2, int(math.ceil(math.dist(first, second) / (grid / 2.0))))
            for index in range(sample_count + 1):
                factor = index / sample_count
                sample = (
                    first[0] + (second[0] - first[0]) * factor,
                    first[1] + (second[1] - first[1]) * factor,
                )
                if is_blocked(sample):
                    return False
            return True

        all_obstacle_points = [point for segment in obstacle_segments for point in segment]
        all_points = [start, goal] + all_obstacle_points
        min_x = min(point[0] for point in all_points) - 30.0
        max_x = max(point[0] for point in all_points) + 30.0
        min_y = min(point[1] for point in all_points) - 30.0
        max_y = max(point[1] for point in all_points) + 30.0

        def to_cell(point):
            return round((point[0] - min_x) / grid), round((point[1] - min_y) / grid)

        def to_world(cell):
            return min_x + cell[0] * grid, min_y + cell[1] * grid

        max_cell_x = int(math.ceil((max_x - min_x) / grid))
        max_cell_y = int(math.ceil((max_y - min_y) / grid))
        for cell_x in range(max_cell_x + 1):
            for cell_y in range(max_cell_y + 1):
                if is_blocked(to_world((cell_x, cell_y))):
                    occupied.add((cell_x, cell_y))

        start_cell = to_cell(start)
        goal_cell = to_cell(goal)
        occupied.discard(start_cell)
        occupied.discard(goal_cell)
        frontier = [(0.0, start_cell)]
        came_from = {start_cell: None}
        cost = {start_cell: 0.0}
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal_cell:
                break
            for step_x, step_y in (
                (1, 0), (-1, 0), (0, 1), (0, -1),
                (1, 1), (1, -1), (-1, 1), (-1, -1),
            ):
                neighbor = (current[0] + step_x, current[1] + step_y)
                if neighbor in occupied or not (
                    0 <= neighbor[0] <= max_cell_x and
                    0 <= neighbor[1] <= max_cell_y
                ):
                    continue
                if step_x and step_y:
                    if ((current[0] + step_x, current[1]) in occupied or
                            (current[0], current[1] + step_y) in occupied):
                        continue
                new_cost = cost[current] + math.hypot(step_x, step_y)
                if new_cost < cost.get(neighbor, float("inf")):
                    cost[neighbor] = new_cost
                    priority = new_cost + math.dist(to_world(neighbor), goal)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current
        if goal_cell not in came_from:
            return []
        cells = []
        current = goal_cell
        while current is not None:
            cells.append(current)
            current = came_from[current]
        cells.reverse()
        raw_route = [to_world(cell) for cell in reversed(cells)]
        simplified = [raw_route[0]]
        anchor = 0
        while anchor < len(raw_route) - 1:
            furthest = anchor + 1
            for candidate in range(anchor + 1, len(raw_route)):
                if line_is_clear(raw_route[anchor], raw_route[candidate]):
                    furthest = candidate
                else:
                    break
            simplified.append(raw_route[furthest])
            anchor = furthest
        simplified[-1] = goal
        return simplified

    def _route_to_commands(self, route):
        commands = []
        current_heading = self.robot_heading
        for first, second in zip(route, route[1:]):
            dx = second[0] - first[0]
            dy = second[1] - first[1]
            distance = math.hypot(dx, dy)
            target_heading = math.degrees(math.atan2(dy, dx))
            turn = ((target_heading - current_heading + 180.0) % 360.0) - 180.0
            if abs(turn) > 0.5:
                commands.append(("TURN", turn, target_heading))
                current_heading = target_heading
            commands.append(("MOVE", distance, second[0], second[1]))
        return commands

    def _send_next_route_command(self):
        if self.route_waiting_for:
            return
        if not self.route_commands:
            self.route_waiting_for = None
            self.status_label.setText("Goal reached")
            return
        command_data = self.route_commands.pop(0)
        command, value = command_data[:2]
        self.route_waiting_for = command
        if command == "TURN":
            desired_heading = command_data[2]
            physical_turn = ((desired_heading - self.robot_heading + 180.0) % 360.0) - 180.0
            physical_turn *= self.ROUTE_TURN_SIGN
            self.navigation_target_heading = desired_heading % 360.0
            self.comm.send_command(f"TURN:{physical_turn:.1f}")
        else:
            self.navigation_target = (command_data[2], command_data[3])
            self.comm.send_command(f"MOVE:{value:.1f}:1.0")
        self.status_label.setText(f"Executing route: {command} {value:.1f}")

    def _update_telemetry(self):
        self.telemetry_label.setText(
            f"Map points: {len(self.map_points)} | "
            f"Pose: {self.robot_x:.1f}, {self.robot_y:.1f} cm | "
            f"Heading: {self.robot_heading:.1f} deg"
        )

    def _schedule_draw(self):
        """Coalesce rapid LiDAR packets into the next timer redraw."""
        if not self.redraw_timer.isActive():
            self.redraw_timer.start()

    def _on_connection_changed(self, connected: bool, message: str):
        if not connected and not self.map_points:
            self.status_label.setText(message)
            self.status_label.setStyleSheet("color: #ffcc00; font-family: monospace;")

    def clear_scan(self):
        self.map_points.clear()
        self.lidar_rays.clear()
        self.current_sweep_walls.clear()
        self.last_wall_segments.clear()
        self.sweep_segments_finalized = False
        self.robot_trail.clear()
        self.goal_point = None
        self.route_points.clear()
        self.route_commands.clear()
        self.route_waiting_for = None
        self.scan_points_by_angle.clear()
        self.free_rays.clear()
        self.free_rays_by_angle.clear()
        self.last_angle = None
        self.last_packet = ""
        self.last_packet_time = 0.0
        self.last_reached = ""
        self.status_label.setText("Waiting for LiDAR data")
        self.status_label.setStyleSheet("color: #ffcc00; font-family: monospace;")
        self._update_telemetry()
        self._draw_map()

    def _complete_wall_polygon(self):
        """Create adjacent wall segments and remove raw sweep points."""
        walls = sorted(self.current_sweep_walls, key=lambda item: item[0])
        if len(walls) < 2:
            self.current_sweep_walls.clear()
            return

        wall_points = [item[1] for item in walls]
        max_connection_gap_cm = 5.0
        self.last_wall_segments = []
        for start, end in zip(wall_points, wall_points[1:]):
            if math.dist(start, end) <= max_connection_gap_cm:
                self.last_wall_segments.append((start, end))

        for _, endpoint, ray in self.current_sweep_walls:
            if endpoint in self.map_points:
                self.map_points.remove(endpoint)
            if ray in self.lidar_rays:
                self.lidar_rays.remove(ray)
        self.current_sweep_walls.clear()

    def fit_view(self):
        """Fit the viewport to all map points and return to automatic tracking."""
        self._user_view = False
        self._draw_map()

    def _on_scroll(self, event):
        if event.xdata is None or event.ydata is None:
            return
        scale = 0.8 if event.button == "up" else 1.25
        x_min, x_max = self.axes.get_xlim()
        y_min, y_max = self.axes.get_ylim()
        new_width = (x_max - x_min) * scale
        new_height = (y_max - y_min) * scale
        x_ratio = (event.xdata - x_min) / (x_max - x_min)
        y_ratio = (event.ydata - y_min) / (y_max - y_min)
        self.axes.set_xlim(
            event.xdata - new_width * x_ratio,
            event.xdata + new_width * (1.0 - x_ratio),
        )
        self.axes.set_ylim(
            event.ydata - new_height * y_ratio,
            event.ydata + new_height * (1.0 - y_ratio),
        )
        self._user_view = True
        self.canvas.draw_idle()

    def _on_mouse_press(self, event):
        if event.button == 2 and event.x is not None and event.y is not None:
            self._is_panning = True
            self._pan_start = (event.x, event.y)
            self._pan_limits = (self.axes.get_xlim(), self.axes.get_ylim())

    def _on_mouse_release(self, event):
        if event.button == 2:
            self._is_panning = False
            self._pan_start = None

    def _on_mouse_move(self, event):
        if not self._is_panning or self._pan_start is None:
            return
        if event.x is None or event.y is None:
            return
        bbox = self.axes.get_window_extent()
        if bbox.width <= 0 or bbox.height <= 0:
            return
        dx = event.x - self._pan_start[0]
        dy = event.y - self._pan_start[1]
        x_limits, y_limits = self._pan_limits
        x_shift = dx / bbox.width * (x_limits[1] - x_limits[0])
        y_shift = dy / bbox.height * (y_limits[1] - y_limits[0])
        self.axes.set_xlim(x_limits[0] - x_shift, x_limits[1] - x_shift)
        self.axes.set_ylim(y_limits[0] - y_shift, y_limits[1] - y_shift)
        self._user_view = True
        self.canvas.draw_idle()

    def _draw_map(self):
        saved_xlim = self.axes.get_xlim() if self._user_view else None
        saved_ylim = self.axes.get_ylim() if self._user_view else None
        self.axes.clear()
        self.axes.set_facecolor("#242424")
        self.axes.set_aspect("equal", adjustable="box")
        if not self._user_view:
            visible_points = self.map_points + self.robot_trail + [(self.robot_x, self.robot_y)]
            min_x = min(point[0] for point in visible_points)
            max_x = max(point[0] for point in visible_points)
            min_y = min(point[1] for point in visible_points)
            max_y = max(point[1] for point in visible_points)
            half_range = max(
                self.max_range_cm,
                (max_x - min_x) / 2.0,
                (max_y - min_y) / 2.0,
            )
            center_x = (min_x + max_x) / 2.0
            center_y = (min_y + max_y) / 2.0
            self.axes.set_xlim(center_x - half_range, center_x + half_range)
            self.axes.set_ylim(center_y - half_range, center_y + half_range)
        else:
            self.axes.set_xlim(saved_xlim)
            self.axes.set_ylim(saved_ylim)
        self.axes.set_xlabel("World X (cm)", color="#b8b8b8")
        self.axes.set_ylabel("World Y (cm)", color="#b8b8b8")
        self.axes.grid(color="#3a3a3a", linestyle=":", linewidth=0.7)

        if self.map_points:
            xs, ys = zip(*self.map_points)
            self.axes.scatter(xs, ys, s=18, color="#e53935", edgecolors="#ff8a80",
                              linewidths=0.3, alpha=0.95, zorder=4, label="Walls")

        if self.lidar_rays:
            self.axes.add_collection(LineCollection(
                self.lidar_rays,
                colors="#b0b0b0",
                linewidths=0.8,
                alpha=0.55,
                zorder=2,
                label="Clear path",
            ))

        if self.free_rays:
            self.axes.add_collection(LineCollection(
                self.free_rays,
                colors="#ffffff",
                linewidths=0.7,
                alpha=0.6,
                zorder=1,
                label="Open space",
            ))

        if self.last_wall_segments:
            self.axes.add_collection(LineCollection(
                self.last_wall_segments,
                colors="#e53935",
                linewidths=2.0,
                alpha=0.95,
                zorder=4,
                label="Adjacent wall segments",
            ))

        if len(self.robot_trail) > 1:
            trail_x, trail_y = zip(*self.robot_trail)
            self.axes.plot(trail_x, trail_y, color="#b0b0b0", linewidth=2.5,
                           linestyle="-", alpha=0.95, label="Clear path")

        if self.route_points and len(self.route_points) > 1:
            route_x, route_y = zip(*self.route_points)
            self.axes.plot(route_x, route_y, color="#00d2ff", linewidth=1.5,
                           linestyle="--", alpha=0.85, label="Planned route")

        if self.home_pose:
            self.axes.scatter(
                [self.home_pose[0]], [self.home_pose[1]],
                marker="*", s=130, color="#00ffaa", edgecolors="white",
                linewidths=0.8, zorder=6, label="Home",
            )

        if self.goal_point:
            self.axes.scatter(
                [self.goal_point[0]], [self.goal_point[1]],
                marker="X", s=100, color="#ff9900", edgecolors="white",
                linewidths=0.8, zorder=6, label="Goal",
            )

        half_length = self.ROBOT_LENGTH_CM / 2.0
        half_width = self.ROBOT_WIDTH_CM / 2.0
        heading = math.radians(self.robot_heading)
        forward = (math.cos(heading), math.sin(heading))
        side = (-math.sin(heading), math.cos(heading))
        robot_corners = [
            (self.robot_x + forward[0] * half_length + side[0] * half_width,
             self.robot_y + forward[1] * half_length + side[1] * half_width),
            (self.robot_x + forward[0] * half_length - side[0] * half_width,
             self.robot_y + forward[1] * half_length - side[1] * half_width),
            (self.robot_x - forward[0] * half_length - side[0] * half_width,
             self.robot_y - forward[1] * half_length - side[1] * half_width),
            (self.robot_x - forward[0] * half_length + side[0] * half_width,
             self.robot_y - forward[1] * half_length + side[1] * half_width),
        ]
        self.axes.add_patch(MplPolygon(
            robot_corners, closed=True, facecolor="#ffd700",
            edgecolor="white", linewidth=1.2, alpha=0.9,
            label="Robot 11 x 9 cm", zorder=5,
        ))
        heading_x = 12.0 * math.cos(math.radians(self.robot_heading))
        heading_y = 12.0 * math.sin(math.radians(self.robot_heading))
        self.axes.arrow(self.robot_x, self.robot_y, heading_x, heading_y,
                        color="#ffd700", width=0.5, head_width=4.0,
                        length_includes_head=True)
        self.axes.legend(loc="upper right", facecolor="#1e1e1e",
                         edgecolor="#444444", fontsize=8)
        self.canvas.draw_idle()

    def close(self):
        self.redraw_timer.stop()
        if self.event_thread.isRunning():
            self.event_thread.stop()
            self.event_thread.wait(1500)
