"""Live VL53L0X continuous LiDAR mapping page."""
import math
import socket
import time
import heapq
import urllib.error
import urllib.request
from typing import List, Tuple, Optional

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QDoubleSpinBox, QGroupBox, QFormLayout, QSizePolicy
)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, QUrl
from PyQt5.QtGui import QFont
from PyQt5.QtNetwork import QNetworkAccessManager, QNetworkRequest
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.patches import Polygon as MplPolygon
import shapely
from shapely.geometry import Polygon as ShPolygon, MultiPolygon, Point
from shapely.ops import unary_union

from navigation.lidar_protocol import decode_lidar_packet, parse_lidar_message
from models.path import Command
from models.pose import normalize_angle_deg
from robot.robot_controller import RobotController
from robot.robot_state import RobotState
from navigation.navigation_executor import NavigationExecutor


class LidarEventThread(QThread):
    """Read the firmware's Server-Sent Events stream without blocking Qt."""
    packet_received = pyqtSignal(str)
    connection_changed = pyqtSignal(bool, str)

    def __init__(self, host: str, parent=None):
        super().__init__(parent)
        self.host = host
        self.url = f"http://{host}/events"
        self._running = True
        self._active_response = None

    def set_host(self, host: str):
        if host and host != self.host:
            self.host = host
            self.url = f"http://{host}/events"

    def run(self):
        while self._running:
            try:
                connect_url = self.url
                if ".local" in self.host:
                    try:
                        resolved_ip = socket.gethostbyname(self.host)
                        connect_url = f"http://{resolved_ip}/events"
                    except Exception:
                        connect_url = self.url

                request = urllib.request.Request(
                    connect_url, headers={"Accept": "text/event-stream"}
                )
                with urllib.request.urlopen(request, timeout=3) as response:
                    self._active_response = response
                    self.connection_changed.emit(True, f"SSE connected ({self.host})")
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
                    self._active_response = None
            except (OSError, urllib.error.URLError):
                self._active_response = None
                if self._running:
                    self.connection_changed.emit(False, "SSE unavailable")
                    self.msleep(1000)

    def stop(self):
        self._running = False
        resp = self._active_response
        if resp is not None:
            try:
                resp.close()
            except Exception:
                pass
            self._active_response = None


class LidarMappingPage(QWidget):
    """Accumulate continuous firmware scans in world coordinates."""
    ROBOT_LENGTH_CM = 11.0
    ROBOT_WIDTH_CM = 9.0
    LIDAR_STEP_ANGLE_DEG = 1.0

    def __init__(
        self,
        host: str,
        comm,
        controller: Optional[RobotController] = None,
        state: Optional[RobotState] = None,
        parent=None,
    ):
        super().__init__(parent)
        self.host = host
        self.comm = comm
        self.state = state if state is not None else RobotState()
        self.controller = controller if controller is not None else RobotController(self.comm, self.state)
        self.executor = NavigationExecutor(self.controller, self.state)

        # Wire up executor signals for clean feedback
        self.executor.step_started.connect(self._on_nav_step_started)
        self.executor.step_completed.connect(self._on_nav_step_completed)
        self.executor.completed.connect(self._on_nav_completed)
        self.executor.stopped.connect(self._on_nav_stopped)
        self.executor.safety_halted.connect(self._on_nav_safety_halted)

        self.http = QNetworkAccessManager(self)
        self.max_range_cm = 300.0

        if self.state.pose is not None:
            self.robot_x = self.state.pose.x
            self.robot_y = self.state.pose.y
            self.robot_heading = self.state.pose.heading
            self.home_pose = (self.robot_x, self.robot_y)
            self.robot_trail: List[tuple] = [(self.robot_x, self.robot_y)]
        else:
            self.robot_x = 0.0
            self.robot_y = 0.0
            self.robot_heading = 0.0
            self.home_pose = None
            self.robot_trail: List[tuple] = []

        self.sweep_degrees = 180.0
        self.forward_step_cm = 10.0
        self.last_angle = None
        self.last_packet = ""
        self.last_packet_time = 0.0
        self.last_reached = ""
        self.mapping_active = False
        self.sweep_polygons: List[ShPolygon] = []
        self.unified_explored_polygon: Optional[ShPolygon] = None
        self.global_wall_segments: List[tuple] = []
        self.last_wall_segments: List[tuple] = []
        self.map_points: List[tuple] = []
        self.current_sweep_walls: List[tuple] = []
        self.current_sweep_endpoints: List[tuple] = []
        self.sweep_segments_finalized = False
        self.scan_points: List[tuple] = []
        self.scan_points_by_angle = {}
        self.lidar_rays: List[tuple] = []
        self.free_rays: List[tuple] = []
        self.free_rays_by_angle = {}
        self.goal_point = None
        self.route_points: List[tuple] = []
        self._is_panning = False
        self._pan_start = None
        self._pan_limits = None
        self._user_view = False

        self.event_thread = LidarEventThread(host, self)
        self.redraw_timer = QTimer(self)
        self.redraw_timer.setInterval(100)
        self.redraw_timer.setSingleShot(True)
        self.redraw_timer.timeout.connect(self._draw_map)
        self.figure, self.axes = plt.subplots(figsize=(8, 6), facecolor="#1e1e24")
        self.figure.tight_layout(pad=1.0)
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
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

        if hasattr(self.comm, "feedback_received"):
            try:
                self.comm.feedback_received.disconnect(self.handle_udp_message)
            except (TypeError, RuntimeError):
                pass
            self.comm.feedback_received.connect(self.handle_udp_message)
        if hasattr(self.comm, "listener_thread") and self.comm.listener_thread:
            try:
                self.comm.listener_thread.peer_discovered.disconnect(self._on_peer_discovered)
            except (TypeError, RuntimeError):
                pass
            self.comm.listener_thread.peer_discovered.connect(self._on_peer_discovered)

    @property
    def explored_space_polygons(self):
        return self.sweep_polygons

    @explored_space_polygons.setter
    def explored_space_polygons(self, value):
        self.sweep_polygons = value

    def _on_peer_discovered(self, ip_str: str):
        if ip_str and ip_str != self.host:
            self.host = ip_str
            self.event_thread.set_host(ip_str)

    def _build_ui(self):
        layout = QVBoxLayout(self)
        toolbar = QHBoxLayout()

        title = QLabel("LIVE LIDAR MAP")
        title.setStyleSheet("color: #00d2ff; font-weight: bold; font-size: 13px;")
        toolbar.addWidget(title)
        self.status_label = QLabel("Waiting for LiDAR data")
        self.status_label.setFont(QFont("Menlo", 11))
        self.status_label.setStyleSheet("color: #ffcc00;")
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
        sim_button = QPushButton("Simulate Walls")
        sim_button.setObjectName("secondaryBtn")
        sim_button.setToolTip("Inject synthetic room LiDAR scan to verify wall plotting and pathfinding")
        sim_button.clicked.connect(self.simulate_test_walls)
        toolbar.addWidget(sim_button)
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
        layout.addWidget(self.canvas, stretch=1)

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
        self._schedule_draw()

    def _set_lidar_step_angle(self, value: float):
        self.LIDAR_STEP_ANGLE_DEG = float(value)
        self.http.get(QNetworkRequest(QUrl(
            f"http://{self.host}/config?param=lidar_step&val={value:g}"
        )))

    def set_robot_pose(self, x: float, y: float, heading: float):
        moved = False
        if not self.mapping_active:
            if math.hypot(float(x) - self.robot_x, float(y) - self.robot_y) > 0.5:
                moved = True
            self.robot_x = float(x)
            self.robot_y = float(y)
            if self.home_pose is None:
                self.home_pose = (self.robot_x, self.robot_y)
            if not self.robot_trail or math.hypot(self.robot_x - self.robot_trail[-1][0], self.robot_y - self.robot_trail[-1][1]) > 0.5:
                self.robot_trail.append((self.robot_x, self.robot_y))
        if abs(float(heading) - self.robot_heading) > 1.0:
            moved = True
        self.robot_heading = float(heading)
        if moved:
            # Clear only live rays so they re-anchor cleanly to the new robot pose
            self.scan_points_by_angle.clear()
            self.free_rays_by_angle.clear()
            self.lidar_rays.clear()
            self.free_rays.clear()
        self._update_telemetry()
        self._schedule_draw()

    def handle_lidar_packet(self, packet_hex: str):
        if packet_hex.startswith("POSE:"):
            self.handle_udp_message(packet_hex[5:])
            return
        now = time.monotonic()
        if packet_hex == self.last_packet and now - self.last_packet_time < 0.05:
            return
        self.last_packet = packet_hex
        self.last_packet_time = now
        decoded = decode_lidar_packet(packet_hex)
        if decoded is None:
            return
        angle, distance = decoded
        self._process_single_lidar_point(angle, distance)

    def _process_single_lidar_point(self, angle: float, distance: float):
        # Detect new sweep cycle when angle wraps / resets
        if self.last_angle is not None and angle < self.last_angle:
            if not self.sweep_segments_finalized and len(self.current_sweep_walls) >= 2:
                self._complete_wall_polygon()
            self.current_sweep_walls.clear()
            self.current_sweep_endpoints.clear()
            self.lidar_rays.clear()
            self.free_rays.clear()
            self.sweep_segments_finalized = False

        self.last_angle = angle

        half_sweep = self.sweep_degrees / 2.0
        if angle < half_sweep:
            relative_angle = -(angle + self.LIDAR_STEP_ANGLE_DEG)
            scan_heading = self.robot_heading + relative_angle
        elif angle <= self.sweep_degrees:
            relative_angle = (angle - half_sweep)
            scan_heading = self.robot_heading + relative_angle
        else:
            scan_heading = self.robot_heading + angle

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

            # Record for current live sweep
            self.scan_points_by_angle[ray_key] = (distance, endpoint, ray)
            self.free_rays_by_angle.pop(ray_key, None)
            self.current_sweep_walls.append((angle, endpoint, ray))
            self.current_sweep_endpoints.append((angle, endpoint))

            if endpoint not in self.map_points:
                self.map_points.append(endpoint)
        else:
            free_endpoint = (
                sensor_origin[0] + self.max_range_cm * direction_x,
                sensor_origin[1] + self.max_range_cm * direction_y,
            )
            free_ray = (sensor_origin, free_endpoint)
            self.free_rays_by_angle[ray_key] = free_ray
            self.scan_points_by_angle.pop(ray_key, None)
            self.current_sweep_endpoints.append((angle, free_endpoint))

        sweep_end_threshold = max(self.sweep_degrees - self.LIDAR_STEP_ANGLE_DEG - 0.5, 0.0)
        if not self.sweep_segments_finalized and angle >= sweep_end_threshold:
            self._complete_wall_polygon()
            self.sweep_segments_finalized = True
            self.status_label.setText("Sweep complete; occupancy map updated")

        if distance > 0.0:
            self.status_label.setText(
                f"Receiving {self.sweep_degrees:.0f} deg scan, angle {angle:.0f} deg ({distance:.1f} cm)"
            )
            self.status_label.setStyleSheet("color: #00ffaa;")
        else:
            self.status_label.setText(
                f"Receiving {self.sweep_degrees:.0f} deg scan, angle {angle:.0f} deg (0.0 cm: Open space / Out of range)"
            )
            self.status_label.setStyleSheet("color: #ffaa00;")
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
                self.lidar_rays.clear()
                self.sweep_segments_finalized = False
                self.status_label.setText("Physical move reached; mapping pose confirmed")
                self._update_telemetry()
                self._schedule_draw()
        elif message.startswith("ALERT:") and self.mapping_active:
            self.mapping_active = False
            self.scan_button.setText("Start 180 deg / 10 cm")
            self.status_label.setText("Scan stopped by robot safety alert")
            self.status_label.setStyleSheet("color: #ff3366; font-weight: bold;")
        elif message.startswith("LIDAR:") or message.startswith("LIDAR_SCAN:") or message.startswith("SCAN:"):
            points = parse_lidar_message(message)
            for angle, distance in points:
                self._process_single_lidar_point(angle, distance)

    def toggle_scan(self):
        if self.scan_button.text().startswith("Stop"):
            self.comm.send_command("STOP")
            try:
                self.http.get(QNetworkRequest(QUrl(f"http://{self.host}/lidar?cmd=stop")))
            except Exception:
                pass
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
        try:
            url = f"http://{self.host}/lidar?cmd=continuous&angle={int(self.sweep_degrees)}&distance={self.forward_step_cm:g}"
            self.http.get(QNetworkRequest(QUrl(url)))
        except Exception:
            pass
        self.scan_button.setText("Stop Continuous Scan")
        self.status_label.setText(
            f"Starting {self.sweep_degrees:.0f} deg scan / "
            f"{self.forward_step_cm:.1f} cm step"
        )

    def emergency_stop(self):
        """Immediately stop physical motion and cancel all LiDAR navigation."""
        if self.executor.is_running:
            self.executor.stop()
        self.controller.stop()
        self.comm.send_command("STOP")
        try:
            self.http.get(QNetworkRequest(QUrl(f"http://{self.host}/stop")))
        except Exception:
            pass
        self.mapping_active = False
        self.route_points.clear()
        self.scan_button.setText("Start 180 deg / 10 cm")
        self.status_label.setText("EMERGENCY STOP SENT")
        self.status_label.setStyleSheet("color: #ff3366; font-weight: bold;")
        self._schedule_draw()

    def _plan_route_to_goal(self):
        if self.goal_point is None:
            self.status_label.setText("Click the map to select a goal point first")
            return
        if self.executor.is_running:
            self.status_label.setText("A route is already executing")
            return

        curr_x = self.state.x if (self.state and self.state.pose) else self.robot_x
        curr_y = self.state.y if (self.state and self.state.pose) else self.robot_y
        curr_h = self.state.heading if (self.state and self.state.pose) else self.robot_heading

        start = (curr_x, curr_y)
        goal = self.goal_point
        if math.hypot(goal[0] - start[0], goal[1] - start[1]) < 1.0:
            self.status_label.setText("Goal is already reached")
            return

        if self.mapping_active:
            self.comm.send_command("STOP")
            self.mapping_active = False
            self.scan_button.setText("Start 180 deg / 10 cm")

        route = self._plan_path(start, goal)
        if not route or len(route) < 2:
            self.status_label.setText("No safe route found to selected goal")
            return

        self.route_points = route
        commands = self._route_to_commands(route, curr_h)
        if not commands:
            self.status_label.setText("Goal is already reached")
            return

        self.status_label.setText(f"Navigating to goal ({len(commands)} steps)...")
        self.status_label.setStyleSheet("color: #00d2ff;")
        self._schedule_draw()

        self.executor.set_commands(commands)
        self.executor.start()

    def _plan_path(self, start, goal):
        obstacle_segments = list(self.last_wall_segments)
        if not obstacle_segments:
            if self.map_points:
                step = max(1, len(self.map_points) // 60)
                subsampled = self.map_points[::step]
                obstacle_segments = [(pt, pt) for pt in subsampled]
            else:
                return [start, goal]

        nominal_clearance = math.hypot(
            self.ROBOT_LENGTH_CM / 2.0,
            self.ROBOT_WIDTH_CM / 2.0,
        ) + 2.0

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

        start_dist = min(point_segment_distance(start, f, s) for f, s in obstacle_segments)
        goal_dist = min(point_segment_distance(goal, f, s) for f, s in obstacle_segments)
        min_endpoint_dist = min(start_dist, goal_dist)
        if min_endpoint_dist <= nominal_clearance:
            clearance = max(1.5, min_endpoint_dist - 0.5)
        else:
            clearance = nominal_clearance

        def is_blocked(point):
            return any(
                point_segment_distance(point, first, second) <= clearance
                for first, second in obstacle_segments
            )

        def line_is_clear(first, second):
            sample_count = max(2, int(math.ceil(math.dist(first, second) / 1.0)))
            for index in range(sample_count + 1):
                factor = index / sample_count
                sample = (
                    first[0] + (second[0] - first[0]) * factor,
                    first[1] + (second[1] - first[1]) * factor,
                )
                if is_blocked(sample):
                    return False
            return True

        if line_is_clear(start, goal):
            return [start, goal]

        return self._astar_route(start, goal, obstacle_segments, clearance, line_is_clear)

    def _astar_route(self, start, goal, obstacle_segments, clearance, line_is_clear):
        grid = 4.0
        all_obstacle_points = [point for segment in obstacle_segments for point in segment]
        all_points = [start, goal] + all_obstacle_points
        min_x = min(point[0] for point in all_points) - 20.0
        max_x = max(point[0] for point in all_points) + 20.0
        min_y = min(point[1] for point in all_points) - 20.0
        max_y = max(point[1] for point in all_points) + 20.0

        def to_cell(point):
            return round((point[0] - min_x) / grid), round((point[1] - min_y) / grid)

        def to_world(cell):
            return min_x + cell[0] * grid, min_y + cell[1] * grid

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

        start_cell = to_cell(start)
        goal_cell = to_cell(goal)
        blocked_cache = {}

        def is_blocked(cell):
            if cell in blocked_cache:
                return blocked_cache[cell]
            if cell == start_cell or cell == goal_cell:
                blocked_cache[cell] = False
                return False
            pt = to_world(cell)
            result = any(
                point_segment_distance(pt, first, second) <= clearance
                for first, second in obstacle_segments
            )
            blocked_cache[cell] = result
            return result

        max_cell_x = int(math.ceil((max_x - min_x) / grid))
        max_cell_y = int(math.ceil((max_y - min_y) / grid))

        frontier = [(0.0, start_cell)]
        came_from = {start_cell: None}
        cost = {start_cell: 0.0}
        max_iterations = 6000
        iterations = 0

        while frontier and iterations < max_iterations:
            iterations += 1
            _, current = heapq.heappop(frontier)
            if current == goal_cell:
                break
            for step_x, step_y in (
                (1, 0), (-1, 0), (0, 1), (0, -1),
                (1, 1), (1, -1), (-1, 1), (-1, -1),
            ):
                neighbor = (current[0] + step_x, current[1] + step_y)
                if not (0 <= neighbor[0] <= max_cell_x and 0 <= neighbor[1] <= max_cell_y):
                    continue
                if is_blocked(neighbor):
                    continue
                if step_x and step_y:
                    if is_blocked((current[0] + step_x, current[1])) or is_blocked((current[0], current[1] + step_y)):
                        continue
                step_cost = math.hypot(step_x, step_y) * grid
                new_cost = cost[current] + step_cost
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

        raw_route = [to_world(cell) for cell in cells]
        raw_route[0] = start
        raw_route[-1] = goal

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

    def _route_to_commands(self, route: List[tuple], start_heading: float) -> List[Command]:
        """Convert consecutive route waypoints into discrete TURN and MOVE Command objects.
        Adopts the exact angular logic and coordinate convention from Mission Path Planning (models/path.py).
        """
        commands = []
        if len(route) < 2:
            return commands

        current_heading = start_heading
        for p1, p2 in zip(route, route[1:]):
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            dist = math.hypot(dx, dy)

            if dist < 0.1:
                continue

            target_heading = math.degrees(math.atan2(dy, dx))
            turn = normalize_angle_deg(target_heading - current_heading)

            if abs(turn) > 0.5:
                commands.append(Command(type="TURN", value=turn))
                current_heading = target_heading

            commands.append(Command(type="MOVE", value=dist, direction=1.0))

        return commands

    def _on_nav_step_started(self, curr: int, total: int, desc: str):
        self.status_label.setText(f"Navigating: {desc} (Step {curr}/{total})")
        self.status_label.setStyleSheet("color: #00d2ff; font-weight: bold;")

    def _on_nav_step_completed(self, curr: int, total: int):
        self._update_telemetry()
        self._schedule_draw()

    def _on_nav_completed(self):
        self.status_label.setText("Goal reached successfully!")
        self.status_label.setStyleSheet("color: #00ffaa; font-weight: bold;")
        self.route_points.clear()
        self.goal_point = None
        self._update_telemetry()
        self._schedule_draw()

    def _on_nav_stopped(self):
        self.status_label.setText("Navigation stopped")
        self.status_label.setStyleSheet("color: #ffcc00;")
        self.route_points.clear()
        self._schedule_draw()

    def _on_nav_safety_halted(self):
        self.status_label.setText("Route stopped by robot safety alert")
        self.status_label.setStyleSheet("color: #ff3366; font-weight: bold;")
        self.route_points.clear()
        self._schedule_draw()

    def _update_telemetry(self):
        wall_count = len(self.global_wall_segments) if self.global_wall_segments else len(self.last_wall_segments)
        self.telemetry_label.setText(
            f"Global map: {len(self.map_points)} obstacle cells | "
            f"Walls: {wall_count} segments | "
            f"Pose: ({self.robot_x:.1f}, {self.robot_y:.1f}) cm | "
            f"Heading: {self.robot_heading:.1f} deg"
        )

    def _schedule_draw(self):
        """Coalesce rapid LiDAR packets into the next timer redraw."""
        if not self.redraw_timer.isActive():
            self.redraw_timer.start()

    def _on_connection_changed(self, connected: bool, message: str):
        if not connected and not self.map_points:
            self.status_label.setText(message)
            self.status_label.setStyleSheet("color: #ffcc00;")

    def clear_scan(self):
        self.sweep_polygons.clear()
        self.unified_explored_polygon = None
        self.global_wall_segments.clear()
        self.last_wall_segments.clear()
        self.map_points.clear()
        self.current_sweep_walls.clear()
        self.current_sweep_endpoints.clear()
        self.lidar_rays.clear()
        self.scan_points_by_angle.clear()
        self.free_rays.clear()
        self.free_rays_by_angle.clear()
        self.sweep_segments_finalized = False
        self.robot_trail.clear()
        if self.home_pose:
            self.robot_trail.append((self.robot_x, self.robot_y))
        self.goal_point = None
        self.route_points.clear()
        if hasattr(self, "executor") and self.executor.is_running:
            self.executor.stop()
        self.last_angle = None
        self.last_packet = ""
        self.last_packet_time = 0.0
        self.last_reached = ""
        self.status_label.setText("Waiting for LiDAR data")
        self.status_label.setStyleSheet("color: #ffcc00;")
        self._update_telemetry()
        self._draw_map()

    def _complete_wall_polygon(self):
        """Extract unified free-space floor and single outer wall loop from sweeps."""
        walls = sorted(self.current_sweep_walls, key=lambda item: item[0])
        endpoints = sorted(self.current_sweep_endpoints, key=lambda item: item[0])
        sensor_offset = self.ROBOT_LENGTH_CM / 2.0
        robot_heading_radians = math.radians(self.robot_heading)
        sensor_origin = (
            self.robot_x + sensor_offset * math.cos(robot_heading_radians),
            self.robot_y + sensor_offset * math.sin(robot_heading_radians),
        )

        if len(endpoints) >= 3:
            pts = [pt for _, pt in endpoints]
            if self.sweep_degrees >= 350.0:
                if pts[0] != pts[-1]:
                    pts.append(pts[0])
                poly_verts = pts
            else:
                poly_verts = [sensor_origin] + pts + [sensor_origin]

            try:
                sweep_poly = ShPolygon(poly_verts)
                if not sweep_poly.is_valid:
                    sweep_poly = sweep_poly.buffer(0)

                if sweep_poly.is_valid and sweep_poly.area > 1.0:
                    self.sweep_polygons.append(sweep_poly)
                    if len(self.sweep_polygons) > 50:
                        self.sweep_polygons = [unary_union(self.sweep_polygons)]

                    self.unified_explored_polygon = unary_union(self.sweep_polygons)
                    if hasattr(self.unified_explored_polygon, "buffer"):
                        self.unified_explored_polygon = self.unified_explored_polygon.buffer(0)

                    new_wall_segments = []
                    if isinstance(self.unified_explored_polygon, ShPolygon):
                        polys = [self.unified_explored_polygon]
                    elif isinstance(self.unified_explored_polygon, MultiPolygon):
                        polys = list(self.unified_explored_polygon.geoms)
                    else:
                        polys = []

                    for p in polys:
                        coords = list(p.exterior.coords)
                        for i in range(len(coords) - 1):
                            new_wall_segments.append((coords[i], coords[i+1]))
                        for interior in p.interiors:
                            icoords = list(interior.coords)
                            for i in range(len(icoords) - 1):
                                new_wall_segments.append((icoords[i], icoords[i+1]))

                    self.global_wall_segments = new_wall_segments
                    self.last_wall_segments = new_wall_segments
            except Exception:
                pass
        elif len(walls) >= 2:
            wall_points = [item[1] for item in walls]
            new_segs = []
            for start, end in zip(wall_points, wall_points[1:]):
                if math.dist(start, end) <= 65.0:
                    new_segs.append((start, end))
            self.global_wall_segments = new_segs
            self.last_wall_segments = new_segs
            self.map_points = list(set(self.map_points + wall_points))

        self.current_sweep_walls.clear()
        self.current_sweep_endpoints.clear()

    def simulate_test_walls(self):
        """Inject synthetic LiDAR scan of an enclosed room to verify wall plotting and pathfinding."""
        self.clear_scan()
        self.status_label.setText("Simulating test room LiDAR scan...")
        self.status_label.setStyleSheet("color: #00d2ff;")

        # Simulate an enclosed 140 cm x 140 cm room centered at (0, 0) (walls at +/-70 cm)
        sensor_origin = (self.robot_x, self.robot_y)
        for deg in range(0, 360, 4):
            rad = math.radians(float(deg))
            cos_a = math.cos(rad)
            sin_a = math.sin(rad)
            dists = []
            if cos_a > 1e-4:
                dists.append(70.0 / cos_a)
            elif cos_a < -1e-4:
                dists.append(-70.0 / cos_a)
            if sin_a > 1e-4:
                dists.append(70.0 / sin_a)
            elif sin_a < -1e-4:
                dists.append(-70.0 / sin_a)
            valid = [d for d in dists if d > 0]
            if valid:
                dist = min(valid)
                endpoint = (sensor_origin[0] + dist * cos_a, sensor_origin[1] + dist * sin_a)
                ray = (sensor_origin, endpoint)

                self.lidar_rays.append(ray)
                self.scan_points_by_angle[round(float(deg), 2)] = (dist, endpoint, ray)
                self.current_sweep_walls.append((float(deg), endpoint, ray))
                self.current_sweep_endpoints.append((float(deg), endpoint))
                self.map_points.append(endpoint)

        self.sweep_degrees = 360.0
        self._complete_wall_polygon()
        self.sweep_segments_finalized = True
        self.status_label.setText("Simulated room plotted! Click anywhere on map to set Goal.")
        self.status_label.setStyleSheet("color: #00ffaa; font-weight: bold;")
        self._update_telemetry()
        self._draw_map()

    def fit_view(self):
        """Fit the viewport to all map points and return to automatic tracking."""
        self._user_view = False
        self._schedule_draw()

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
        self._schedule_draw()

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
        self._schedule_draw()

    def _draw_map(self):
        try:
            saved_xlim = self.axes.get_xlim() if self._user_view else None
            saved_ylim = self.axes.get_ylim() if self._user_view else None
            self.axes.clear()

            # ROS Occupancy Grid background: medium gray (unknown space)
            self.axes.set_facecolor("#787c82")
            self.axes.set_aspect("equal", adjustable="box")

            if not self._user_view:
                all_pts = []
                if self.map_points:
                    all_pts.extend(self.map_points)
                if self.robot_trail:
                    all_pts.extend(self.robot_trail)
                all_pts.append((self.robot_x, self.robot_y))

                min_x = min(pt[0] for pt in all_pts)
                max_x = max(pt[0] for pt in all_pts)
                min_y = min(pt[1] for pt in all_pts)
                max_y = max(pt[1] for pt in all_pts)

                span_x = max_x - min_x
                span_y = max_y - min_y
                half_range = max(self.max_range_cm, span_x / 2.0 + 35.0, span_y / 2.0 + 35.0)
                center_x = (min_x + max_x) / 2.0
                center_y = (min_y + max_y) / 2.0
                self.axes.set_xlim(center_x - half_range, center_x + half_range)
                self.axes.set_ylim(center_y - half_range, center_y + half_range)
            else:
                self.axes.set_xlim(saved_xlim)
                self.axes.set_ylim(saved_ylim)

            self.axes.set_xlabel("World X (cm)", color="#c5c8d4", fontsize=9)
            self.axes.set_ylabel("World Y (cm)", color="#c5c8d4", fontsize=9)
            self.axes.tick_params(colors="#c5c8d4", labelsize=8)
            self.axes.grid(color="#666a70", linestyle=":", linewidth=0.7)

            # 1. Explored Free-Space Floor (pure solid WHITE, like in ROS SLAM)
            if self.unified_explored_polygon:
                if isinstance(self.unified_explored_polygon, ShPolygon):
                    polys = [self.unified_explored_polygon]
                elif isinstance(self.unified_explored_polygon, MultiPolygon):
                    polys = list(self.unified_explored_polygon.geoms)
                else:
                    polys = []

                for poly in polys:
                    # White interior floor
                    self.axes.add_patch(MplPolygon(
                        list(poly.exterior.coords), closed=True,
                        facecolor="#ffffff", edgecolor="none", zorder=2,
                    ))
                    # Single outer wall loop (crisp solid BLACK boundary line, like in ROS SLAM)
                    xs, ys = zip(*list(poly.exterior.coords))
                    self.axes.plot(
                        xs, ys, color="#111111", linewidth=3.0,
                        zorder=4, label="Mapped wall loop",
                    )
                    # Interior holes / pillars (solid black)
                    for interior in poly.interiors:
                        ixs, iys = zip(*list(interior.coords))
                        self.axes.fill(
                            ixs, iys, facecolor="#111111", edgecolor="#111111",
                            linewidth=2.5, zorder=4,
                        )
            elif self.last_wall_segments:
                # Fallback for early points before closed polygon forms
                self.axes.add_collection(LineCollection(
                    self.last_wall_segments,
                    colors="#111111",
                    linewidths=3.0,
                    zorder=4,
                    label="Mapped wall loop",
                ))

            # 2. Live open-space rays (beyond max range / clear line of sight)
            free_rays = list(self.free_rays_by_angle.values()) if self.free_rays_by_angle else self.free_rays
            if free_rays:
                self.axes.add_collection(LineCollection(
                    free_rays[-360:],
                    colors="#d0d0d0",
                    linewidths=0.5,
                    alpha=0.35,
                    zorder=1,
                    label="Clear line-of-sight",
                ))

            # 3. Live active laser beams to obstacles
            live_rays = [v[2] for v in self.scan_points_by_angle.values()] if self.scan_points_by_angle else self.lidar_rays
            if live_rays:
                self.axes.add_collection(LineCollection(
                    live_rays[-360:],
                    colors="#00acc1",
                    linewidths=0.75,
                    alpha=0.60,
                    zorder=5,
                    label="Live laser beams",
                ))

            # 4. Live Scanner Reflections (active sweep hit endpoints)
            if self.scan_points_by_angle:
                live_pts = [v[1] for v in self.scan_points_by_angle.values()]
                if live_pts:
                    l_xs, l_ys = zip(*live_pts)
                    self.axes.scatter(
                        l_xs, l_ys, s=20, color="#00e5ff", edgecolors="#000000",
                        linewidths=0.5, alpha=0.95, zorder=6, label="Live scan hits",
                    )

            # 5. Robot Trajectory Trail (solid blue line with waypoints, like in ROS SLAM)
            if len(self.robot_trail) > 1:
                trail_x, trail_y = zip(*self.robot_trail)
                self.axes.plot(
                    trail_x, trail_y, color="#1565c0", linewidth=2.2,
                    linestyle="-", marker="o", markersize=3.5, zorder=7, label="Robot trajectory",
                )

            # 6. Planned Route to Goal
            if self.route_points and len(self.route_points) > 1:
                route_x, route_y = zip(*self.route_points)
                self.axes.plot(
                    route_x, route_y, color="#2e7d32", linewidth=2.2,
                    linestyle="--", alpha=0.95, zorder=7, label="Planned route",
                )

            # 7. Home & Goal Markers
            if self.home_pose:
                self.axes.scatter(
                    [self.home_pose[0]], [self.home_pose[1]],
                    marker="*", s=140, color="#00c853", edgecolors="#000000",
                    linewidths=1.0, zorder=8, label="Home",
                )
            if self.goal_point:
                self.axes.scatter(
                    [self.goal_point[0]], [self.goal_point[1]],
                    marker="X", s=120, color="#ff6d00", edgecolors="#000000",
                    linewidths=1.0, zorder=8, label="Goal",
                )

            # 8. Robot Chassis Footprint & Heading Arrow
            half_length = self.ROBOT_LENGTH_CM / 2.0
            half_width = self.ROBOT_WIDTH_CM / 2.0
            heading = math.radians(self.robot_heading)
            cos_h = math.cos(heading)
            sin_h = math.sin(heading)
            robot_corners = [
                (self.robot_x + cos_h * half_length - sin_h * half_width,
                 self.robot_y + sin_h * half_length + cos_h * half_width),
                (self.robot_x + cos_h * half_length + sin_h * half_width,
                 self.robot_y + sin_h * half_length - cos_h * half_width),
                (self.robot_x - cos_h * half_length + sin_h * half_width,
                 self.robot_y - sin_h * half_length - cos_h * half_width),
                (self.robot_x - cos_h * half_length - sin_h * half_width,
                 self.robot_y - sin_h * half_length + cos_h * half_width),
            ]
            self.axes.add_patch(MplPolygon(
                robot_corners, closed=True, facecolor="#ffd700",
                edgecolor="#000000", linewidth=1.2, alpha=0.95,
                label="Robot 11 x 9 cm", zorder=8,
            ))
            self.axes.arrow(
                self.robot_x, self.robot_y, 10.0 * cos_h, 10.0 * sin_h,
                color="#000000", width=0.5, head_width=3.5,
                length_includes_head=True, zorder=9,
            )

            handles, labels = self.axes.get_legend_handles_labels()
            if handles:
                by_label = dict(zip(labels, handles))
                self.axes.legend(
                    by_label.values(), by_label.keys(),
                    loc="upper right", facecolor="#252932",
                    edgecolor="#3a3f4d", labelcolor="#e0e0e0", fontsize=8,
                )

            self.canvas.draw_idle()
        except Exception:
            pass

    def showEvent(self, event):
        super().showEvent(event)
        try:
            self.figure.tight_layout(pad=1.0)
        except Exception:
            pass
        self._schedule_draw()

    def close(self):
        self.redraw_timer.stop()
        if self.event_thread.isRunning():
            self.event_thread.stop()
            self.event_thread.quit()
            if not self.event_thread.wait(1000):
                self.event_thread.terminate()
                self.event_thread.wait(500)
