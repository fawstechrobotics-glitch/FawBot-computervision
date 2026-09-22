"""Live VL53L0X continuous LiDAR mapping page."""
import math
import time
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
        self.map_points: List[tuple] = []
        self.lidar_rays: List[tuple] = []
        self.robot_trail: List[tuple] = []
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

    def _set_max_range(self, value: float):
        self.max_range_cm = value
        self._draw_map()

    def _set_lidar_step_angle(self, value: float):
        self.LIDAR_STEP_ANGLE_DEG = float(value)
        self.http.get(QNetworkRequest(QUrl(
            f"http://{self.host}/config?param=lidar_step&val={value:g}"
        )))

    def set_robot_pose(self, x: float, y: float, heading: float):
        if not self.mapping_active:
            self.robot_x = float(x)
            self.robot_y = float(y)
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

        if 0.0 < distance <= self.max_range_cm:
            half_sweep = self.sweep_degrees / 2.0
            if angle < half_sweep:
                # During the first firmware phase the robot turns -1 deg
                # before each reading, so packet angles run opposite to the
                # physical clockwise scan direction.
                scan_heading = self.robot_heading - (
                    angle + self.LIDAR_STEP_ANGLE_DEG
                )
            else:
                # The second phase turns +5 deg and already matches the map
                # angle direction.
                scan_heading = self.robot_heading + (angle - half_sweep)
            world_angle = math.radians(scan_heading)
            direction_x = math.cos(world_angle)
            direction_y = math.sin(world_angle)
            sensor_offset = self.ROBOT_LENGTH_CM / 2.0
            sensor_origin = (
                self.robot_x + sensor_offset * direction_x,
                self.robot_y + sensor_offset * direction_y,
            )
            endpoint = (
                sensor_origin[0] + distance * direction_x,
                sensor_origin[1] + distance * direction_y,
            )
            origin = sensor_origin
            self.map_points.append(endpoint)
            self.lidar_rays.append((origin, endpoint))

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
                self.status_label.setText("Physical move reached; mapping pose confirmed")
                self._update_telemetry()
                self._schedule_draw()

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
        self.robot_trail.clear()
        self.last_angle = None
        self.last_packet = ""
        self.last_packet_time = 0.0
        self.last_reached = ""
        self.status_label.setText("Waiting for LiDAR data")
        self.status_label.setStyleSheet("color: #ffcc00; font-family: monospace;")
        self._update_telemetry()
        self._draw_map()

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

        if len(self.robot_trail) > 1:
            trail_x, trail_y = zip(*self.robot_trail)
            self.axes.plot(trail_x, trail_y, color="#b0b0b0", linewidth=2.5,
                           linestyle="-", alpha=0.95, label="Clear path")

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
