"""Reusable Matplotlib-based Map & CAD Grid Widget."""
import math
from typing import Dict, List, Tuple, Optional
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtCore import pyqtSignal, Qt
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.transforms as mtransforms
from matplotlib.patches import PathPatch, Polygon
from matplotlib.path import Path as MplPath

import config.settings as settings
from models.obstacle import Boundary, RestrictedArea
from navigation.geometry import compute_robot_footprint

plt.style.use('dark_background')


class MapWidget(QWidget):
    """High-performance CAD grid and robot visualization viewport."""
    ROBOT_COLORS = ['#ffd700', '#00d2ff', '#ff66aa', '#7dff6b', '#c084fc', '#ff9966']
    grid_clicked = pyqtSignal(float, float)                     # Clicked (x, y) coordinates
    mouse_moved = pyqtSignal(float, float)                      # Live hover (x, y) coordinates
    grid_step_changed = pyqtSignal(float)                       # Current dynamic CAD step

    def __init__(self, parent=None):
        super().__init__(parent)

        # Plot setup
        self.fig, self.ax = plt.subplots(figsize=(6, 6), facecolor='#121212')
        self.fig.tight_layout(pad=1.0)
        self.canvas = FigureCanvas(self.fig)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.canvas)

        # State storage for rendering
        self.robot_pos: Optional[Tuple[float, float]] = None
        self.robot_angle: float = 0.0
        self.robot_poses: Dict[str, Tuple[float, float, float]] = {}
        self.initial_pos: Optional[Tuple[float, float]] = None
        self.current_target: Optional[Tuple[float, float]] = None
        self.waypoint_queue: List[Tuple[float, float]] = []
        self.robot_waypoints: Dict[str, List[Tuple[float, float]]] = {}
        self.path_history: List[List[Tuple[float, float]]] = []
        self.robot_path_histories: Dict[str, List[List[Tuple[float, float]]]] = {}

        # Mission planning overlays
        self.boundary: Boundary = Boundary()
        self.restricted_areas: List[RestrictedArea] = []
        self.planned_path_points: List[Tuple[float, float]] = []
        self.ghost_pose: Optional[Tuple[float, float, float]] = None  # (x, y, heading) for preview
        self.show_footprint_margin: bool = True

        # Mouse & Pan/Zoom state
        self.mouse_x: Optional[float] = None
        self.mouse_y: Optional[float] = None
        self.is_panning = False
        self.pan_start_x = 0
        self.pan_start_y = 0
        self.pan_orig_xlim = (-10.0, settings.ENV_WIDTH_CM + 10.0)
        self.pan_orig_ylim = (-10.0, settings.ENV_HEIGHT_CM + 10.0)
        self.current_grid_cm = settings.INITIAL_GRID_STEP_CM

        # Connect canvas events
        self.canvas.mpl_connect('button_press_event', self._on_mouse_press)
        self.canvas.mpl_connect('button_release_event', self._on_mouse_release)
        self.canvas.mpl_connect('motion_notify_event', self._on_mouse_move)
        self.canvas.mpl_connect('scroll_event', self._on_mouse_wheel_zoom)

        self.reset_view()

    def create_custom_robot_patch(self, length: float, width: float) -> PathPatch:
        """Constructs robot chassis matching the physical plate: rectangular back with rounded front arc."""
        rect_length = length * 0.65
        arc_radius = width / 2.0
        r_x = rect_length / 2.0
        half_w = width / 2.0
        l_x = -length / 2.0

        vertices = [
            (l_x, -half_w),
            (r_x, -half_w),
            (r_x + arc_radius, 0.0),
            (r_x, half_w),
            (l_x, half_w),
            (l_x, -half_w)
        ]
        codes = [
            MplPath.MOVETO,
            MplPath.LINETO,
            MplPath.CURVE3,
            MplPath.CURVE3,
            MplPath.LINETO,
            MplPath.CLOSEPOLY
        ]
        return PathPatch(
            MplPath(vertices, codes),
            facecolor='#ffd700',
            edgecolor='#ffaa00',
            linewidth=1.5,
            alpha=0.9,
            zorder=8,
            label="Robot Chassis"
        )

    def calculate_cad_grid_step(self, visible_range_cm: float) -> float:
        """Dynamically compute standard engineering grid subdivisions (1, 2, 5, 10)."""
        target_lines = 15.0
        raw_step = visible_range_cm / target_lines
        if raw_step <= 0:
            return 1.0

        exponent = math.floor(math.log10(raw_step))
        fraction = raw_step / (10 ** exponent)

        if fraction < 1.5:
            nice_fraction = 1.0
        elif fraction < 3.5:
            nice_fraction = 2.0
        elif fraction < 7.5:
            nice_fraction = 5.0
        else:
            nice_fraction = 10.0

        return nice_fraction * (10 ** exponent)

    def draw_grid(self):
        """Redraw all viewport layers: grid, boundary, obstacles, path, robot, telemetry overlays."""
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()

        visible_width = abs(xlim[1] - xlim[0])
        self.current_grid_cm = self.calculate_cad_grid_step(visible_width)
        self.grid_step_changed.emit(self.current_grid_cm)

        self.ax.clear()
        self.ax.set_facecolor('#121212')

        grid_step = self.current_grid_cm
        major_step = grid_step * 5.0

        x_start = math.floor(xlim[0] / grid_step) * grid_step
        x_end = math.ceil(xlim[1] / grid_step) * grid_step
        y_start = math.floor(ylim[0] / grid_step) * grid_step
        y_end = math.ceil(ylim[1] / grid_step) * grid_step

        minor_xticks = []
        curr = x_start
        while curr <= x_end:
            minor_xticks.append(curr)
            curr += grid_step

        minor_yticks = []
        curr = y_start
        while curr <= y_end:
            minor_yticks.append(curr)
            curr += grid_step

        major_xticks = [x for x in minor_xticks if abs(x % major_step) < 1e-5]
        major_yticks = [y for y in minor_yticks if abs(y % major_step) < 1e-5]

        self.ax.set_xticks(minor_xticks, minor=True)
        self.ax.set_yticks(minor_yticks, minor=True)
        self.ax.set_xticks(major_xticks)
        self.ax.set_yticks(major_yticks)

        self.ax.grid(which='minor', color='#242424', linestyle=':', linewidth=0.5)
        self.ax.grid(which='major', color='#3a3a3a', linestyle='-', linewidth=0.8)
        self.ax.set_xlabel("X Position (cm)", color='#888', fontsize=9)
        self.ax.set_ylabel("Y Position (cm)", color='#888', fontsize=9)

        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)

        # 1. WORLD ORIGIN (0,0)
        self.ax.axhline(0, color='#ff6600', linewidth=1.0, alpha=0.5)
        self.ax.axvline(0, color='#ff6600', linewidth=1.0, alpha=0.5)
        self.ax.plot(0, 0, marker='o', markersize=7, color='#ff6600', markeredgecolor='white', markeredgewidth=1.2, zorder=10, label="Origin (0,0)")

        # 2. ENVIRONMENT BOUNDARY (only if user explicitly enabled it)
        if self.boundary and self.boundary.enabled and self.boundary.polygon:
            b_poly = self.boundary.polygon
            closed_b = b_poly + [b_poly[0]]
            bx, by = zip(*closed_b)
            self.ax.plot(bx, by, color='#00d2ff', linestyle='--', linewidth=1.8, alpha=0.8, zorder=3, label="Boundary")

        # 3. RESTRICTED / NO-GO AREAS
        for ra in self.restricted_areas:
            if ra.is_valid_polygon:
                ra_color = ra.color if ra.active else '#555555'
                patch = Polygon(
                    ra.polygon,
                    closed=True,
                    facecolor=ra_color,
                    edgecolor='#ff0055' if ra.active else '#666666',
                    linewidth=1.5,
                    alpha=0.35 if ra.active else 0.15,
                    hatch='//',
                    zorder=4
                )
                self.ax.add_patch(patch)
                # Compute centroid for label
                cx = sum(p[0] for p in ra.polygon) / len(ra.polygon)
                cy = sum(p[1] for p in ra.polygon) / len(ra.polygon)
                self.ax.text(cx, cy, ra.name, color='#ff6688' if ra.active else '#888', fontsize=8, fontweight='bold', ha='center', va='center', zorder=5)

        # 4. INITIAL START HOME MARKER
        if self.initial_pos:
            self.ax.plot(self.initial_pos[0], self.initial_pos[1], marker='*', markersize=11, color='#ffaa00', markeredgecolor='white', zorder=11, label="Start Home")

        # 5. CONTINUOUS PATH TRAILS
        for stroke in self.path_history:
            if len(stroke) > 1:
                xs, ys = zip(*stroke)
                self.ax.plot(xs, ys, color='#00d2ff', linewidth=2.5, alpha=0.85, zorder=6)
        trail_colors = ['#ffd700', '#00d2ff', '#ff66aa', '#7dff6b', '#c084fc', '#ff9966']
        for index, (robot_id, histories) in enumerate(self.robot_path_histories.items()):
            for stroke in histories:
                if len(stroke) > 1:
                    xs, ys = zip(*stroke)
                    self.ax.plot(
                        xs, ys,
                        color=trail_colors[index % len(trail_colors)],
                        linewidth=2.5,
                        alpha=0.85,
                        zorder=6,
                        label=f"{robot_id} trail"
                    )

        # 6. PLANNED / LOADED MISSION PATH
        if self.planned_path_points and len(self.planned_path_points) > 1:
            pxs, pys = zip(*self.planned_path_points)
            self.ax.plot(pxs, pys, color='#bd00ff', linestyle=':', linewidth=2.0, alpha=0.9, zorder=6, label="Mission Path")

        # 7. QUEUED WAYPOINTS
        if self.waypoint_queue:
            q_xs, q_ys = zip(*self.waypoint_queue)
            self.ax.plot(q_xs, q_ys, 'o--', color='#ffaa00', linewidth=1.5, markersize=7, markeredgecolor='white', zorder=11, label="Queued Path")
            for idx, (wpt_x, wpt_y) in enumerate(self.waypoint_queue, start=1):
                self.ax.text(wpt_x + 1.5, wpt_y + 1.5, f"P{idx}", color='#ffaa00', fontsize=8, fontweight='bold', zorder=12)
        for index, (robot_id, waypoints) in enumerate(self.robot_waypoints.items()):
            if not waypoints:
                continue
            color = self._robot_color(robot_id, index)
            if len(waypoints) > 1:
                waypoint_xs, waypoint_ys = zip(*waypoints)
                self.ax.plot(
                    waypoint_xs,
                    waypoint_ys,
                    'o--',
                    color=color,
                    linewidth=1.5,
                    markersize=7,
                    markeredgecolor='white',
                    zorder=11,
                    label=f"{robot_id} waypoints"
                )
            else:
                self.ax.plot(
                    waypoints[0][0], waypoints[0][1],
                    'o', color=color, markersize=7,
                    markeredgecolor='white', zorder=11,
                    label=f"{robot_id} waypoint"
                )
            for waypoint_index, (wpt_x, wpt_y) in enumerate(waypoints, start=1):
                self.ax.text(
                    wpt_x + 1.5, wpt_y + 1.5,
                    f"{robot_id} P{waypoint_index}",
                    color=color, fontsize=8, fontweight='bold', zorder=12
                )

        # 8. ACTIVE TARGET
        if self.current_target:
            tx, ty = self.current_target
            self.ax.plot(tx, ty, 'gx', markersize=12, markeredgewidth=2.5, zorder=12, label="Active Target")
            self.ax.add_patch(plt.Circle((tx, ty), 2.5, color='#00ffaa', fill=False, linestyle='--', zorder=12))

        # 9. CURRENT LEG TRAJECTORY
        if self.robot_pos and self.current_target:
            self.ax.plot([self.robot_pos[0], self.current_target[0]],
                         [self.robot_pos[1], self.current_target[1]],
                         '#ff0055', linestyle='--', linewidth=1.5, zorder=7)

        # 10. GHOST PREVIEW POSE (FOR DRY-RUN PREVIEW)
        if self.ghost_pose:
            gx, gy, gh = self.ghost_pose
            ghost_fp = compute_robot_footprint(gx, gy, gh, settings.ROBOT_LENGTH_CM, settings.ROBOT_WIDTH_CM, 0.0)
            ghost_patch = Polygon(ghost_fp, closed=True, facecolor='#bd00ff', edgecolor='#ff00ff', alpha=0.5, linewidth=1.2, zorder=8)
            self.ax.add_patch(ghost_patch)

        # 11. ROBOT CHASSIS, SAFETY MARGIN FOOTPRINT, AND ORIENTATION ARROW
        for index, (robot_id, (rx, ry, robot_angle)) in enumerate(self.robot_poses.items()):
            color = self._robot_color(robot_id, index)
            if self.show_footprint_margin:
                fp_margin = compute_robot_footprint(
                    rx, ry, robot_angle,
                    settings.ROBOT_LENGTH_CM, settings.ROBOT_WIDTH_CM,
                    settings.ROBOT_SAFETY_MARGIN_CM
                )
                margin_patch = Polygon(
                    fp_margin,
                    closed=True,
                    facecolor=color,
                    edgecolor=color,
                    linestyle='--',
                    linewidth=1.0,
                    alpha=0.18,
                    zorder=7,
                    label=f"{robot_id} footprint"
                )
                self.ax.add_patch(margin_patch)

            robot_patch = self.create_custom_robot_patch(settings.ROBOT_LENGTH_CM, settings.ROBOT_WIDTH_CM)
            robot_patch.set_facecolor(color)
            robot_patch.set_edgecolor(color)
            t_rotate = mtransforms.Affine2D().rotate_deg(robot_angle)
            t_translate = mtransforms.Affine2D().translate(rx, ry)
            robot_patch.set_transform(t_rotate + t_translate + self.ax.transData)
            self.ax.add_patch(robot_patch)

            dx = (settings.ROBOT_LENGTH_CM / 2 + 3) * math.cos(math.radians(robot_angle))
            dy = (settings.ROBOT_LENGTH_CM / 2 + 3) * math.sin(math.radians(robot_angle))
            self.ax.arrow(rx, ry, dx, dy, head_width=2.5, head_length=2.5, fc=color, ec=color, zorder=9)
            self.ax.text(rx, ry + settings.ROBOT_WIDTH_CM, robot_id, color=color, fontsize=8,
                         fontweight='bold', ha='center', va='bottom', zorder=12)

        # 12. MOUSE TOOLTIP
        if self.mouse_x is not None and self.mouse_y is not None:
            self.ax.plot(self.mouse_x, self.mouse_y, '+', color='#ffcc00', markersize=8, markeredgewidth=1.2, zorder=13)
            offset_x = (xlim[1] - xlim[0]) * 0.02
            offset_y = (ylim[1] - ylim[0]) * 0.02
            self.ax.text(
                self.mouse_x + offset_x, self.mouse_y + offset_y,
                f"X: {self.mouse_x:.1f}\nY: {self.mouse_y:.1f}",
                color='#ffcc00', fontsize=8, fontfamily='monospace', fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='#111111', edgecolor='#ffcc00', alpha=0.85),
                zorder=14
            )

        self.ax.legend(loc='upper right', facecolor='#1e1e1e', edgecolor='#444', fontsize=8)
        self.canvas.draw_idle()

    def reset_view(self):
        """Center and reset zoom on the environment."""
        if self.boundary and self.boundary.enabled and self.boundary.polygon:
            min_x, min_y, max_x, max_y = self.boundary.bounding_box
            self.ax.set_xlim(min_x - 10, max_x + 10)
            self.ax.set_ylim(min_y - 10, max_y + 10)
        else:
            # Unbounded map: show a default 200x200 cm view centered at origin
            self.ax.set_xlim(-20, 200)
            self.ax.set_ylim(-20, 200)
        self.draw_grid()

    # --- EXTERNAL STATE SETTERS ---

    def _robot_color(self, robot_id: str, fallback_index: int = 0) -> str:
        """Keep waypoint and robot marker colors stable for each robot ID."""
        if robot_id in self.robot_poses:
            robot_index = list(self.robot_poses).index(robot_id)
        else:
            robot_index = fallback_index
        return self.ROBOT_COLORS[robot_index % len(self.ROBOT_COLORS)]

    def set_robot_pose(self, x: float, y: float, heading: float):
        self.robot_pos = (x, y)
        self.robot_angle = heading
        self.set_named_robot_pose("primary", x, y, heading, redraw=False)
        self.draw_grid()

    def set_named_robot_pose(self, robot_id: str, x: float, y: float, heading: float, redraw: bool = True):
        """Update one robot marker while retaining the legacy primary pose API."""
        self.robot_poses[robot_id] = (float(x), float(y), float(heading))
        if redraw:
            self.draw_grid()

    def set_initial_pos(self, pos: Optional[Tuple[float, float]]):
        self.initial_pos = pos
        self.draw_grid()

    def set_current_target(self, target: Optional[Tuple[float, float]]):
        self.current_target = target
        self.draw_grid()

    def set_waypoint_queue(self, queue: List[Tuple[float, float]]):
        self.waypoint_queue = list(queue)
        self.draw_grid()

    def set_robot_waypoints(self, waypoints: Dict[str, List[Tuple[float, float]]]):
        """Render independent waypoint paths for each robot without cross-links."""
        self.robot_waypoints = {
            robot_id: list(points)
            for robot_id, points in waypoints.items()
        }
        self.waypoint_queue = []
        self.draw_grid()

    def set_path_history(self, history: List[List[Tuple[float, float]]]):
        self.path_history = history
        self.draw_grid()

    def set_named_path_history(self, robot_id: str, history: List[List[Tuple[float, float]]], redraw: bool = True):
        """Update the visible trail for one fleet robot."""
        self.robot_path_histories[robot_id] = history
        if redraw:
            self.draw_grid()

    def set_boundary(self, boundary: Boundary):
        self.boundary = boundary
        self.draw_grid()

    def set_restricted_areas(self, areas: List[RestrictedArea]):
        self.restricted_areas = areas
        self.draw_grid()

    def set_planned_path(self, points: List[Tuple[float, float]]):
        self.planned_path_points = points
        self.draw_grid()

    def set_ghost_pose(self, pose: Optional[Tuple[float, float, float]]):
        self.ghost_pose = pose
        self.draw_grid()

    def clear_trails(self):
        self.path_history.clear()
        self.robot_path_histories.clear()
        self.draw_grid()

    # --- MOUSE INTERACTION HANDLERS ---

    def _on_mouse_press(self, event):
        if event.button == 2:  # Middle click: drag-pan
            self.is_panning = True
            self.pan_start_x = event.x
            self.pan_start_y = event.y
            self.pan_orig_xlim = self.ax.get_xlim()
            self.pan_orig_ylim = self.ax.get_ylim()
        elif event.button == 1:  # Left click: emit grid click
            if event.xdata is not None and event.ydata is not None:
                grid_step = self.current_grid_cm
                x_snap = round(event.xdata / grid_step) * grid_step
                y_snap = round(event.ydata / grid_step) * grid_step
                self.grid_clicked.emit(x_snap, y_snap)

    def _on_mouse_release(self, event):
        if event.button == 2:
            self.is_panning = False

    def _on_mouse_move(self, event):
        if event.xdata is not None and event.ydata is not None:
            self.mouse_x = event.xdata
            self.mouse_y = event.ydata
            self.mouse_moved.emit(self.mouse_x, self.mouse_y)
        else:
            self.mouse_x = None
            self.mouse_y = None

        if self.is_panning and event.x is not None and event.y is not None:
            dx_pixels = event.x - self.pan_start_x
            dy_pixels = event.y - self.pan_start_y
            bbox = self.ax.get_window_extent()
            x_range = self.pan_orig_xlim[1] - self.pan_orig_xlim[0]
            y_range = self.pan_orig_ylim[1] - self.pan_orig_ylim[0]

            dx_data = (dx_pixels / bbox.width) * x_range
            dy_data = (dy_pixels / bbox.height) * y_range

            self.ax.set_xlim([self.pan_orig_xlim[0] - dx_data, self.pan_orig_xlim[1] - dx_data])
            self.ax.set_ylim([self.pan_orig_ylim[0] - dy_data, self.pan_orig_ylim[1] - dy_data])

        self.draw_grid()

    def _on_mouse_wheel_zoom(self, event):
        if event.xdata is None or event.ydata is None:
            return

        base_scale = 1.15
        scale_factor = (1 / base_scale) if event.button == 'up' else base_scale

        cur_x, cur_y = event.xdata, event.ydata
        x_min, x_max = self.ax.get_xlim()
        y_min, y_max = self.ax.get_ylim()

        new_width = (x_max - x_min) * scale_factor
        new_height = (y_max - y_min) * scale_factor

        rel_x = (cur_x - x_min) / (x_max - x_min)
        rel_y = (cur_y - y_min) / (y_max - y_min)

        self.ax.set_xlim([cur_x - new_width * rel_x, cur_x + new_width * (1 - rel_x)])
        self.ax.set_ylim([cur_y - new_height * rel_y, cur_y + new_height * (1 - rel_y)])

        self.draw_grid()
