#!/usr/bin/env python3
"""
FawBot Multi-Robot Fleet Overhead Controller (IDs: 22, 23, 31)
==============================================================
Tracks multiple FawBots simultaneously via an overhead camera, allows clicking
arbitrary target points or drawing multiple polygon restricted areas on the camera feed.
Upon pressing SPACEBAR, executes obstacle-avoiding routing via Visibility Graphs.

UI & Key Controls:
  - Left Click: Place target point OR draw polygon vertex
  - 'N': Finish current restricted polygon and allow drawing another
  - 'Right Click': Undo last point
  - 'SPACEBAR': Optimize path avoiding restricted areas & START
  - 'S': Emergency Stop
  - 'C': Clear all routes, points, and restricted areas
  - 'F': Toggle Fullscreen
  - 'Q' / 'Esc': Quit
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import socket
import math
import time
import sys
import threading
import argparse
from collections import deque

import networkx as nx
from shapely.geometry import Point, LineString, Polygon
from shapely.ops import unary_union

# Dynamic Screen Resolution Detection
try:
    from screeninfo import get_monitors
    monitors = get_monitors()
    if monitors:
        PRIMARY_SCREEN_W = monitors[0].width
        PRIMARY_SCREEN_H = monitors[0].height
    else:
        PRIMARY_SCREEN_W, PRIMARY_SCREEN_H = 1920, 1080
except Exception:
    PRIMARY_SCREEN_W, PRIMARY_SCREEN_H = 1920, 1080

try:
    from scipy.optimize import linear_sum_assignment
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

FLEET_CONFIG = {
    22: {
        "host": "fawbot_22.local",
        "fallback_ip": "192.168.1.6",
        "port": 8888,
        "color": (255, 255, 0),       # Cyan
        "name": "Fawbot_22"
    },
    23: {
        "host": "fawbot_23.local",
        "fallback_ip": "192.168.1.13",
        "port": 8888,
        "color": (0, 255, 0),         # Lime Green
        "name": "Fawbot_23"
    },
    31: {
        "host": "fawbot_31.local",
        "fallback_ip": "192.168.1.7",
        "port": 8888,
        "color": (255, 0, 255),       # Magenta
        "name": "Fawbot_31"
    }
}

ARRIVAL_THRESHOLD_PX = 35
ALIGN_ENTER_DEG = 22.0
ALIGN_EXIT_DEG = 12.0
CMD_SEND_INTERVAL_SEC = 0.06
COLLISION_RADIUS_PX = 75
BORDER_PAD = 20  # Pixels padded around frame for edge detection
ROBOT_SAFETY_BUFFER_PX = 25  # Safety offset applied around obstacles

SUPPORTED_DICTS = {
    "DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL,
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_4X4_100": aruco.DICT_4X4_100,
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_6X6_50": aruco.DICT_6X6_50,
}


def draw_outlined_text(img, text, pos, scale=0.40, color=(255, 255, 255), thickness=1):
    """Draws smaller text with a black stroke outline over transparent camera feed."""
    x, y = pos
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


class RobotAgent:
    def __init__(self, robot_id, cfg):
        self.id = robot_id
        self.host = cfg["host"]
        self.fallback_ip = cfg["fallback_ip"]
        self.port = cfg["port"]
        self.color = cfg["color"]
        self.name = cfg["name"]

        self.pos = None
        self.angle = None
        self.corners = None
        self.visible = False
        self.last_seen_time = 0.0
        self.history_trail = deque(maxlen=40)

        self.route = []
        self.is_navigating = False
        self.turning = False
        self.turn_dir = None
        self.is_yielding = False
        self.motion_cmd = "S"
        self.status_msg = "IDLE"

        self.ip = self.fallback_ip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.target_addr = (self.ip, self.port)
        self.last_cmd = None
        self.last_cmd_time = 0.0
        self.last_robot_alert = ""
        self.is_connected = False
        self._lock = threading.Lock()

    def update_ip(self, new_ip):
        with self._lock:
            if new_ip != self.ip:
                self.ip = new_ip
                self.target_addr = (self.ip, self.port)
                self.is_connected = True

    def send(self, cmd_str, force=False):
        now = time.time()
        if not force and cmd_str == self.last_cmd and (now - self.last_cmd_time) < CMD_SEND_INTERVAL_SEC:
            return
        try:
            with self._lock:
                addr = self.target_addr
            payload = f"{cmd_str}\n".encode("utf-8")
            self.sock.sendto(payload, addr)
            self.last_cmd = cmd_str
            self.last_cmd_time = now
        except Exception as e:
            self.status_msg = f"UDP ERR: {e}"

    def emergency_stop(self):
        self.is_navigating = False
        self.route = []
        self.turning = False
        self.motion_cmd = "S"
        for _ in range(3):
            self.send("S", force=True)
            self.send("STOP", force=True)
            time.sleep(0.005)

    def poll_feedback(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(256)
                msg = data.decode("utf-8", errors="ignore").strip()
                if msg:
                    self.last_robot_alert = msg
                    self.is_connected = True
            except (BlockingIOError, socket.error):
                break

    def update_pose(self, center, heading, corners):
        self.pos = (float(center[0]), float(center[1]))
        self.angle = heading
        self.corners = corners.astype(int)
        self.visible = True
        self.last_seen_time = time.time()
        self.history_trail.append(self.pos)

    def mark_lost(self):
        self.visible = False
        self.pos = None
        self.angle = None
        self.corners = None
        if self.is_navigating and self.motion_cmd != "S":
            self.motion_cmd = "S"
            self.send("S", force=True)
            self.status_msg = "MARKER LOST"


class MultiRobotFleetController:
    def __init__(self, cam_idx=0, fullscreen=True, target_w=PRIMARY_SCREEN_W, target_h=PRIMARY_SCREEN_H):
        self.robots = {rid: RobotAgent(rid, cfg) for rid, cfg in FLEET_CONFIG.items()}
        self.target_w = target_w
        self.target_h = target_h
        self.raw_w = 1280
        self.raw_h = 720

        # macOS Robust AVFoundation Video Capture Probe
        self.cap = None
        search_indices = [cam_idx, 0, 1, 2] if cam_idx not in [0, 1, 2] else [cam_idx, 0, 1, 2]
        seen_indices = []
        
        for idx in search_indices:
            if idx in seen_indices:
                continue
            seen_indices.append(idx)
            print(f"[Camera] Probing index {idx} with CAP_AVFOUNDATION...")
            cap = cv2.VideoCapture(idx, cv2.CAP_AVFOUNDATION)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.raw_w)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.raw_h)
                ret, test_frame = cap.read()
                if ret and test_frame is not None:
                    print(f"[Camera] Successfully initialized camera index {idx}!")
                    self.cap = cap
                    self.raw_h, self.raw_w = test_frame.shape[:2]
                    break
                cap.release()

        if self.cap is None:
            print("[Error] Failed to open any camera device (checked indices 0, 1, 2).")
            print("[Fix] Ensure camera permissions are granted to Terminal/VSCode in macOS System Settings.")
            sys.exit(1)

        self.dict_names = list(SUPPORTED_DICTS.keys())
        self.active_dict_idx = 0
        self.detectors = {}
        
        # Initialize CLAHE for low-light contrast enhancement
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self._init_detectors()

        self.unassigned_points = []
        
        # Restricted Area Data Structures
        self.restricted_polygons = []   # Completed Polygon objects (np.ndarray of shape (N, 2))
        self.current_poly_pts = []      # Currently active polyline being drawn
        self.drawing_obstacle = False   # Toggle drawing mode

        self.fleet_active = False
        self.banner_text = "CLICK TO ADD GOAL POINTS | PRESS 'N' TO DRAW RESTRICTED POLYGON"
        self.banner_color = (0, 255, 255)

        self._running = True
        self._mdns_thread = threading.Thread(target=self._background_mdns_resolver, daemon=True)
        self._mdns_thread.start()

        self.window_name = "FawBot Multi-Robot Fleet Navigation (IDs: 22, 23, 31)"
        self.is_fullscreen = fullscreen

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        if self.is_fullscreen:
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        cv2.setMouseCallback(self.window_name, self._on_mouse)

    def _init_detectors(self):
        params = aruco.DetectorParameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        params.minMarkerPerimeterRate = 0.01
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 45
        params.adaptiveThreshWinSizeStep = 5
        params.adaptiveThreshConstant = 7
        params.minDistanceToBorder = 0
        params.perspectiveRemoveIgnoredMarginPerCell = 0.05

        for name, dict_val in SUPPORTED_DICTS.items():
            adict = aruco.getPredefinedDictionary(dict_val)
            self.detectors[name] = aruco.ArucoDetector(adict, params)

    def _background_mdns_resolver(self):
        while self._running:
            for rid, robot in self.robots.items():
                if not self._running:
                    break
                try:
                    resolved = socket.gethostbyname(robot.host)
                    robot.update_ip(resolved)
                except Exception:
                    pass
            for _ in range(16):
                if not self._running:
                    break
                time.sleep(0.5)

    def _on_mouse(self, event, x, y, flags, param):
        raw_x = int(x * (self.raw_w / float(self.target_w)))
        raw_y = int(y * (self.raw_h / float(self.target_h)))

        if event == cv2.EVENT_LBUTTONDOWN:
            if self.drawing_obstacle:
                self.current_poly_pts.append((raw_x, raw_y))
                self.banner_text = f"DRAWING POLYGON: {len(self.current_poly_pts)} PTS. PRESS 'N' TO FINISH"
                self.banner_color = (0, 165, 255)
            else:
                self.unassigned_points.append((raw_x, raw_y))
                if not self.fleet_active:
                    self.banner_text = f"{len(self.unassigned_points)} POINT(S) ADDED. PRESS SPACEBAR TO EXECUTE"
                    self.banner_color = (0, 255, 255)

        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.drawing_obstacle and self.current_poly_pts:
                self.current_poly_pts.pop()
            elif self.unassigned_points:
                self.unassigned_points.pop()
                count = len(self.unassigned_points)
                self.banner_text = f"{count} POINT(S) REMAINING. PRESS SPACE TO RUN" if count > 0 else "CLICK TO ADD GOAL POINTS"

    def finalize_current_polygon(self):
        """Closes the current restricted polygon definition."""
        if len(self.current_poly_pts) >= 3:
            poly_np = np.array(self.current_poly_pts, dtype=np.int32)
            self.restricted_polygons.append(poly_np)
            self.current_poly_pts = []
            self.drawing_obstacle = False
            self.banner_text = f"RESTRICTED AREA ADDED! TOTAL AREAS: {len(self.restricted_polygons)}"
            self.banner_color = (0, 255, 0)
        else:
            self.banner_text = "NEED AT LEAST 3 POINTS TO FORM A POLYGON RESTRICTED AREA!"
            self.banner_color = (0, 0, 255)

    def toggle_fullscreen(self):
        self.is_fullscreen = not self.is_fullscreen
        prop = cv2.WINDOW_FULLSCREEN if self.is_fullscreen else cv2.WINDOW_NORMAL
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, prop)

    def cycle_dictionary(self):
        self.active_dict_idx = (self.active_dict_idx + 1) % len(self.dict_names)

    def detect_all_robots(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        enhanced_gray = self.clahe.apply(gray)

        padded_gray = cv2.copyMakeBorder(
            enhanced_gray, BORDER_PAD, BORDER_PAD, BORDER_PAD, BORDER_PAD, 
            cv2.BORDER_REFLECT
        )

        active_name = self.dict_names[self.active_dict_idx]
        corners, ids, _ = self.detectors[active_name].detectMarkers(padded_gray)

        current_seen = set()
        if ids is not None:
            flat_ids = ids.flatten()
            for rid, robot in self.robots.items():
                if rid in flat_ids:
                    idx = np.where(flat_ids == rid)[0][0]
                    c = corners[idx][0].copy()
                    
                    c[:, 0] -= BORDER_PAD
                    c[:, 1] -= BORDER_PAD
                    
                    center = np.mean(c, axis=0)

                    front = (c[0] + c[1]) / 2.0
                    back = (c[2] + c[3]) / 2.0
                    heading_deg = math.degrees(math.atan2(front[1] - back[1], front[0] - back[0]))

                    robot.update_pose(center, heading_deg, c)
                    current_seen.add(rid)

        for rid, robot in self.robots.items():
            if rid not in current_seen:
                robot.mark_lost()

    # ========================== VISIBILITY GRAPH PATHFINDING ========================== #
    def _compute_obstacle_free_path(self, start_pt, end_pt):
        """
        Calculates an optimal geometric visibility graph path avoiding restricted polygons
        by maintaining strict clearance using the robot's physical safety buffer.
        """
        start_tuple = (int(start_pt[0]), int(start_pt[1]))
        end_tuple = (int(end_pt[0]), int(end_pt[1]))

        if not self.restricted_polygons:
            return [start_tuple, end_tuple]

        buffered_polys = []

        for p_np in self.restricted_polygons:
            if len(p_np) < 3:
                continue
            sp = Polygon(p_np)
            # Fix geometry winding order
            if not sp.is_valid:
                sp = sp.buffer(0)
            
            # Buffer polygon outwards to account for robot footprint body radius
            buf = sp.buffer(ROBOT_SAFETY_BUFFER_PX, join_style=2)
            if not buf.is_valid:
                buf = buf.buffer(0)
            buffered_polys.append(buf)

        if not buffered_polys:
            return [start_tuple, end_tuple]

        combined_buffered = unary_union(buffered_polys)

        p_start = Point(start_tuple)
        p_end = Point(end_tuple)

        def line_collides(line_seg, obstacle_geom):
            """Robust collision check against buffered obstacle geometries."""
            if obstacle_geom.intersects(line_seg) or obstacle_geom.crosses(line_seg):
                return True
            if obstacle_geom.contains(line_seg):
                return True
            return False

        direct_line = LineString([start_tuple, end_tuple])
        
        # If direct path does NOT collide with any obstacle buffer, take direct route
        if not line_collides(direct_line, combined_buffered):
            return [start_tuple, end_tuple]

        # Extract graph nodes from exterior vertices of buffered polygons
        graph_nodes = [start_tuple, end_tuple]
        
        if combined_buffered.geom_type == 'Polygon':
            ext_polys = [combined_buffered]
        elif combined_buffered.geom_type == 'MultiPolygon':
            ext_polys = list(combined_buffered.geoms)
        else:
            ext_polys = []

        for poly in ext_polys:
            coords = list(poly.exterior.coords)[:-1]
            for c in coords:
                pt_coord = (int(c[0]), int(c[1]))
                # Only add waypoint node if it isn't inside another obstacle zone
                if not combined_buffered.contains(Point(pt_coord)):
                    graph_nodes.append(pt_coord)

        # Build Visibility Graph using NetworkX
        G = nx.Graph()
        n_count = len(graph_nodes)

        for i in range(n_count):
            for j in range(i + 1, n_count):
                ptA = graph_nodes[i]
                ptB = graph_nodes[j]
                
                # Ignore zero-length edges
                if ptA == ptB:
                    continue
                    
                edge = LineString([ptA, ptB])

                # Verify line segment does NOT collide with restricted zones
                if not line_collides(edge, combined_buffered):
                    dist = math.hypot(ptA[0] - ptB[0], ptA[1] - ptB[1])
                    G.add_edge(ptA, ptB, weight=dist)

        try:
            path = nx.shortest_path(G, source=start_tuple, target=end_tuple, weight='weight')
            return path
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            print("[Warning] No direct visibility path around obstacles found!")
            return [start_tuple, end_tuple]

    def optimize_and_assign_routes(self):
        if not self.unassigned_points:
            self.banner_text = "NO POINTS TO ASSIGN! CLICK ON CAMERA VIEW FIRST."
            self.banner_color = (0, 0, 255)
            return

        visible_robots = [r for r in self.robots.values() if r.pos is not None]
        if not visible_robots:
            self.banner_text = "NO ROBOTS DETECTED! ENSURE ARUCO MARKERS ARE VISIBLE."
            self.banner_color = (0, 0, 255)
            return

        for r in self.robots.values():
            r.route = []
            r.is_navigating = False
            r.turning = False

        num_points = len(self.unassigned_points)
        num_robots = len(visible_robots)

        if num_robots == 1:
            robot = visible_robots[0]
            curr_pos = robot.pos
            unvisited = list(self.unassigned_points)
            full_route = []

            while unvisited:
                next_pt = min(unvisited, key=lambda p: math.hypot(p[0] - curr_pos[0], p[1] - curr_pos[1]))
                segment = self._compute_obstacle_free_path(curr_pos, next_pt)
                if segment:
                    full_route.extend(segment[1:])
                    curr_pos = next_pt
                unvisited.remove(next_pt)

            robot.route = full_route
            robot.is_navigating = bool(full_route)

        elif num_points <= num_robots and SCIPY_AVAILABLE:
            cost_matrix = np.zeros((num_robots, num_points))
            for i, robot in enumerate(visible_robots):
                for j, pt in enumerate(self.unassigned_points):
                    cost_matrix[i, j] = math.hypot(pt[0] - robot.pos[0], pt[1] - robot.pos[1])

            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            for r_idx, p_idx in zip(row_ind, col_ind):
                r = visible_robots[r_idx]
                target = self.unassigned_points[p_idx]
                segment = self._compute_obstacle_free_path(r.pos, target)
                if segment:
                    r.route = segment[1:]
                    r.is_navigating = True

        else:
            robot_clusters = {r.id: [] for r in visible_robots}
            for pt in self.unassigned_points:
                best_robot = min(visible_robots, key=lambda r: math.hypot(pt[0] - r.pos[0], pt[1] - r.pos[1]))
                robot_clusters[best_robot.id].append(pt)

            for robot in visible_robots:
                pts = robot_clusters[robot.id]
                if not pts:
                    continue

                curr_pos = robot.pos
                unvisited = list(pts)
                sequenced_route = []

                while unvisited:
                    next_pt = min(unvisited, key=lambda p: math.hypot(p[0] - curr_pos[0], p[1] - curr_pos[1]))
                    segment = self._compute_obstacle_free_path(curr_pos, next_pt)
                    if segment:
                        sequenced_route.extend(segment[1:])
                        curr_pos = next_pt
                    unvisited.remove(next_pt)

                robot.route = sequenced_route
                robot.is_navigating = bool(sequenced_route)

        self.unassigned_points = []
        self.fleet_active = True
        self.banner_text = "FLEET EXECUTING OBSTACLE-FREE NEAREST ROUTES..."
        self.banner_color = (0, 255, 0)

    def check_collision_avoidance(self):
        active_robots = [r for r in self.robots.values() if r.is_navigating and r.pos is not None]

        for r in active_robots:
            r.is_yielding = False

        for i in range(len(active_robots)):
            for j in range(i + 1, len(active_robots)):
                r1 = active_robots[i]
                r2 = active_robots[j]
                d = math.hypot(r1.pos[0] - r2.pos[0], r1.pos[1] - r2.pos[1])

                if d < COLLISION_RADIUS_PX:
                    dist_to_goal1 = math.hypot(r1.route[0][0] - r1.pos[0], r1.route[0][1] - r1.pos[1]) if r1.route else 9999
                    dist_to_goal2 = math.hypot(r2.route[0][0] - r2.pos[0], r2.route[0][1] - r2.pos[1]) if r2.route else 9999

                    yielding_bot = r1 if dist_to_goal1 > dist_to_goal2 else r2
                    yielding_bot.is_yielding = True
                    yielding_bot.send("S")
                    yielding_bot.status_msg = "YIELDING (COLLISION AVOIDANCE)"

    def update_fleet_navigation(self):
        if not self.fleet_active:
            return

        all_completed = True

        for robot in self.robots.values():
            if not robot.is_navigating or not robot.route:
                continue

            all_completed = False

            if not robot.visible or robot.pos is None:
                robot.status_msg = "SEARCHING..."
                continue

            if robot.is_yielding:
                continue

            target_pt = robot.route[0]
            dx = target_pt[0] - robot.pos[0]
            dy = target_pt[1] - robot.pos[1]
            dist_px = math.hypot(dx, dy)

            if dist_px < ARRIVAL_THRESHOLD_PX:
                robot.route.pop(0)
                robot.turning = False

                if not robot.route:
                    robot.is_navigating = False
                    robot.motion_cmd = "S"
                    robot.send("S", force=True)
                    robot.status_msg = "COMPLETED"
                else:
                    robot.status_msg = f"NEXT GOAL ({len(robot.route)} LEFT)"
                continue

            target_angle = math.degrees(math.atan2(dy, dx))
            error_angle = (target_angle - robot.angle + 180.0) % 360.0 - 180.0

            if not robot.turning:
                if abs(error_angle) > ALIGN_ENTER_DEG:
                    robot.turning = True
                    robot.turn_dir = "R" if error_angle > 0 else "L"
            else:
                if abs(error_angle) <= ALIGN_EXIT_DEG:
                    robot.turning = False
                    robot.turn_dir = None
                else:
                    robot.turn_dir = "R" if error_angle > 0 else "L"

            if robot.turning:
                robot.motion_cmd = robot.turn_dir
                robot.send(robot.motion_cmd)
                robot.status_msg = f"ROTATING {'R' if robot.turn_dir == 'R' else 'L'} ({error_angle:+.0f} deg)"
            else:
                robot.motion_cmd = "F"
                robot.send("F")
                robot.status_msg = f"DRIVING ({dist_px:.0f}px)"

        if all_completed and self.fleet_active:
            self.fleet_active = False
            self.banner_text = "FLEET MISSION COMPLETE! ALL ROBOTS ARRIVED."
            self.banner_color = (0, 255, 0)

    def emergency_stop_all(self):
        self.fleet_active = False
        self.unassigned_points = []
        for robot in self.robots.values():
            robot.emergency_stop()
            robot.status_msg = "STOPPED"
        self.banner_text = "EMERGENCY STOPPED!"
        self.banner_color = (0, 0, 255)

    def clear_all(self):
        self.fleet_active = False
        self.unassigned_points = []
        self.restricted_polygons = []
        self.current_poly_pts = []
        self.drawing_obstacle = False
        for robot in self.robots.values():
            robot.is_navigating = False
            robot.route = []
            robot.turning = False
            robot.send("S", force=True)
            robot.status_msg = "CLEARED"
        self.banner_text = "ALL ROUTES & RESTRICTED AREAS CLEARED."
        self.banner_color = (255, 255, 255)

    def draw_hud(self, frame):
        # Render Restricted Area Polygons
        overlay = frame.copy()
        for poly in self.restricted_polygons:
            cv2.fillPoly(overlay, [poly], (0, 0, 200))  # Semi-transparent red overlay
            cv2.polylines(frame, [poly], True, (0, 0, 255), 2, cv2.LINE_AA)
        
        # Blend overlay for transparent restricted zones
        cv2.addWeighted(overlay, 0.35, frame, 0.65, 0, frame)

        # Draw currently forming obstacle polygon
        if self.current_poly_pts:
            pts_arr = np.array(self.current_poly_pts, dtype=np.int32)
            cv2.polylines(frame, [pts_arr], False, (0, 165, 255), 2, cv2.LINE_AA)
            for pt in self.current_poly_pts:
                cv2.circle(frame, pt, 4, (0, 165, 255), -1)

        # Breadcrumbs
        for robot in self.robots.values():
            pts = list(robot.history_trail)
            for i in range(1, len(pts)):
                p1 = (int(pts[i - 1][0]), int(pts[i - 1][1]))
                p2 = (int(pts[i][0]), int(pts[i][1]))
                alpha = i / len(pts)
                c = tuple(int(ch * alpha) for ch in robot.color)
                cv2.line(frame, p1, p2, c, 1)

        # Unassigned Target Points
        for i, pt in enumerate(self.unassigned_points):
            cv2.circle(frame, pt, ARRIVAL_THRESHOLD_PX, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.circle(frame, pt, 5, (0, 0, 0), -1)
            cv2.circle(frame, pt, 3, (0, 255, 255), -1)
            draw_outlined_text(frame, f"P{i+1}", (pt[0] + 8, pt[1] - 8), scale=0.42, color=(0, 255, 255), thickness=1)

        # Assigned Obstacle-Avoiding Routes
        for robot in self.robots.values():
            if robot.route:
                if robot.pos is not None:
                    cv2.line(frame, (int(robot.pos[0]), int(robot.pos[1])), robot.route[0], robot.color, 2, cv2.LINE_AA)

                for j in range(len(robot.route) - 1):
                    cv2.line(frame, robot.route[j], robot.route[j + 1], robot.color, 2, cv2.LINE_AA)

                for idx, pt in enumerate(robot.route):
                    is_active = (idx == 0)
                    cv2.circle(frame, pt, ARRIVAL_THRESHOLD_PX, robot.color, 2 if is_active else 1, cv2.LINE_AA)
                    cv2.circle(frame, pt, 5, (0, 0, 0), -1)
                    cv2.circle(frame, pt, 3, robot.color, -1)
                    label = f"R{robot.id} (#{idx+1})"
                    draw_outlined_text(frame, label, (pt[0] + 8, pt[1] - 8), scale=0.40, color=robot.color, thickness=1)

        # Robot Marker & Tag Overlays
        for robot in self.robots.values():
            if robot.visible and robot.pos is not None:
                rx, ry = int(robot.pos[0]), int(robot.pos[1])

                if robot.corners is not None:
                    cv2.polylines(frame, [robot.corners], True, robot.color, 2)

                cv2.circle(frame, (rx, ry), 5, (0, 0, 0), -1)
                cv2.circle(frame, (rx, ry), 3, robot.color, -1)

                rad = math.radians(robot.angle)
                arrow_len = 30
                tip_x = int(rx + arrow_len * math.cos(rad))
                tip_y = int(ry + arrow_len * math.sin(rad))
                cv2.arrowedLine(frame, (rx, ry), (tip_x, tip_y), (0, 0, 255), 2, tipLength=0.3)

                tag = f"Bot {robot.id}"
                if robot.is_yielding:
                    tag += " [YIELD]"
                    cv2.circle(frame, (rx, ry), COLLISION_RADIUS_PX, (0, 0, 255), 1, cv2.LINE_AA)

                draw_outlined_text(frame, tag, (rx - 20, ry - 12), scale=0.42, color=robot.color, thickness=1)

        # Top Fleet Status Text
        draw_outlined_text(frame, self.banner_text, (15, 25), scale=0.45, color=self.banner_color, thickness=1)

        y_offset = 45
        for rid, robot in self.robots.items():
            status_color = robot.color if robot.visible else (160, 160, 160)
            vis_str = "VIS" if robot.visible else "LOST"
            route_str = f"Route: {len(robot.route)} pts" if robot.route else "No Route"
            row_text = f"[{vis_str}] Bot {rid} ({robot.name}) | {robot.ip} | {route_str} | {robot.status_msg}"
            draw_outlined_text(frame, row_text, (15, y_offset), scale=0.38, color=status_color, thickness=1)
            y_offset += 18

        # Bottom Instructions Text
        h, w = frame.shape[:2]
        legend_str = "[Left Click]: Add Pt/Vertex | [N]: Finish Restricted Area | [SPACEBAR]: Assign & START | [S]: STOP | [C]: Clear All | [F]: Fullscreen | [Q/Esc]: Quit"
        draw_outlined_text(frame, legend_str, (15, h - 15), scale=0.38, color=(240, 240, 240), thickness=1)

    def run(self):
        try:
            while True:
                ret, raw_frame = self.cap.read()
                if not ret or raw_frame is None:
                    time.sleep(0.01)
                    continue

                for robot in self.robots.values():
                    robot.poll_feedback()

                # Process computer vision and path logic on native raw coordinates
                self.detect_all_robots(raw_frame)
                self.check_collision_avoidance()
                self.update_fleet_navigation()
                self.draw_hud(raw_frame)

                # Resize raw image to match target screen dimensions
                display_frame = cv2.resize(raw_frame, (self.target_w, self.target_h), interpolation=cv2.INTER_LINEAR)

                cv2.imshow(self.window_name, display_frame)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    self.emergency_stop_all()
                    break
                elif key == 32:  # SPACEBAR
                    self.optimize_and_assign_routes()
                elif key in (ord('n'), ord('N')):
                    if not self.drawing_obstacle:
                        self.drawing_obstacle = True
                        self.banner_text = "DRAWING MODE: LEFT-CLICK TO PLACE POLYGON VERTICES"
                        self.banner_color = (0, 165, 255)
                    else:
                        self.finalize_current_polygon()
                elif key in (ord('s'), ord('S')):
                    self.emergency_stop_all()
                elif key in (ord('c'), ord('C')):
                    self.clear_all()
                elif key in (ord('f'), ord('F')):
                    self.toggle_fullscreen()
                elif key in (ord('d'), ord('D')):
                    self.cycle_dictionary()

        except KeyboardInterrupt:
            pass
        finally:
            self._running = False
            self.emergency_stop_all()
            if self.cap is not None:
                self.cap.release()
            cv2.destroyAllWindows()


def parse_args():
    parser = argparse.ArgumentParser(description="FawBot Multi-Robot Fleet Controller with Restricted Area Avoidance")
    parser.add_argument("--camera", type=int, default=0, help="Camera device index")
    parser.add_argument("--windowed", action="store_true", help="Launch in windowed mode")
    parser.add_argument("--ip22", type=str, default=FLEET_CONFIG[22]["fallback_ip"])
    parser.add_argument("--ip23", type=str, default=FLEET_CONFIG[23]["fallback_ip"])
    parser.add_argument("--ip31", type=str, default=FLEET_CONFIG[31]["fallback_ip"])
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    FLEET_CONFIG[22]["fallback_ip"] = args.ip22
    FLEET_CONFIG[23]["fallback_ip"] = args.ip23
    FLEET_CONFIG[31]["fallback_ip"] = args.ip31

    controller = MultiRobotFleetController(cam_idx=args.camera, fullscreen=(not args.windowed))
    controller.run()