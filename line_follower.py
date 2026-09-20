#!/usr/bin/env python3
"""
FawBot Multi-Robot Fleet Virtual Line Follower (IDs: 22, 23, 31)
=================================================================
Tracks FawBots via overhead camera in true borderless FULLSCREEN mode.
Fixes coordinate distortion by decoupling computer vision tracking math
from display frame stretching, and sends robust UDP packets to robots.

Controls:
  - Left Mouse Drag  : Draw black virtual line on camera feed
  - Right Click      : Clear drawn virtual lines
  - SPACEBAR         : Assign drawn lines to robots & START line following
  - S                : EMERGENCY STOP all robots
  - C                : Clear all lines and routes
  - F                : Toggle full-screen mode
  - Q / ESC          : Quit
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

ARRIVAL_THRESHOLD_PX = 30
ALIGN_ENTER_DEG = 22.0
ALIGN_EXIT_DEG = 12.0
CMD_SEND_INTERVAL_SEC = 0.06
COLLISION_RADIUS_PX = 75
BORDER_PAD = 20
MIN_DRAW_DIST_PX = 15

SUPPORTED_DICTS = {
    "DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL,
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_4X4_100": aruco.DICT_4X4_100,
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_6X6_50": aruco.DICT_6X6_50,
}


def draw_outlined_text(img, text, pos, scale=0.40, color=(255, 255, 255), thickness=1):
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
            
            # Send standard command string with newline termination for firmware compatibility
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


class MultiRobotLineFollowerController:
    def __init__(self, cam_idx=0, fullscreen=True, target_w=PRIMARY_SCREEN_W, target_h=PRIMARY_SCREEN_H):
        self.robots = {rid: RobotAgent(rid, cfg) for rid, cfg in FLEET_CONFIG.items()}
        self.target_w = target_w
        self.target_h = target_h
        self.raw_w = 1280
        self.raw_h = 720

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
            print("[Error] Failed to open any camera device.")
            sys.exit(1)

        self.dict_names = list(SUPPORTED_DICTS.keys())
        self.active_dict_idx = 0
        self.detectors = {}
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self._init_detectors()

        # Mouse Drawing State Variables
        self.is_drawing = False
        self.drawn_lines = []
        self.current_line = []

        self.fleet_active = False
        self.banner_text = "CLICK & DRAG TO DRAW BLACK VIRTUAL LINE -> PRESS SPACEBAR TO FOLLOW"
        self.banner_color = (0, 255, 255)

        self._running = True
        self._mdns_thread = threading.Thread(target=self._background_mdns_resolver, daemon=True)
        self._mdns_thread.start()

        self.window_name = "FawBot Multi-Robot Line Follower (IDs: 22, 23, 31)"
        self.is_fullscreen = fullscreen

        # Window Setup
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
        # Scale screen click coordinates back to raw unscaled camera dimensions
        raw_x = int(x * (self.raw_w / float(self.target_w)))
        raw_y = int(y * (self.raw_h / float(self.target_h)))

        if event == cv2.EVENT_LBUTTONDOWN:
            self.is_drawing = True
            self.current_line = [(raw_x, raw_y)]

        elif event == cv2.EVENT_MOUSEMOVE and self.is_drawing:
            if self.current_line:
                last_pt = self.current_line[-1]
                dist = math.hypot(raw_x - last_pt[0], raw_y - last_pt[1])
                if dist >= MIN_DRAW_DIST_PX:
                    self.current_line.append((raw_x, raw_y))

        elif event == cv2.EVENT_LBUTTONUP:
            if self.is_drawing:
                self.is_drawing = False
                if len(self.current_line) > 1:
                    self.drawn_lines.append(list(self.current_line))
                    if not self.fleet_active:
                        self.banner_text = f"{len(self.drawn_lines)} LINE(S) DRAWN. PRESS SPACEBAR TO FOLLOW"
                        self.banner_color = (0, 255, 255)
                self.current_line = []

        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.drawn_lines:
                self.drawn_lines.pop()
                count = len(self.drawn_lines)
                self.banner_text = f"{count} LINE(S) REMAINING. PRESS SPACE TO RUN" if count > 0 else "CLICK & DRAG TO DRAW LINE"

    def toggle_fullscreen(self):
        self.is_fullscreen = not self.is_fullscreen
        prop = cv2.WINDOW_FULLSCREEN if self.is_fullscreen else cv2.WINDOW_NORMAL
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, prop)

    def detect_all_robots(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        enhanced_gray = self.clahe.apply(gray)
        padded_gray = cv2.copyMakeBorder(enhanced_gray, BORDER_PAD, BORDER_PAD, BORDER_PAD, BORDER_PAD, cv2.BORDER_REFLECT)

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

    def assign_lines_to_robots(self):
        all_paths = list(self.drawn_lines)
        if self.current_line and len(self.current_line) > 1:
            all_paths.append(self.current_line)

        if not all_paths:
            self.banner_text = "NO VIRTUAL LINES DRAWN! DRAG MOUSE TO DRAW FIRST."
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

        num_paths = len(all_paths)
        num_robots = len(visible_robots)

        if num_paths <= num_robots and SCIPY_AVAILABLE:
            cost_matrix = np.zeros((num_robots, num_paths))
            for i, robot in enumerate(visible_robots):
                for j, path in enumerate(all_paths):
                    start_pt = path[0]
                    end_pt = path[-1]
                    dist_to_start = math.hypot(start_pt[0] - robot.pos[0], start_pt[1] - robot.pos[1])
                    dist_to_end = math.hypot(end_pt[0] - robot.pos[0], end_pt[1] - robot.pos[1])
                    cost_matrix[i, j] = min(dist_to_start, dist_to_end)

            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            for r_idx, p_idx in zip(row_ind, col_ind):
                robot = visible_robots[r_idx]
                path = all_paths[p_idx]
                
                dist_s = math.hypot(path[0][0] - robot.pos[0], path[0][1] - robot.pos[1])
                dist_e = math.hypot(path[-1][0] - robot.pos[0], path[-1][1] - robot.pos[1])
                robot.route = list(reversed(path)) if dist_e < dist_s else list(path)
                robot.is_navigating = True
        else:
            for path in all_paths:
                best_robot = min(visible_robots, key=lambda r: math.hypot(path[0][0] - r.pos[0], path[0][1] - r.pos[1]))
                if not best_robot.is_navigating:
                    best_robot.route = list(path)
                    best_robot.is_navigating = True

        self.fleet_active = True
        self.banner_text = "FLEET FOLLOWING BLACK VIRTUAL LINES..."
        self.banner_color = (0, 255, 0)

    def check_collision_avoidance(self):
        active_robots = [r for r in self.robots.values() if r.is_navigating and r.pos is not None]
        for r in active_robots:
            r.is_yielding = False

        for i in range(len(active_robots)):
            for j in range(i + 1, len(active_robots)):
                r1, r2 = active_robots[i], active_robots[j]
                d = math.hypot(r1.pos[0] - r2.pos[0], r1.pos[1] - r2.pos[1])

                if d < COLLISION_RADIUS_PX:
                    dist1 = math.hypot(r1.route[0][0] - r1.pos[0], r1.route[0][1] - r1.pos[1]) if r1.route else 9999
                    dist2 = math.hypot(r2.route[0][0] - r2.pos[0], r2.route[0][1] - r2.pos[1]) if r2.route else 9999
                    yielding_bot = r1 if dist1 > dist2 else r2
                    yielding_bot.is_yielding = True
                    yielding_bot.send("S")
                    yielding_bot.status_msg = "YIELDING (COLLISION)"

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
                    robot.status_msg = "LINE FINISHED"
                else:
                    robot.status_msg = f"FOLLOWING LINE ({len(robot.route)} pts left)"
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
                robot.status_msg = f"ALIGNING ({error_angle:+.0f} deg)"
            else:
                robot.motion_cmd = "F"
                robot.send("F")
                robot.status_msg = f"TRACKING LINE ({dist_px:.0f}px)"

        if all_completed and self.fleet_active:
            self.fleet_active = False
            self.banner_text = "LINE FOLLOWING COMPLETE! ALL ROBOTS FINISHED."
            self.banner_color = (0, 255, 0)

    def emergency_stop_all(self):
        self.fleet_active = False
        for robot in self.robots.values():
            robot.emergency_stop()
            robot.status_msg = "STOPPED"
        self.banner_text = "EMERGENCY STOPPED!"
        self.banner_color = (0, 0, 255)

    def clear_all(self):
        self.fleet_active = False
        self.drawn_lines = []
        self.current_line = []
        for robot in self.robots.values():
            robot.is_navigating = False
            robot.route = []
            robot.turning = False
            robot.send("S", force=True)
            robot.status_msg = "CLEARED"
        self.banner_text = "ALL VIRTUAL LINES CLEARED. DRAG MOUSE TO DRAW."
        self.banner_color = (255, 255, 255)

    def draw_hud(self, frame):
        BLACK = (0, 0, 0)
        WHITE_OUTLINE = (255, 255, 255)

        # Unassigned User Drawn Lines
        for line in self.drawn_lines:
            for i in range(1, len(line)):
                cv2.line(frame, line[i - 1], line[i], WHITE_OUTLINE, 5, cv2.LINE_AA)
                cv2.line(frame, line[i - 1], line[i], BLACK, 3, cv2.LINE_AA)
            for pt in line:
                cv2.circle(frame, pt, 4, WHITE_OUTLINE, -1)
                cv2.circle(frame, pt, 2, BLACK, -1)

        # Currently Active Line Drag
        if len(self.current_line) > 1:
            for i in range(1, len(self.current_line)):
                cv2.line(frame, self.current_line[i - 1], self.current_line[i], WHITE_OUTLINE, 4, cv2.LINE_AA)
                cv2.line(frame, self.current_line[i - 1], self.current_line[i], BLACK, 2, cv2.LINE_AA)

        # Assigned Lines per Robot
        for robot in self.robots.values():
            if robot.route:
                if robot.pos is not None:
                    cv2.line(frame, (int(robot.pos[0]), int(robot.pos[1])), robot.route[0], WHITE_OUTLINE, 3, cv2.LINE_AA)
                    cv2.line(frame, (int(robot.pos[0]), int(robot.pos[1])), robot.route[0], BLACK, 1, cv2.LINE_AA)

                for j in range(len(robot.route) - 1):
                    cv2.line(frame, robot.route[j], robot.route[j + 1], WHITE_OUTLINE, 6, cv2.LINE_AA)
                    cv2.line(frame, robot.route[j], robot.route[j + 1], BLACK, 4, cv2.LINE_AA)

                for pt in robot.route:
                    cv2.circle(frame, pt, 5, WHITE_OUTLINE, -1)
                    cv2.circle(frame, pt, 3, BLACK, -1)

        # Robot Tag Overlays
        for robot in self.robots.values():
            if robot.visible and robot.pos is not None:
                rx, ry = int(robot.pos[0]), int(robot.pos[1])
                if robot.corners is not None:
                    cv2.polylines(frame, [robot.corners], True, robot.color, 2)

                rad = math.radians(robot.angle)
                tip_x = int(rx + 30 * math.cos(rad))
                tip_y = int(ry + 30 * math.sin(rad))
                cv2.arrowedLine(frame, (rx, ry), (tip_x, tip_y), (0, 0, 255), 2, tipLength=0.3)

                tag = f"Bot {robot.id}"
                if robot.is_yielding:
                    tag += " [YIELD]"
                    cv2.circle(frame, (rx, ry), COLLISION_RADIUS_PX, (0, 0, 255), 1, cv2.LINE_AA)

                draw_outlined_text(frame, tag, (rx - 20, ry - 12), scale=0.42, color=robot.color, thickness=1)

        # HUD Top Status Text
        draw_outlined_text(frame, self.banner_text, (15, 25), scale=0.45, color=self.banner_color, thickness=1)

        y_offset = 45
        for rid, robot in self.robots.items():
            status_color = robot.color if robot.visible else (160, 160, 160)
            vis_str = "VIS" if robot.visible else "LOST"
            route_str = f"Line: {len(robot.route)} pts" if robot.route else "No Line"
            row_text = f"[{vis_str}] Bot {rid} ({robot.name}) | {robot.ip} | {route_str} | {robot.status_msg}"
            draw_outlined_text(frame, row_text, (15, y_offset), scale=0.38, color=status_color, thickness=1)
            y_offset += 18

        h, w = frame.shape[:2]
        legend_str = "[Left Drag]: Draw Black Line  |  [Right Click]: Undo Line  |  [SPACEBAR]: Assign & FOLLOW  |  [S]: STOP  |  [C]: Clear  |  [F]: Fullscreen  |  [Q/Esc]: Quit"
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

                # Resize ONLY the final display output to match full screen dimensions
                display_frame = cv2.resize(raw_frame, (self.target_w, self.target_h), interpolation=cv2.INTER_LINEAR)

                cv2.imshow(self.window_name, display_frame)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    self.emergency_stop_all()
                    break
                elif key == 32:
                    self.assign_lines_to_robots()
                elif key in (ord('s'), ord('S')):
                    self.emergency_stop_all()
                elif key in (ord('c'), ord('C')):
                    self.clear_all()
                elif key in (ord('f'), ord('F')):
                    self.toggle_fullscreen()

        except KeyboardInterrupt:
            pass
        finally:
            self._running = False
            self.emergency_stop_all()
            if self.cap is not None:
                self.cap.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="FawBot Multi-Robot Fleet Virtual Line Follower")
    parser.add_argument("--camera", type=int, default=0, help="Camera device index")
    parser.add_argument("--windowed", action="store_true", help="Launch in windowed mode")
    args = parser.parse_args()

    controller = MultiRobotLineFollowerController(cam_idx=args.camera, fullscreen=(not args.windowed))
    controller.run()