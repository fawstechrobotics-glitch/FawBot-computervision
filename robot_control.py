#!/usr/bin/env python3
"""
FawBot Overhead Camera ArUco Navigation Controller
==================================================
Tracks FawBot (ArUco ID 23) via an overhead webcam and drives the robot
autonomously to any clicked position using closed-loop visual servoing over UDP.

UI Enhancements:
  - Full-screen display mode by default (press 'F' to toggle)
  - 100% Transparent text background with crisp contrast outlines
  - Fast single-pass ArUco detection

Key Controls:
  - Left Click        : Set target point and start navigation
  - Shift + Left Click: Append waypoint to route
  - 'S' or SPACE      : Emergency STOP
  - 'C'               : Clear current target / cancel navigation
  - 'F'               : Toggle Fullscreen on / off
  - 'D'               : Cycle ArUco dictionary (Original -> 4x4 -> 5x5 -> 6x6)
  - 'Q' or ESC        : Stop motors and exit
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import socket
import math
import time
import sys
import argparse
from collections import deque

# --- DEFAULT SETTINGS ---
DEFAULT_ROBOT_HOST = "fawbot_23.local"
DEFAULT_FALLBACK_IP = "192.168.1.13"
DEFAULT_UDP_PORT = 8888
DEFAULT_ROBOT_ID = 23
DEFAULT_CAMERA_INDEX = 0

# --- CONTROL PARAMETERS ---
ARRIVAL_THRESHOLD_PX = 35     # Distance in pixels considered "arrived"
ALIGN_ENTER_DEG = 22.0        # Turn if heading error exceeds this threshold
ALIGN_EXIT_DEG = 12.0         # Stop turning when heading error drops below this threshold
CMD_SEND_INTERVAL_SEC = 0.06  # Rate limit UDP commands to ~16 Hz (prevents packet queue lag)

# --- ARUCO DICTIONARY REGISTRY ---
SUPPORTED_DICTS = {
    "DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL,
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_4X4_100": aruco.DICT_4X4_100,
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_6X6_50": aruco.DICT_6X6_50,
}


def draw_outlined_text(img, text, pos, scale=0.55, color=(255, 255, 255), thickness=1):
    """Draws text with a sharp black outline so it remains 100% readable on any transparent background."""
    x, y = pos
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 3, cv2.LINE_AA)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


class FawBotUDPClient:
    """Non-blocking UDP communication handler for FawBot firmware."""

    def __init__(self, host=DEFAULT_ROBOT_HOST, port=DEFAULT_UDP_PORT, fallback_ip=DEFAULT_FALLBACK_IP):
        self.host = host
        self.port = port
        self.fallback_ip = fallback_ip
        self.ip = self._resolve_ip()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.target_addr = (self.ip, self.port)
        self.last_sent_cmd = None
        self.last_sent_time = 0
        self.last_robot_alert = ""
        print(f"[UDP] Connected to robot at {self.host} ({self.ip}:{self.port})")

    def _resolve_ip(self):
        try:
            resolved = socket.gethostbyname(self.host)
            print(f"[UDP] Resolved mDNS {self.host} -> {resolved}")
            return resolved
        except Exception as e:
            print(f"[UDP] mDNS resolution failed for '{self.host}' ({e}). Falling back to {self.fallback_ip}")
            return self.fallback_ip

    def send(self, cmd_str, force=False):
        """Sends a UDP datagram to the robot, rate-limited unless forced."""
        now = time.time()
        if not force and cmd_str == self.last_sent_cmd and (now - self.last_sent_time) < CMD_SEND_INTERVAL_SEC:
            return

        try:
            self.sock.sendto(cmd_str.encode("utf-8"), self.target_addr)
            self.last_sent_cmd = cmd_str
            self.last_sent_time = now
        except Exception as e:
            print(f"[UDP Error] Send failed: {e}")

    def emergency_stop(self):
        """Sends immediate STOP to abort motors."""
        for _ in range(2):
            self.send("STOP", force=True)
            time.sleep(0.01)

    def poll_feedback(self):
        """Drains incoming UDP packets (e.g. alerts, confirmations)."""
        while True:
            try:
                data, _ = self.sock.recvfrom(256)
                msg = data.decode("utf-8", errors="ignore").strip()
                if msg:
                    self.last_robot_alert = msg
                    print(f"[Robot Feedback] <- {msg}")
            except (BlockingIOError, socket.error):
                break


class OverheadCameraController:
    """Overhead camera tracking and waypoint visual servoing."""

    def __init__(self, cam_idx=DEFAULT_CAMERA_INDEX, robot_id=DEFAULT_ROBOT_ID,
                 robot_host=DEFAULT_ROBOT_HOST, udp_port=DEFAULT_UDP_PORT, fullscreen=True):
        self.robot_id = robot_id
        self.udp = FawBotUDPClient(host=robot_host, port=udp_port)

        # Video Capture
        self.cap = cv2.VideoCapture(cam_idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        if not self.cap.isOpened():
            print(f"[Error] Could not open camera index {cam_idx}. Trying index 1...")
            self.cap = cv2.VideoCapture(1)
            if not self.cap.isOpened():
                print("[Error] No usable video capture device found.")
                sys.exit(1)

        # ArUco Detectors (Setup with multiple dictionaries)
        self.dict_names = list(SUPPORTED_DICTS.keys())
        self.active_dict_idx = 0  # Starts with DICT_ARUCO_ORIGINAL
        self.detectors = {}
        self._init_detectors()

        # Navigation State
        self.waypoints = []              # List of (x, y) target coordinates
        self.is_navigating = False       # Flag for autonomous tracking
        self.robot_pos = None            # (x, y) center
        self.robot_angle = None          # Degrees
        self.robot_corners = None        # 4 corners
        self.state_text = "CLICK TO SET TARGET"
        self.state_color = (0, 255, 255) # Yellow

        # Controller state memory
        self.turning = False             # True if in a turn alignment phase
        self.turn_dir = None             # 'R' or 'L'
        self.motion_cmd = "S"
        self.history_trail = deque(maxlen=60) # Visual path breadcrumbs

        # Window & Mouse Setup (Full Screen by default)
        self.window_name = f"FawBot Overhead Navigation (Robot ID: {self.robot_id})"
        self.is_fullscreen = fullscreen

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        if self.is_fullscreen:
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        else:
            cv2.resizeWindow(self.window_name, 1280, 720)

        cv2.setMouseCallback(self.window_name, self._on_mouse)

    def _init_detectors(self):
        params = aruco.DetectorParameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        for name, dict_val in SUPPORTED_DICTS.items():
            adict = aruco.getPredefinedDictionary(dict_val)
            self.detectors[name] = aruco.ArucoDetector(adict, params)

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            shift_pressed = (flags & cv2.EVENT_FLAG_SHIFTKEY) != 0
            if shift_pressed:
                # Add waypoint to queue
                self.waypoints.append((x, y))
                print(f"[Target] Added waypoint #{len(self.waypoints)} at ({x}, {y})")
            else:
                # Single click sets primary destination
                self.waypoints = [(x, y)]
                print(f"[Target] Target set to ({x}, {y})")

            self.is_navigating = True
            self.turning = False

        elif event == cv2.EVENT_RBUTTONDOWN:
            # Right click clears targets
            self.waypoints = []
            self.is_navigating = False
            self.udp.send("S", force=True)
            self.state_text = "CLEARED BY MOUSE"
            self.state_color = (200, 200, 200)

    def toggle_fullscreen(self):
        """Toggles between fullscreen and normal windowed view."""
        self.is_fullscreen = not self.is_fullscreen
        prop = cv2.WINDOW_FULLSCREEN if self.is_fullscreen else cv2.WINDOW_NORMAL
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, prop)
        if not self.is_fullscreen:
            cv2.resizeWindow(self.window_name, 1280, 720)
        print(f"[UI] Fullscreen: {'ON' if self.is_fullscreen else 'OFF'}")

    def cycle_dictionary(self):
        self.active_dict_idx = (self.active_dict_idx + 1) % len(self.dict_names)
        active_name = self.dict_names[self.active_dict_idx]
        print(f"[ArUco] Switched active dictionary to: {active_name}")

    def detect_robot(self, frame):
        """Attempts detection using the active dictionary, with auto-scan fallback."""
        active_name = self.dict_names[self.active_dict_idx]
        corners, ids, _ = self.detectors[active_name].detectMarkers(frame)

        # If not found in active dictionary, search other dictionaries
        if ids is None or self.robot_id not in ids.flatten():
            for idx, name in enumerate(self.dict_names):
                if name == active_name:
                    continue
                c, i, _ = self.detectors[name].detectMarkers(frame)
                if i is not None and self.robot_id in i.flatten():
                    self.active_dict_idx = idx
                    active_name = name
                    corners, ids = c, i
                    print(f"[ArUco Auto-Detect] Found Robot ID {self.robot_id} in {active_name}!")
                    break

        if ids is not None and self.robot_id in ids.flatten():
            idx = np.where(ids.flatten() == self.robot_id)[0][0]
            c = corners[idx][0]
            center = np.mean(c, axis=0)

            # Orientation vector
            front = (c[0] + c[1]) / 2.0
            back = (c[2] + c[3]) / 2.0
            heading_deg = math.degrees(math.atan2(front[1] - back[1], front[0] - back[0]))

            self.robot_pos = (float(center[0]), float(center[1]))
            self.robot_angle = heading_deg
            self.robot_corners = c.astype(int)
            self.history_trail.append(self.robot_pos)
            return True

        self.robot_pos = None
        self.robot_angle = None
        self.robot_corners = None
        return False

    def update_navigation(self):
        """Closed-loop visual servoing state machine."""
        if not self.is_navigating or not self.waypoints:
            if self.motion_cmd != "S":
                self.motion_cmd = "S"
                self.udp.send("S", force=True)
            if not self.waypoints:
                self.state_text = "READY - CLICK ANYWHERE TO DRIVE"
                self.state_color = (255, 255, 255)
            return

        if self.robot_pos is None:
            if self.motion_cmd != "S":
                self.motion_cmd = "S"
                self.udp.send("S", force=True)
            self.state_text = f"SEARCHING FOR MARKER ID {self.robot_id}..."
            self.state_color = (0, 0, 255)
            return

        target_pt = self.waypoints[0]
        dx = target_pt[0] - self.robot_pos[0]
        dy = target_pt[1] - self.robot_pos[1]
        dist_px = math.hypot(dx, dy)

        # 1. ARRIVAL CHECK
        if dist_px < ARRIVAL_THRESHOLD_PX:
            print(f"[Navigation] Reached waypoint {target_pt} (dist: {dist_px:.1f}px)")
            self.waypoints.pop(0)
            self.turning = False

            if not self.waypoints:
                self.is_navigating = False
                self.motion_cmd = "S"
                self.udp.send("S", force=True)
                self.state_text = "GOAL REACHED!"
                self.state_color = (0, 255, 0)
            else:
                self.state_text = f"PROCEEDING TO NEXT WAYPOINT ({len(self.waypoints)} REMAINING)"
                self.state_color = (255, 255, 0)
            return

        # 2. HEADING ALIGNMENT
        target_angle = math.degrees(math.atan2(dy, dx))
        error_angle = (target_angle - self.robot_angle + 180.0) % 360.0 - 180.0

        # 3. HYSTERESIS CONTROL
        if not self.turning:
            if abs(error_angle) > ALIGN_ENTER_DEG:
                self.turning = True
                self.turn_dir = "R" if error_angle > 0 else "L"
        else:
            if abs(error_angle) <= ALIGN_EXIT_DEG:
                self.turning = False
                self.turn_dir = None
            else:
                self.turn_dir = "R" if error_angle > 0 else "L"

        # 4. EXECUTE COMMAND
        if self.turning:
            self.motion_cmd = self.turn_dir
            self.udp.send(self.motion_cmd)
            self.state_text = f"ROTATING {'RIGHT' if self.turn_dir == 'R' else 'LEFT'} (Err: {error_angle:+.1f} deg)"
            self.state_color = (0, 165, 255)
        else:
            self.motion_cmd = "F"
            self.udp.send("F")
            self.state_text = f"DRIVING FORWARD (Dist: {dist_px:.0f}px, Err: {error_angle:+.1f} deg)"
            self.state_color = (0, 255, 0)

    def draw_hud(self, frame):
        """Draws visual cues, trajectory, target, and telemetry overlay with transparent backgrounds."""
        # 1. Draw Breadcrumb Trail
        trail_pts = list(self.history_trail)
        for i in range(1, len(trail_pts)):
            pt1 = (int(trail_pts[i - 1][0]), int(trail_pts[i - 1][1]))
            pt2 = (int(trail_pts[i][0]), int(trail_pts[i][1]))
            alpha = i / len(trail_pts)
            color = (int(120 * alpha), int(255 * alpha), int(255 * alpha))
            cv2.line(frame, pt1, pt2, color, 2)

        # 2. Draw Robot Visual Overlay
        if self.robot_pos is not None and self.robot_corners is not None:
            cv2.polylines(frame, [self.robot_corners], True, (0, 255, 0), 2)

            rx, ry = int(self.robot_pos[0]), int(self.robot_pos[1])
            cv2.circle(frame, (rx, ry), 8, (0, 0, 0), -1)
            cv2.circle(frame, (rx, ry), 6, (0, 255, 255), -1)

            rad = math.radians(self.robot_angle)
            arrow_len = 50
            tip_x = int(rx + arrow_len * math.cos(rad))
            tip_y = int(ry + arrow_len * math.sin(rad))
            cv2.arrowedLine(frame, (rx, ry), (tip_x, tip_y), (0, 0, 255), 3, tipLength=0.35)

            draw_outlined_text(frame, f"FawBot #{self.robot_id} ({self.robot_angle:.0f} deg)",
                               (rx - 40, ry - 20), scale=0.55, color=(255, 255, 255), thickness=2)

        # 3. Draw Target Points & Path
        if self.waypoints:
            if self.robot_pos is not None:
                cv2.line(frame, (int(self.robot_pos[0]), int(self.robot_pos[1])),
                         self.waypoints[0], (0, 255, 255), 2, cv2.LINE_AA)

            for i in range(len(self.waypoints) - 1):
                cv2.line(frame, self.waypoints[i], self.waypoints[i + 1], (255, 120, 0), 2, cv2.LINE_AA)

            for i, pt in enumerate(self.waypoints):
                is_active = (i == 0)
                ring_color = (0, 255, 0) if is_active else (255, 0, 255)
                cv2.circle(frame, pt, ARRIVAL_THRESHOLD_PX, ring_color, 2 if is_active else 1, cv2.LINE_AA)
                cv2.circle(frame, pt, 8, (0, 0, 0), -1)
                cv2.circle(frame, pt, 5, ring_color, -1)
                cv2.line(frame, (pt[0] - 12, pt[1]), (pt[0] + 12, pt[1]), ring_color, 2)
                cv2.line(frame, (pt[0], pt[1] - 12), (pt[0], pt[1] + 12), ring_color, 2)
                label = f"Target" if is_active else f"WP #{i + 1}"
                draw_outlined_text(frame, label, (pt[0] + 15, pt[1] - 10), scale=0.60, color=ring_color, thickness=2)

        # 4. Top Status Header (100% Transparent Background with outlined text)
        active_dict_str = self.dict_names[self.active_dict_idx]
        draw_outlined_text(frame, f"FawBot Vision Navigation | Marker: ID {self.robot_id} | [{active_dict_str}]",
                           (22, 34), scale=0.58, color=(210, 210, 210), thickness=1)

        draw_outlined_text(frame, self.state_text, (22, 65), scale=0.75, color=self.state_color, thickness=2)

        conn_text = f"Target IP: {self.udp.ip}:{self.udp.port} | Last Cmd: {self.motion_cmd}"
        if self.udp.last_robot_alert:
            conn_text += f" | Alert: {self.udp.last_robot_alert}"
        draw_outlined_text(frame, conn_text, (22, 95), scale=0.50, color=(190, 190, 190), thickness=1)

        # 5. Bottom Controls Legend (100% Transparent Background with outlined text)
        h, w = frame.shape[:2]
        controls_str = "[Left Click]: Set Target  |  [Shift+Click]: Add WP  |  [C]: Clear  |  [S/Space]: STOP  |  [F]: Fullscreen  |  [D]: Dict  |  [Q/Esc]: Quit"
        draw_outlined_text(frame, controls_str, (22, h - 20), scale=0.48, color=(230, 230, 230), thickness=1)

    def run(self):
        """Main control loop."""
        print("\n=======================================================")
        print(" FawBot Overhead Camera Controller Started")
        print("-------------------------------------------------------")
        print(" Controls:")
        print("  - Left Click        : Set target and navigate robot")
        print("  - Shift + Left Click: Append waypoint to path")
        print("  - 'S' or SPACE      : Emergency STOP")
        print("  - 'C'               : Clear target / stop navigation")
        print("  - 'F'               : Toggle Fullscreen on / off")
        print("  - 'D'               : Cycle ArUco dictionary")
        print("  - 'Q' or ESC        : Quit")
        print("=======================================================\n")

        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("[Warning] Camera frame drop or disconnect. Retrying...")
                    time.sleep(0.05)
                    continue

                self.udp.poll_feedback()
                self.detect_robot(frame)
                self.update_navigation()
                self.draw_hud(frame)

                cv2.imshow(self.window_name, frame)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):  # 'Q' or ESC
                    print("\n[Controller] Quitting cleanly...")
                    self.udp.emergency_stop()
                    break
                elif key in (ord('s'), 32):  # 'S' or SPACE
                    print("[Controller] Emergency STOP triggered by keyboard!")
                    self.is_navigating = False
                    self.waypoints = []
                    self.udp.emergency_stop()
                    self.state_text = "EMERGENCY STOPPED!"
                    self.state_color = (0, 0, 255)
                elif key == ord('c'):
                    print("[Controller] Targets cleared.")
                    self.is_navigating = False
                    self.waypoints = []
                    self.udp.send("S", force=True)
                    self.state_text = "TARGET CLEARED"
                    self.state_color = (255, 255, 255)
                elif key in (ord('f'), ord('F')):
                    self.toggle_fullscreen()
                elif key == ord('d'):
                    self.cycle_dictionary()

        except KeyboardInterrupt:
            print("\n[Controller] Interrupted by user.")
        finally:
            self.udp.emergency_stop()
            self.cap.release()
            cv2.destroyAllWindows()
            print("[Controller] Shutdown cleanly.")


def parse_args():
    parser = argparse.ArgumentParser(description="FawBot Overhead Camera ArUco Navigation")
    parser.add_argument("--camera", type=int, default=DEFAULT_CAMERA_INDEX, help="Camera device index (default: 0)")
    parser.add_argument("--windowed", action="store_true", help="Launch in windowed mode instead of fullscreen")
    parser.add_argument("--id", type=int, default=DEFAULT_ROBOT_ID, help="ArUco Marker ID for robot (default: 23)")
    parser.add_argument("--host", type=str, default=DEFAULT_ROBOT_HOST, help="Robot hostname/mDNS (default: fawbot_23.local)")
    parser.add_argument("--ip", type=str, default=DEFAULT_FALLBACK_IP, help="Robot fallback IP if mDNS fails (default: 192.168.1.13)")
    parser.add_argument("--port", type=int, default=DEFAULT_UDP_PORT, help="Robot UDP port (default: 8888)")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    controller = OverheadCameraController(
        cam_idx=args.camera,
        robot_id=args.id,
        robot_host=args.host,
        udp_port=args.port,
        fullscreen=(not args.windowed)
    )
    controller.run()
