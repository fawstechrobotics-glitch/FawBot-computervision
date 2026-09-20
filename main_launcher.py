#!/usr/bin/env python3
"""
FawBot Multi-Script Launcher & Optimized Web Dashboard
======================================================
Features:
  1. Full-screen optimized web layout targeting responsive viewports.
  2. Integrated mDNS robot address management (saved_robots.json).
  3. Dynamic script execution suite for local OpenCV controls.
"""

import sys
import os
import json
import webbrowser
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QLabel, QComboBox, QPushButton, QTextEdit, QGroupBox, QTabWidget,
    QLineEdit, QSizePolicy
)
from PyQt5.QtCore import QProcess, Qt, QUrl
from PyQt5.QtGui import QFont, QTextCursor

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(BASE_DIR, "saved_robots.json")
FAWBOT_OS_DIR = os.path.join(BASE_DIR, "FawBot_OS")
if FAWBOT_OS_DIR not in sys.path:
    sys.path.insert(0, FAWBOT_OS_DIR)

try:
    from ui.main_window import MainWindow as FawBotOSWindow
    FAWBOT_OS_AVAILABLE = True
except Exception as exc:
    FawBotOSWindow = None
    FAWBOT_OS_AVAILABLE = False
    FAWBOT_OS_IMPORT_ERROR = exc

try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    from PyQt5.QtWebEngineWidgets import QWebEngineSettings
    WEBENGINE_AVAILABLE = True
except ImportError:
    WEBENGINE_AVAILABLE = False

def scan_available_scripts():
    """Scans the directory for runnable .py scripts."""
    ignore_files = {"main_launcher.py", "__init__.py"}
    scripts = {}

    if os.path.exists(BASE_DIR):
        for file in sorted(os.listdir(BASE_DIR)):
            if file.endswith(".py") and file not in ignore_files:
                display_name = file[:-3].replace("_", " ").title()
                scripts[display_name] = os.path.join(BASE_DIR, file)

    return scripts


def load_saved_robots():
    """Loads saved robot numbers from JSON configuration file."""
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                data = json.load(f)
                return sorted(list(set(data.get("robots", []))))
        except Exception:
            return ["01"]
    return ["01"]


def save_robots_to_file(robot_list):
    """Saves robot numbers array to JSON configuration file."""
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump({"robots": sorted(list(set(robot_list)))}, f, indent=4)
    except Exception as e:
        print(f"Error saving config: {e}")


def build_mdns_url(robot_num):
    """Generates mDNS URL formatted as: http://fawbot_{num}.local/Fawbot"""
    formatted_num = str(robot_num).zfill(2) if len(str(robot_num)) == 1 else str(robot_num)
    return f"http://fawbot_{formatted_num}.local/Fawbot"


class FawBotLauncher(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FawBot Central Control Suite & Web Dashboard")
        self.resize(1280, 800)

        self.process = None
        self.script_registry = {}
        self.saved_robots = load_saved_robots()

        self.init_ui()
        self.refresh_script_list()
        self.update_robot_dropdown()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(4, 4, 4, 4)

        # Main Tab Widget
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        # Tab 1: Python Script Launcher
        self.script_tab = QWidget()
        self.build_script_tab()
        self.tabs.addTab(self.script_tab, "Python Script Launcher")

        # Tab 2: Embedded Robot Web Interface
        self.web_tab = QWidget()
        self.build_web_tab()
        self.tabs.addTab(self.web_tab, "Robot Web Interface")

        # Tab 3: FawBot OS application
        self.build_fawbot_os_tab()

    def build_script_tab(self):
        layout = QVBoxLayout(self.script_tab)

        title_label = QLabel("FawBot Python Script Execution Suite")
        title_font = QFont("Arial", 14)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        selection_group = QGroupBox("Mode Selection")
        selection_layout = QHBoxLayout()

        combo_label = QLabel("Select Operating Mode:")
        self.script_combo = QComboBox()

        self.btn_refresh = QPushButton("Refresh List")
        self.btn_refresh.setFixedWidth(100)
        self.btn_refresh.clicked.connect(self.refresh_script_list)

        selection_layout.addWidget(combo_label)
        selection_layout.addWidget(self.script_combo, stretch=1)
        selection_layout.addWidget(self.btn_refresh)
        selection_group.setLayout(selection_layout)
        layout.addWidget(selection_group)

        button_layout = QHBoxLayout()

        self.btn_start = QPushButton("Start Selected Mode")
        self.btn_start.setStyleSheet(
            "background-color: #2e7d32; color: white; font-weight: bold; padding: 10px; border-radius: 4px;"
        )
        self.btn_start.clicked.connect(self.start_script)

        self.btn_stop = QPushButton("Stop Mode")
        self.btn_stop.setEnabled(False)
        self.btn_stop.setStyleSheet(
            "background-color: #c62828; color: white; font-weight: bold; padding: 10px; border-radius: 4px;"
        )
        self.btn_stop.clicked.connect(self.stop_script)

        button_layout.addWidget(self.btn_start)
        button_layout.addWidget(self.btn_stop)
        layout.addLayout(button_layout)

        log_group = QGroupBox("Console Output Log")
        log_layout = QVBoxLayout()

        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setStyleSheet(
            "background-color: #1e1e1e; color: #00ff00; font-family: monospace; font-size: 12px;"
        )
        log_layout.addWidget(self.log_output)

        log_group.setLayout(log_layout)
        layout.addWidget(log_group)

    def build_web_tab(self):
        layout = QVBoxLayout(self.web_tab)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        # Compact Combined Top Control Bar
        top_bar = QHBoxLayout()
        top_bar.setSpacing(8)

        # Robot Fleet Selector & Management
        nav_label = QLabel("Robot:")
        self.robot_combo = QComboBox()
        self.robot_combo.setMinimumWidth(110)
        self.robot_combo.currentIndexChanged.connect(self.on_robot_selected)

        num_label = QLabel("Add Num:")
        self.num_input = QLineEdit()
        self.num_input.setPlaceholderText("e.g. 01")
        self.num_input.setFixedWidth(60)

        self.btn_add_robot = QPushButton("+ Add")
        self.btn_add_robot.clicked.connect(self.add_robot)

        self.btn_remove_robot = QPushButton("Delete")
        self.btn_remove_robot.clicked.connect(self.remove_robot)

        self.url_display = QLineEdit()
        self.url_display.setReadOnly(True)

        self.btn_load_page = QPushButton("Reload")
        self.btn_load_page.clicked.connect(self.load_robot_page)

        self.btn_external_browser = QPushButton("Open in Browser")
        self.btn_external_browser.clicked.connect(self.open_external_browser)

        # Assemble compact toolbar
        top_bar.addWidget(nav_label)
        top_bar.addWidget(self.robot_combo)
        top_bar.addWidget(num_label)
        top_bar.addWidget(self.num_input)
        top_bar.addWidget(self.btn_add_robot)
        top_bar.addWidget(self.btn_remove_robot)
        top_bar.addWidget(self.url_display, stretch=1)
        top_bar.addWidget(self.btn_load_page)
        top_bar.addWidget(self.btn_external_browser)

        layout.addLayout(top_bar)

        # Web View Engine expanded to maximum stretch
        if WEBENGINE_AVAILABLE:
            self.web_view = QWebEngineView()
            self.web_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            
            # Web engine setting adjustments for full-page fit
            settings = self.web_view.settings()
            settings.setAttribute(QWebEngineSettings.ShowScrollBars, True)
            settings.setAttribute(QWebEngineSettings.LocalContentCanAccessRemoteUrls, True)
            
            layout.addWidget(self.web_view, stretch=1)
        else:
            fallback_label = QLabel(
                "PyQt5-WebEngine is not installed.\n"
                "Install it using: pip install PyQt5 PyQtWebEngine"
            )
            fallback_label.setAlignment(Qt.AlignCenter)
            fallback_label.setStyleSheet("color: #d32f2f; font-size: 14px; font-weight: bold;")
            layout.addWidget(fallback_label, stretch=1)

    def build_fawbot_os_tab(self):
        """Add the FawBot OS interface as an embedded launcher page."""
        if FAWBOT_OS_AVAILABLE:
            self.fawbot_os_window = FawBotOSWindow()
            self.tabs.addTab(self.fawbot_os_window, "FawBot OS")
            return

        self.fawbot_os_window = None
        unavailable_page = QWidget()
        layout = QVBoxLayout(unavailable_page)
        message = QLabel(
            "FawBot OS could not be loaded.\n"
            f"{FAWBOT_OS_IMPORT_ERROR}\n\n"
            "Install dependencies from FawBot_OS/requirements.txt."
        )
        message.setAlignment(Qt.AlignCenter)
        message.setStyleSheet("color: #d32f2f; font-size: 14px; font-weight: bold;")
        layout.addWidget(message)
        self.tabs.addTab(unavailable_page, "FawBot OS")

    def update_robot_dropdown(self):
        """Populates the dropdown with saved robots and updates the URL display."""
        self.robot_combo.blockSignals(True)
        self.robot_combo.clear()

        for bot_num in self.saved_robots:
            display_str = f"FawBot {bot_num}"
            self.robot_combo.addItem(display_str, userData=bot_num)

        self.robot_combo.blockSignals(False)

        if self.saved_robots:
            self.on_robot_selected()

    def add_robot(self):
        raw_num = self.num_input.text().strip()
        if not raw_num:
            return

        bot_num = raw_num.zfill(2) if len(raw_num) == 1 else raw_num

        if bot_num not in self.saved_robots:
            self.saved_robots.append(bot_num)
            self.saved_robots.sort()
            save_robots_to_file(self.saved_robots)
            self.update_robot_dropdown()

            index = self.robot_combo.findData(bot_num)
            if index >= 0:
                self.robot_combo.setCurrentIndex(index)

            self.num_input.clear()
            self.append_log(f"[SYSTEM] Saved FawBot {bot_num} to config file.\n")

    def remove_robot(self):
        current_num = self.robot_combo.currentData()
        if current_num and current_num in self.saved_robots:
            self.saved_robots.remove(current_num)
            save_robots_to_file(self.saved_robots)
            self.update_robot_dropdown()
            self.append_log(f"[SYSTEM] Removed FawBot {current_num} from config file.\n")

    def on_robot_selected(self):
        bot_num = self.robot_combo.currentData()
        if bot_num:
            url = build_mdns_url(bot_num)
            self.url_display.setText(url)
            self.load_robot_page()

    def load_robot_page(self):
        url_str = self.url_display.text().strip()
        if WEBENGINE_AVAILABLE and url_str:
            self.web_view.setUrl(QUrl(url_str))

    def open_external_browser(self):
        url_str = self.url_display.text().strip()
        if url_str:
            webbrowser.open(url_str)

    def refresh_script_list(self):
        self.script_combo.clear()
        self.script_registry = scan_available_scripts()

        if not self.script_registry:
            self.script_combo.addItem("No scripts found in directory")
            self.btn_start.setEnabled(False)
        else:
            for display_name in self.script_registry.keys():
                self.script_combo.addItem(display_name)
            self.btn_start.setEnabled(True)

        self.append_log(f"[SYSTEM] Scanned directory. Found {len(self.script_registry)} Python script(s).\n")

    def start_script(self):
        selected_mode = self.script_combo.currentText()
        script_path = self.script_registry.get(selected_mode)

        if not script_path or not os.path.exists(script_path):
            self.append_log(f"[ERROR] Script path '{script_path}' not found!\n")
            return

        script_filename = os.path.basename(script_path)
        self.append_log(f"[LAUNCHING] Executing {selected_mode} ({script_filename})...\n")

        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)

        self.process.readyReadStandardOutput.connect(self.handle_stdout)
        self.process.finished.connect(self.process_finished)

        self.process.setWorkingDirectory(BASE_DIR)

        python_exe = sys.executable
        self.process.start(python_exe, [script_path])

        self.btn_start.setEnabled(False)
        self.btn_refresh.setEnabled(False)
        self.script_combo.setEnabled(False)
        self.btn_stop.setEnabled(True)

    def stop_script(self):
        if self.process and self.process.state() != QProcess.NotRunning:
            self.append_log("[TERMINATING] Requesting process exit...\n")
            self.process.terminate()
            if not self.process.waitForFinished(2000):
                self.process.kill()

    def handle_stdout(self):
        data = self.process.readAllStandardOutput().data().decode("utf-8", errors="replace")
        self.append_log(data)

    def process_finished(self, exit_code, exit_status):
        self.append_log(f"[FINISHED] Process exited with code {exit_code}.\n\n")
        self.btn_start.setEnabled(True)
        self.btn_refresh.setEnabled(True)
        self.script_combo.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.process = None

    def append_log(self, text):
        self.log_output.moveCursor(QTextCursor.End)
        self.log_output.insertPlainText(text)
        self.log_output.moveCursor(QTextCursor.End)

    def closeEvent(self, event):
        self.stop_script()
        if self.fawbot_os_window:
            self.fawbot_os_window.close()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = FawBotLauncher()
    window.show()
    sys.exit(app.exec())