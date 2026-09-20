"""Gazebo Simulation Workbench Dark Theme Styling for PyQt5 and Matplotlib."""

# Matplotlib dark theme configuration
MATPLOTLIB_STYLE = {
    'figure.facecolor': '#121212',
    'axes.facecolor': '#121212',
    'axes.edgecolor': '#444444',
    'axes.labelcolor': '#aaaaaa',
    'xtick.color': '#888888',
    'ytick.color': '#888888',
    'grid.color': '#282828',
    'text.color': '#dcdcdc'
}

# PyQt5 Gazebo Dark Stylesheet
GAZEBO_DARK_STYLESHEET = """
QMainWindow {
    background-color: #161616;
}
QWidget {
    background-color: #1a1a1a;
    color: #dcdcdc;
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif;
    font-size: 11px;
}
QTabWidget::pane {
    border: 1px solid #333333;
    background-color: #1a1a1a;
    top: -1px;
}
QTabBar::tab {
    background-color: #242424;
    color: #aaaaaa;
    border: 1px solid #333;
    padding: 8px 16px;
    font-weight: bold;
    font-size: 12px;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
    margin-right: 2px;
}
QTabBar::tab:selected {
    background-color: #0f141d;
    color: #00d2ff;
    border-bottom: 2px solid #00d2ff;
}
QTabBar::tab:hover:!selected {
    background-color: #2e2e2e;
    color: #ffffff;
}
QGroupBox {
    border: 1px solid #383838;
    border-radius: 6px;
    margin-top: 8px;
    font-weight: bold;
    color: #00d2ff;
    padding-top: 10px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 8px;
    padding: 0 4px;
}
QLabel {
    font-size: 11px;
}
QPushButton {
    background-color: #2b2b2b;
    border: 1px solid #444444;
    border-radius: 4px;
    padding: 5px 10px;
    font-weight: bold;
    color: #e0e0e0;
    min-height: 20px;
}
QPushButton:hover {
    background-color: #383838;
    border-color: #00d2ff;
    color: #ffffff;
}
QPushButton:pressed {
    background-color: #111111;
}
QPushButton:disabled {
    background-color: #1e1e1e;
    color: #555555;
    border-color: #282828;
}
QPushButton#primaryBtn {
    background-color: #1e5c2b;
    color: #ffffff;
    border: 1px solid #2e8540;
}
QPushButton#primaryBtn:hover {
    background-color: #267336;
    border-color: #00ffaa;
}
QPushButton#dangerBtn {
    background-color: #631e1e;
    color: #ffffff;
    border: 1px solid #852e2e;
}
QPushButton#dangerBtn:hover {
    background-color: #7a2525;
}
QPushButton#warningBtn {
    background-color: #8a5300;
    color: #ffffff;
    border: 1px solid #b36b00;
}
QPushButton#warningBtn:hover {
    background-color: #a66400;
}
QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox {
    background-color: #242424;
    border: 1px solid #444444;
    color: #00ffaa;
    font-weight: bold;
    padding: 3px 6px;
    border-radius: 4px;
    selection-background-color: #00d2ff;
    selection-color: #111;
}
QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus, QComboBox:focus {
    border-color: #00d2ff;
}
QListWidget {
    background-color: #181818;
    border: 1px solid #333333;
    border-radius: 4px;
    color: #e0e0e0;
    padding: 4px;
}
QListWidget::item {
    padding: 4px;
    border-radius: 3px;
}
QListWidget::item:selected {
    background-color: #0f2b3c;
    color: #00d2ff;
}
#statusCard {
    background-color: #0f141d;
    border: 1px solid #00d2ff;
    border-radius: 6px;
    padding: 6px;
}
#telemetryText {
    color: #00ffaa;
    font-family: monospace;
    font-size: 12px;
}
#mouseCoordText {
    color: #ffcc00;
    font-family: monospace;
    font-size: 12px;
    font-weight: bold;
}
QScrollArea {
    border: none;
    background-color: transparent;
}
QScrollBar:vertical {
    border: none;
    background: #181818;
    width: 8px;
    margin: 0px;
}
QScrollBar::handle:vertical {
    background: #333333;
    min-height: 20px;
    border-radius: 4px;
}
QScrollBar::handle:vertical:hover {
    background: #00d2ff;
}
QProgressBar {
    border: 1px solid #444;
    border-radius: 4px;
    text-align: center;
    color: white;
    font-weight: bold;
    background-color: #222;
}
QProgressBar::chunk {
    background-color: #00d2ff;
    border-radius: 3px;
}
"""
