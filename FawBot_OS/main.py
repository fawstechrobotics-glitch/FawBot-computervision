"""FawBot OS - Main Application Entry Point."""
import sys
import os
import logging
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import config.settings as settings
from ui.main_window import MainWindow


def setup_logging():
    """Configure structured file and console logging."""
    log_file = os.path.join(settings.LOGS_DIR, "fawbot.log")
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] (%(name)s) %(message)s",
        handlers=[
            logging.FileHandler(log_file, encoding="utf-8"),
            logging.StreamHandler(sys.stdout)
        ]
    )
    logging.info("==========================================")
    logging.info("Starting FawBot OS Application...")
    logging.info(f"Target Robot: {settings.ROBOT_HOST}:{settings.UDP_PORT}")
    logging.info("==========================================")


def main():
    """Initialize GUI application and event loop."""
    setup_logging()

    # Enable High-DPI scaling for Retina and 4K displays
    if hasattr(Qt, "AA_EnableHighDpiScaling"):
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, "AA_UseHighDpiPixmaps"):
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    window = MainWindow()
    window.showMaximized()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
