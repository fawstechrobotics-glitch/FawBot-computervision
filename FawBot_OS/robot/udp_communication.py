"""UDP Network Communication layer for hardware robot interfacing."""
import socket
import logging
from PyQt5.QtCore import QObject, QThread, pyqtSignal

import config.settings as settings

logger = logging.getLogger(__name__)


class UDPListenerThread(QThread):
    """Background thread to receive UDP telemetry and feedback from the robot."""
    feedback_received = pyqtSignal(str)
    peer_discovered = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, sock: socket.socket):
        super().__init__()
        self.sock = sock
        self._running = True

    def run(self):
        logger.info("UDP Listener thread started.")
        while self._running:
            try:
                data, addr = self.sock.recvfrom(1024)
                if data:
                    message = data.decode("utf-8", errors="replace").strip()
                    logger.debug(f"UDP RX raw: {message} from {addr}")
                    if addr and len(addr) > 0:
                        self.peer_discovered.emit(addr[0])
                    self.feedback_received.emit(message)
            except socket.timeout:
                continue
            except OSError as e:
                # Socket closed during shutdown
                if not self._running:
                    break
                logger.error(f"Socket OSError in listener: {e}")
                self.error_occurred.emit(str(e))
                break
            except Exception as e:
                logger.error(f"Unexpected listener exception: {e}")
                self.error_occurred.emit(str(e))
                break
        logger.info("UDP Listener thread finished.")

    def stop(self):
        self._running = False


class UDPCommunication(QObject):
    """Encapsulates socket management, sending commands, and listening for feedback."""
    feedback_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)
    connection_status_changed = pyqtSignal(bool)

    def __init__(self, host: str = settings.ROBOT_HOST, port: int = settings.UDP_PORT):
        super().__init__()
        self.host = host
        self.port = port
        self.sock = None
        self.listener_thread = None
        self._is_connected = False
        self.last_robot_ip = None
        self.connect_socket()

    def connect_socket(self) -> bool:
        """Create high-speed UDP socket and spawn listener thread."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(settings.SOCKET_TIMEOUT)
            self._is_connected = True

            self.listener_thread = UDPListenerThread(self.sock)
            self.listener_thread.feedback_received.connect(self.feedback_received)
            self.listener_thread.peer_discovered.connect(self._on_peer_discovered)
            self.listener_thread.error_occurred.connect(self.error_occurred)
            self.listener_thread.start()

            self.connection_status_changed.emit(True)
            logger.info(f"UDP socket ready for {self.host}:{self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize UDP socket: {e}")
            self._is_connected = False
            self.connection_status_changed.emit(False)
            self.error_occurred.emit(str(e))
            return False

    def _on_peer_discovered(self, ip_str: str):
        if self.last_robot_ip != ip_str:
            self.last_robot_ip = ip_str
            logger.info(f"Discovered direct physical robot IP: {self.last_robot_ip}")

    def send_command(self, cmd_str: str) -> bool:
        """Send a formatted command string to the robot over UDP."""
        if not self.sock:
            logger.warning(f"Cannot send command '{cmd_str}': Socket not initialized")
            return False

        try:
            dest_host = self.last_robot_ip or self.host
            target_addr = (dest_host, self.port)
            logger.info(f"UDP TX -> {target_addr}: {cmd_str}")
            self.sock.sendto(cmd_str.encode("utf-8"), target_addr)
            return True
        except Exception as e:
            logger.error(f"UDP Send Error for '{cmd_str}': {e}")
            self.error_occurred.emit(f"Send failed: {e}")
            return False

    def close(self):
        """Cleanly terminate listener thread and close UDP socket."""
        logger.info("Closing UDP communication...")
        self._is_connected = False
        if self.listener_thread:
            self.listener_thread.stop()
            self.listener_thread.wait(1000)
            self.listener_thread = None

        if self.sock:
            try:
                self.sock.close()
            except Exception as e:
                logger.error(f"Error closing socket: {e}")
            self.sock = None

        self.connection_status_changed.emit(False)
