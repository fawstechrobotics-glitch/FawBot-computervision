"""Robot package containing communication, state, controller, and fleet classes."""

__all__ = ["UDPCommunication", "RobotState", "RobotController"]


def __getattr__(name):
	"""Load Qt-dependent classes only when a caller actually requests them."""
	if name == "UDPCommunication":
		from robot.udp_communication import UDPCommunication
		return UDPCommunication
	if name == "RobotState":
		from robot.robot_state import RobotState
		return RobotState
	if name == "RobotController":
		from robot.robot_controller import RobotController
		return RobotController
	raise AttributeError(name)
