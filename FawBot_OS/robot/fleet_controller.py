"""Owns the independent communication and state channels for all robots."""
from typing import Dict, Optional

from robot.fleet import RobotRoster, RobotSpec
from robot.robot_controller import RobotController
from robot.robot_state import RobotState
from robot.udp_communication import UDPCommunication


class RobotFleet:
    """Build one controller per configured robot without sharing UDP state."""

    def __init__(self, roster: RobotRoster):
        self.roster = roster
        self.comms: Dict[str, UDPCommunication] = {}
        self.states: Dict[str, RobotState] = {}
        self.controllers: Dict[str, RobotController] = {}
        for robot in roster.robots:
            self._add_robot(robot)

    def _add_robot(self, robot: RobotSpec):
        comm = UDPCommunication(host=robot.host, port=robot.port)
        state = RobotState()
        state.set_pose(robot.start_x, robot.start_y, robot.heading, set_initial_if_unset=True)
        self.comms[robot.robot_id] = comm
        self.states[robot.robot_id] = state
        self.controllers[robot.robot_id] = RobotController(comm, state)

    @property
    def robot_ids(self):
        return list(self.controllers)

    @property
    def primary_id(self) -> Optional[str]:
        return self.robot_ids[0] if self.robot_ids else None

    @property
    def primary_controller(self) -> Optional[RobotController]:
        return self.controllers.get(self.primary_id) if self.primary_id else None

    @property
    def primary_state(self) -> Optional[RobotState]:
        return self.states.get(self.primary_id) if self.primary_id else None

    def close(self):
        for comm in self.comms.values():
            comm.close()