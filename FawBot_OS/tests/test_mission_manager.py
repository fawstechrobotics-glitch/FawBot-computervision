"""Unit tests for MissionManager and end-to-end mission workflows."""
import sys
import unittest
from PyQt5.QtWidgets import QApplication

from navigation.mission_manager import MissionManager
from robot.robot_controller import RobotController
from robot.robot_state import RobotState
from robot.udp_communication import UDPCommunication
from models.obstacle import RestrictedArea
from models.path import PathPoint


class TestMissionManager(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize QApplication once for Qt signals/slots
        if not QApplication.instance():
            cls.app = QApplication(sys.argv)
        else:
            cls.app = QApplication.instance()

    def setUp(self):
        self.comm = UDPCommunication()
        self.state = RobotState()
        self.controller = RobotController(self.comm, self.state)
        self.manager = MissionManager(self.controller, self.state)

    def tearDown(self):
        self.comm.close()

    def test_new_mission(self):
        mission = self.manager.new_mission("warehouse_test")
        self.assertEqual(mission.metadata.name, "warehouse_test")
        self.assertEqual(self.manager.active_mission.metadata.name, "warehouse_test")

    def test_teach_mode_lifecycle(self):
        self.state.set_pose(10.0, 10.0, 0.0)
        self.manager.start_teach_mode()
        self.assertTrue(self.manager.is_recording)

        # Record driving events
        self.manager.record_drive_step(20.0, 10.0, 0.0, action="MOVE")
        self.manager.record_drive_step(20.0, 30.0, 90.0, action="MOVE")

        path = self.manager.stop_teach_mode()
        self.assertFalse(self.manager.is_recording)
        self.assertTrue(len(path.points) >= 3)
        self.assertTrue(len(self.manager.active_mission.commands) >= 2)

    def test_validation_detects_collision(self):
        self.manager.new_mission("collision_test")
        self.manager.active_mission.environment.restricted_areas.append(
            RestrictedArea(
                id="restricted_box",
                name="Forbidden Zone",
                polygon=[(40, 40), (80, 40), (80, 80), (40, 80)]
            )
        )
        # Path intersecting the forbidden zone: (20, 60) -> (100, 60)
        self.manager.active_mission.path.append(PathPoint(20.0, 60.0, 0.0, action="START"))
        self.manager.active_mission.path.append(PathPoint(100.0, 60.0, 0.0, action="MOVE", distance=80.0))

        is_valid, errors = self.manager.validate_mission()
        self.assertFalse(is_valid)
        self.assertTrue(len(errors) > 0)
        self.assertTrue(any("Forbidden Zone" in err for err in errors))


if __name__ == "__main__":
    unittest.main()
