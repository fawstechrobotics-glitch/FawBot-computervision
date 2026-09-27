import unittest

from navigation.lidar_protocol import decode_lidar_packet, parse_lidar_message


class TestLidarMessageParsing(unittest.TestCase):
    def test_scan_packet(self):
        self.assertEqual(
            parse_lidar_message("LIDAR_SCAN:0,10;90,20;-90,5"),
            [(0.0, 10.0), (90.0, 20.0), (-90.0, 5.0)],
        )

    def test_json_packet(self):
        self.assertEqual(
            parse_lidar_message('{"scan":[{"angle":45,"distance_cm":12}]}'),
            [(45.0, 12.0)],
        )

    def test_invalid_packet_is_ignored(self):
        self.assertEqual(parse_lidar_message("COMPLETED:MOVE:10.0"), [])
        self.assertEqual(parse_lidar_message("LIDAR_SCAN:10,-1;bad,20"), [])

    def test_firmware_packet_decoding(self):
        self.assertEqual(decode_lidar_packet("3D01009001"), (0.0, 10.0))
        self.assertIsNone(decode_lidar_packet("COMPLETED"))


from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QObject, pyqtSignal
from ui.lidar_mapping_page import LidarMappingPage
from robot.robot_state import RobotState


class MockComm(QObject):
    feedback_received = pyqtSignal(str)
    connection_status_changed = pyqtSignal(bool)

    def send_command(self, cmd):
        pass


class TestLidarMappingPage(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(["test"])

    def setUp(self):
        self.comm = MockComm()
        self.state = RobotState()
        self.page = LidarMappingPage(host="127.0.0.1", comm=self.comm, state=self.state)

    def tearDown(self):
        self.page.close()

    def test_wall_points_and_segments_persist(self):
        # 1. Feed two points in sweep 1
        self.page.handle_lidar_packet("3D01009001")  # angle 0, 10cm
        self.page.handle_lidar_packet("3D81029001")  # angle 5, 10cm
        self.assertEqual(len(self.page.map_points), 2)

        # 2. Complete wall polygon
        self.page._complete_wall_polygon()
        # Points and segments must persist and not be wiped!
        self.assertEqual(len(self.page.map_points), 2)
        self.assertEqual(len(self.page.last_wall_segments), 1)

        # 3. Sweep 2: stationary robot resweeps same angles
        self.page.handle_lidar_packet("3D01009001")
        self.page.handle_lidar_packet("3D81029001")
        self.assertEqual(len(self.page.map_points), 2)
        self.assertEqual(len(self.page.last_wall_segments), 1)

        # 4. Robot advances: REACHED:LIDAR:10.0:0.0:0.0
        self.page.handle_udp_message("REACHED:LIDAR:10.0:0.0:0.0")
        self.assertEqual(len(self.page.map_points), 2)
        self.assertEqual(len(self.page.last_wall_segments), 1)

        # 5. New scan at new pose adds points to map
        self.page.handle_lidar_packet("3D01009001")
        self.assertEqual(len(self.page.map_points), 3)

        # 6. Map drawing runs without errors
        self.page._draw_map()

        # 7. Path planning routes safely
        self.page.goal_point = (50.0, 50.0)
        route = self.page._plan_path((10.0, 0.0), (50.0, 50.0))
        self.assertGreaterEqual(len(route), 2)

        # 8. Clear scan resets map
        self.page.clear_scan()
        self.assertEqual(len(self.page.map_points), 0)
        self.assertEqual(len(self.page.last_wall_segments), 0)

    def test_simulate_test_walls(self):
        self.page.simulate_test_walls()
        self.assertGreater(len(self.page.map_points), 10)
        self.assertGreater(len(self.page.last_wall_segments), 5)
        self.assertIn("Simulated room", self.page.status_label.text())

    def test_continuous_scan_and_navigation(self):
        self.page.simulate_test_walls()
        self.page.goal_point = (30.0, 30.0)
        self.page._plan_route_to_goal()
        self.assertTrue(len(self.page.route_points) >= 2)
        # Verify controller live animation runs without TypeError
        self.page.controller._update_live_animation()

    def test_clockwise_and_anticlockwise_sweep_directions(self):
        self.page.robot_x = 0.0
        self.page.robot_y = 0.0
        self.page.robot_heading = 90.0
        self.page.sweep_degrees = 180.0
        self.page.LIDAR_STEP_ANGLE_DEG = 1.0

        # Phase 1: Clockwise move (starts from heading ~90 down to 0)
        # angle 0 should map near 89 deg (pointing North/East)
        self.page._process_single_lidar_point(0.0, 50.0)
        pt_0 = self.page.map_points[-1]
        # At 89 deg, x > 0 and y > 0
        self.assertGreater(pt_0[1], 40.0)

        # angle 89 should map to 0 deg (pointing East, +X)
        self.page._process_single_lidar_point(89.0, 50.0)
        pt_89 = self.page.map_points[-1]
        self.assertAlmostEqual(pt_89[0], 50.0, delta=2.0)
        self.assertAlmostEqual(pt_89[1], 5.5, delta=2.0)

        # Phase 2: Anticlockwise move (starts from center 90 up to 180)
        # angle 90 should map to 90 deg (North, +Y)
        self.page._process_single_lidar_point(90.0, 50.0)
        pt_90 = self.page.map_points[-1]
        self.assertAlmostEqual(pt_90[0], 0.0, delta=2.0)
        self.assertAlmostEqual(pt_90[1], 55.5, delta=2.0)

        # angle 180 should map to 180 deg (West, -X)
        self.page._process_single_lidar_point(180.0, 50.0)
        pt_180 = self.page.map_points[-1]
        self.assertAlmostEqual(pt_180[0], -50.0, delta=2.0)
        self.assertAlmostEqual(pt_180[1], 5.5, delta=2.0)

    def test_ros_global_multi_area_mapping(self):
        """Verify that mapping multiple areas preserves the overall map as in ROS SLAM."""
        # 1. Pose 1: Robot at (0, 0, 0 deg) sweeps Area 1
        self.page.set_robot_pose(0.0, 0.0, 0.0)
        for a in range(0, 181, 10):
            self.page._process_single_lidar_point(float(a), 40.0)
        self.page._complete_wall_polygon()
        area1_points = len(self.page.map_points)
        area1_walls = len(self.page.global_wall_segments)
        self.assertGreater(area1_points, 10)
        self.assertGreater(area1_walls, 5)
        self.assertEqual(len(self.page.explored_space_polygons), 1)

        # 2. Pose 2: Robot drives to (60, 0, 0 deg) and sweeps Area 2
        self.page.handle_udp_message("REACHED:LIDAR:60.0:0.0:0.0")
        for a in range(0, 181, 10):
            self.page._process_single_lidar_point(float(a), 40.0)
        self.page._complete_wall_polygon()

        # Both Area 1 and Area 2 must coexist in the global map!
        self.assertGreater(len(self.page.map_points), area1_points)
        self.assertGreater(len(self.page.global_wall_segments), area1_walls)
        self.assertEqual(len(self.page.explored_space_polygons), 2)
        self.assertEqual(len(self.page.robot_trail), 2)

        # 3. Pose 3: Robot turns 90 deg and moves to (60, 50)
        self.page.handle_udp_message("REACHED:LIDAR:60.0:50.0:90.0")
        for a in range(0, 181, 10):
            self.page._process_single_lidar_point(float(a), 40.0)
        self.page._complete_wall_polygon()

        # Three distinct explored areas accumulated
        self.assertEqual(len(self.page.explored_space_polygons), 3)
        self.assertEqual(len(self.page.robot_trail), 3)

        # 4. Global map drawing with all areas, walls, and free-space polygons
        self.page._draw_map()

        # 5. Verify telemetry accurately reports overall global stats
        self.assertIn("Global map:", self.page.telemetry_label.text())
        self.assertIn("Walls:", self.page.telemetry_label.text())


if __name__ == "__main__":
    unittest.main()