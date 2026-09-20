"""Tests for roster parsing and collision-aware destination allocation."""
import os
import tempfile
import unittest

from robot.fleet import RobotRoster, RobotSpec, assign_destinations


class TestRobotFleet(unittest.TestCase):

    def test_roster_parses_robot_identity_and_pose(self):
        with tempfile.NamedTemporaryFile(mode="w", delete=False) as roster_file:
            roster_file.write("# id,host,port,start_x,start_y,heading\n")
            roster_file.write("Fawbot_01,192.168.1.31,8888,10,20,90\n")
            filename = roster_file.name
        try:
            roster = RobotRoster.from_file(filename)
            self.assertEqual(roster.get("Fawbot_01").host, "192.168.1.31")
            self.assertEqual(roster.get("Fawbot_01").start_position, (10.0, 20.0))
        finally:
            os.unlink(filename)

    def test_explicit_and_nearest_assignments_are_one_to_one(self):
        roster = RobotRoster([
            RobotSpec("A", "a.local", start_x=0, start_y=0),
            RobotSpec("B", "b.local", start_x=100, start_y=0),
        ])
        assignments = assign_destinations([
            ("loading", 95, 0, "B"),
            ("parking", 5, 0, None),
        ], roster)
        self.assertEqual({assignment.robot_id for assignment in assignments}, {"A", "B"})
        self.assertEqual(next(item for item in assignments if item.destination_id == "loading").explicit, True)

    def test_close_destinations_are_rejected(self):
        roster = RobotRoster([RobotSpec("A", "a.local"), RobotSpec("B", "b.local")])
        with self.assertRaisesRegex(ValueError, "too close"):
            assign_destinations([("one", 20, 20, "A"), ("two", 21, 20, "B")], roster)

    def test_one_robot_can_have_multiple_explicit_waypoints(self):
        roster = RobotRoster([
            RobotSpec("A", "a.local", start_x=0, start_y=0),
            RobotSpec("B", "b.local", start_x=100, start_y=0),
        ])
        assignments = assign_destinations([
            ("a_one", 20, 20, "A"),
            ("a_two", 40, 20, "A"),
            ("b_one", 100, 40, "B"),
        ], roster)
        self.assertEqual([item.destination_id for item in assignments if item.robot_id == "A"], ["a_one", "a_two"])

    def test_close_waypoints_for_same_robot_are_allowed(self):
        roster = RobotRoster([RobotSpec("A", "a.local"), RobotSpec("B", "b.local")])
        assignments = assign_destinations([
            ("one", 20, 20, "A"),
            ("two", 21, 20, "A"),
        ], roster)
        self.assertEqual(len(assignments), 2)


if __name__ == "__main__":
    unittest.main()