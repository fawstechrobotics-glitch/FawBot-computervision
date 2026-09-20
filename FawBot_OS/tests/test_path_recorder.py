"""Unit tests for PathRecorder intelligent throttling and teach mode."""
import time
import unittest

from navigation.path_recorder import PathRecorder
from models.pose import Pose


class TestPathRecorder(unittest.TestCase):

    def test_start_and_stop_recording(self):
        recorder = PathRecorder(min_distance_cm=1.0, min_angle_deg=2.0, min_interval_sec=0.1)
        self.assertFalse(recorder.is_recording)

        start_pose = Pose(10.0, 10.0, 0.0)
        recorder.start_recording(start_pose)
        self.assertTrue(recorder.is_recording)
        self.assertEqual(len(recorder.path.points), 1)
        self.assertEqual(recorder.path.points[0].action, "START")

        path = recorder.stop_recording()
        self.assertFalse(recorder.is_recording)
        self.assertEqual(len(path.points), 1)

    def test_duplicate_suppression_throttling(self):
        recorder = PathRecorder(min_distance_cm=2.0, min_angle_deg=5.0, min_interval_sec=1.0)
        recorder.start_recording(Pose(0.0, 0.0, 0.0))

        # Tiny movement (0.2 cm) -> should be suppressed
        recorded = recorder.record_manual_event(0.2, 0.0, 0.0, action="MOVE")
        self.assertFalse(recorded)
        self.assertEqual(len(recorder.path.points), 1)

        # Significant movement (5.0 cm) -> should be recorded
        recorded = recorder.record_manual_event(5.0, 0.0, 0.0, action="MOVE")
        self.assertTrue(recorded)
        self.assertEqual(len(recorder.path.points), 2)

        # Tiny turn (1.0 deg) -> should be suppressed
        recorded = recorder.record_manual_event(5.0, 0.0, 1.0, action="TURN")
        self.assertFalse(recorded)

        # Significant turn (20.0 deg) -> should be recorded
        recorded = recorder.record_manual_event(5.0, 0.0, 20.0, action="TURN")
        self.assertTrue(recorded)
        self.assertEqual(len(recorder.path.points), 3)

    def test_convert_to_commands(self):
        recorder = PathRecorder(min_distance_cm=1.0, min_angle_deg=2.0, min_interval_sec=0.1)
        recorder.start_recording(Pose(0.0, 0.0, 0.0))
        recorder.record_manual_event(10.0, 0.0, 0.0, action="MOVE")
        recorder.record_manual_event(10.0, 10.0, 90.0, action="MOVE")

        commands = recorder.path.to_commands()
        self.assertTrue(len(commands) >= 2)
        types = [c.type for c in commands]
        self.assertIn("MOVE", types)
        self.assertIn("TURN", types)


if __name__ == "__main__":
    unittest.main()
