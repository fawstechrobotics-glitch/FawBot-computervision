"""Unit tests for MissionStorage file persistence and JSON schema validation."""
import os
import json
import tempfile
import unittest

from storage.mission_storage import MissionStorage, MissionStorageError
from models.mission import Mission, MissionMetadata
from models.path import Path, PathPoint, Command
from models.pose import Pose


class TestMissionStorage(unittest.TestCase):

    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.storage = MissionStorage(directory=self.temp_dir)

    def tearDown(self):
        for f in os.listdir(self.temp_dir):
            os.remove(os.path.join(self.temp_dir, f))
        os.rmdir(self.temp_dir)

    def test_save_and_load_valid_mission(self):
        mission = Mission(
            metadata=MissionMetadata(name="test_mission", description="A test mission"),
            start_pose=Pose(15.0, 20.0, 90.0)
        )
        mission.path.append(PathPoint(15.0, 20.0, 90.0, action="START"))
        mission.path.append(PathPoint(35.0, 20.0, 0.0, action="MOVE", distance=20.0))
        mission.commands.append(Command("MOVE", 20.0, 1.0))

        filepath = self.storage.save_mission(mission, "test_mission.json")
        self.assertTrue(os.path.exists(filepath))

        loaded = self.storage.load_mission("test_mission.json")
        self.assertEqual(loaded.metadata.name, "test_mission")
        self.assertEqual(loaded.start_pose.x, 15.0)
        self.assertEqual(len(loaded.path.points), 2)
        self.assertEqual(len(loaded.commands), 1)

    def test_load_nonexistent_file(self):
        with self.assertRaises(MissionStorageError):
            self.storage.load_mission("does_not_exist.json")

    def test_load_corrupted_json(self):
        corrupt_path = os.path.join(self.temp_dir, "corrupt.json")
        with open(corrupt_path, "w") as f:
            f.write("{invalid_json: true,")

        with self.assertRaises(MissionStorageError):
            self.storage.load_mission("corrupt.json")

    def test_load_missing_version(self):
        invalid_path = os.path.join(self.temp_dir, "no_version.json")
        with open(invalid_path, "w") as f:
            json.dump({"mission": {}}, f)

        with self.assertRaises(MissionStorageError):
            self.storage.load_mission("no_version.json")

    def test_list_and_delete_missions(self):
        m1 = Mission(metadata=MissionMetadata(name="Mission1"))
        m2 = Mission(metadata=MissionMetadata(name="Mission2"))
        self.storage.save_mission(m1, "m1.json")
        self.storage.save_mission(m2, "m2.json")

        listing = self.storage.list_missions()
        self.assertEqual(len(listing), 2)

        deleted = self.storage.delete_mission("m1.json")
        self.assertTrue(deleted)
        self.assertEqual(len(self.storage.list_missions()), 1)


if __name__ == "__main__":
    unittest.main()
