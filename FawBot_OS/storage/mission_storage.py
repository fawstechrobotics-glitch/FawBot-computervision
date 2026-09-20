"""Mission file persistence and schema validation."""
import os
import json
import logging
from typing import List, Dict, Any, Optional

import config.settings as settings
from models.mission import Mission

logger = logging.getLogger(__name__)


class MissionStorageError(Exception):
    """Custom exception raised for mission storage failures."""
    pass


class MissionStorage:
    """Manages saving, loading, listing, and validating mission JSON files."""

    def __init__(self, directory: str = settings.MISSIONS_DIR):
        self.directory = directory
        os.makedirs(self.directory, exist_ok=True)

    def _resolve_filepath(self, filename: str) -> str:
        """Resolve filename to an absolute path within the missions directory."""
        if not filename.endswith(".json"):
            filename += ".json"
        if os.path.isabs(filename):
            return filename
        return os.path.join(self.directory, filename)

    def save_mission(self, mission: Mission, filename: Optional[str] = None) -> str:
        """
        Serialize and write a Mission to a JSON file.
        Returns the saved file path.
        """
        if not filename:
            safe_name = mission.metadata.name.strip().replace(" ", "_").lower()
            filename = f"{safe_name}.json"

        filepath = self._resolve_filepath(filename)

        try:
            data = mission.to_dict()
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=4)
            logger.info(f"Saved mission '{mission.metadata.name}' to {filepath}")
            return filepath
        except Exception as e:
            msg = f"Failed to save mission to '{filepath}': {e}"
            logger.error(msg)
            raise MissionStorageError(msg) from e

    def load_mission(self, filename: str) -> Mission:
        """
        Read and deserialize a Mission from a JSON file.
        Raises MissionStorageError if file is missing, invalid JSON, or corrupt.
        """
        filepath = self._resolve_filepath(filename)

        if not os.path.exists(filepath):
            raise MissionStorageError(f"Mission file not found: {filepath}")

        try:
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)
        except json.JSONDecodeError as e:
            raise MissionStorageError(f"Corrupted or invalid JSON in '{filename}': {e}") from e
        except Exception as e:
            raise MissionStorageError(f"Error reading mission file '{filename}': {e}") from e

        # Validate Schema
        if not isinstance(data, dict):
            raise MissionStorageError(f"Invalid mission format in '{filename}': root must be a JSON object.")

        version = data.get("version")
        if version is None:
            raise MissionStorageError(f"Invalid mission schema in '{filename}': missing 'version' field.")

        if version > 1:
            raise MissionStorageError(
                f"Incompatible mission schema version {version} in '{filename}'. Supported version is 1."
            )

        try:
            mission = Mission.from_dict(data)
            logger.info(f"Successfully loaded mission '{mission.metadata.name}' from {filepath}")
            return mission
        except Exception as e:
            raise MissionStorageError(f"Failed to parse mission structure in '{filename}': {e}") from e

    def list_missions(self) -> List[Dict[str, Any]]:
        """List all available mission files with basic summary metadata."""
        missions = []
        if not os.path.exists(self.directory):
            return missions

        for fname in sorted(os.listdir(self.directory)):
            if fname.endswith(".json"):
                fpath = os.path.join(self.directory, fname)
                try:
                    with open(fpath, "r", encoding="utf-8") as f:
                        data = json.load(f)
                        meta = data.get("mission", {})
                        missions.append({
                            "filename": fname,
                            "filepath": fpath,
                            "name": meta.get("name", fname),
                            "description": meta.get("description", ""),
                            "created_at": meta.get("created_at", ""),
                            "updated_at": meta.get("updated_at", ""),
                            "points_count": len(data.get("path", []))
                        })
                except Exception as e:
                    logger.warning(f"Could not read metadata from '{fname}': {e}")
                    missions.append({
                        "filename": fname,
                        "filepath": fpath,
                        "name": f"{fname} (unreadable)",
                        "error": str(e)
                    })

        return missions

    def delete_mission(self, filename: str) -> bool:
        """Delete a mission file from the directory."""
        filepath = self._resolve_filepath(filename)
        if not os.path.exists(filepath):
            return False

        try:
            os.remove(filepath)
            logger.info(f"Deleted mission file: {filepath}")
            return True
        except Exception as e:
            msg = f"Failed to delete mission file '{filepath}': {e}"
            logger.error(msg)
            raise MissionStorageError(msg) from e
