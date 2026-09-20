"""Navigation package containing geometry, path recording, planning, execution, and mission management."""
from navigation.geometry import (
    compute_robot_footprint,
    point_in_polygon,
    segments_intersect,
    polygon_intersects_polygon,
    is_footprint_within_boundary,
    validate_trajectory
)
from navigation.path_recorder import PathRecorder
from navigation.path_planner import PathPlanner, RecordedPathPlanner
from navigation.navigation_executor import NavigationExecutor
from navigation.mission_manager import MissionManager

__all__ = [
    "compute_robot_footprint",
    "point_in_polygon",
    "segments_intersect",
    "polygon_intersects_polygon",
    "is_footprint_within_boundary",
    "validate_trajectory",
    "PathRecorder",
    "PathPlanner",
    "RecordedPathPlanner",
    "NavigationExecutor",
    "MissionManager",
]
