"""Path Planner abstraction and implementations (Teach Mode & Future Autonomous Planners)."""
from abc import ABC, abstractmethod
from typing import List, Optional
import math
import logging

from models.pose import Pose
from models.path import Path, PathPoint, Command
from models.mission import EnvironmentModel

logger = logging.getLogger(__name__)


class PathPlanner(ABC):
    """Abstract base class for trajectory generators and path planning algorithms."""

    @abstractmethod
    def plan(self, start: Pose, goal: Pose, environment: EnvironmentModel) -> Path:
        """
        Compute a collision-free path from start to goal within given environment.
        Raises NotImplementedError if not implemented.
        """
        raise NotImplementedError("Path planning algorithm must be implemented by subclass.")


class RecordedPathPlanner(PathPlanner):
    """Planner that generates path commands from user-taught or waypoints trajectory."""

    def plan(self, start: Pose, goal: Pose, environment: EnvironmentModel) -> Path:
        """Simple direct trajectory planner connecting start to goal."""
        path = Path()
        path.append(PathPoint(
            x=start.x,
            y=start.y,
            heading=start.heading,
            action="START"
        ))

        dx = goal.x - start.x
        dy = goal.y - start.y
        target_heading = math.degrees(math.atan2(dy, dx))

        path.append(PathPoint(
            x=goal.x,
            y=goal.y,
            heading=target_heading,
            action="GOAL",
            distance=math.hypot(dx, dy)
        ))
        return path


# --- FUTURE AUTONOMOUS PATH PLANNERS (EXTENSIBILITY HOOKS) ---

class AStarPlanner(PathPlanner):
    """Grid-based A* path planner (Reserved for future autonomous navigation)."""
    def plan(self, start: Pose, goal: Pose, environment: EnvironmentModel) -> Path:
        raise NotImplementedError("A* Planner will be configured in future release.")


class RRTStarPlanner(PathPlanner):
    """Rapidly-exploring Random Tree Star (RRT*) planner (Reserved for future autonomous navigation)."""
    def plan(self, start: Pose, goal: Pose, environment: EnvironmentModel) -> Path:
        raise NotImplementedError("RRT* Planner will be configured in future release.")
