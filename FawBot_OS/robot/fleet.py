"""Robot roster loading and multi-robot destination assignment."""
from dataclasses import dataclass
import math
import os
from typing import Dict, Iterable, List, Optional, Tuple

import config.settings as settings


@dataclass(frozen=True)
class RobotSpec:
    """Network identity and current/start pose for one physical robot."""
    robot_id: str
    host: str
    port: int = settings.UDP_PORT
    start_x: float = 20.0
    start_y: float = 20.0
    heading: float = 0.0

    @property
    def start_position(self) -> Tuple[float, float]:
        return self.start_x, self.start_y


@dataclass(frozen=True)
class DestinationAssignment:
    """A destination assigned to a robot for one mission run."""
    destination_id: str
    x: float
    y: float
    robot_id: str
    explicit: bool = False


class RobotRoster:
    """Loads robot identities from embedded_code/assigned_robot.txt.

    Each non-empty, non-comment line uses either ``robot_id`` or:
    ``robot_id,host[,port[,start_x,start_y[,heading]]]``.
    A robot-only line uses ``<robot_id>.local`` as its mDNS host.
    """

    def __init__(self, robots: Iterable[RobotSpec]):
        self.robots = list(robots)
        self._by_id: Dict[str, RobotSpec] = {robot.robot_id: robot for robot in self.robots}

    def get(self, robot_id: str) -> Optional[RobotSpec]:
        return self._by_id.get(robot_id)

    @classmethod
    def from_file(cls, filename: Optional[str] = None) -> "RobotRoster":
        filename = filename or os.path.join(settings.BASE_DIR, "embedded_code", "assigned_robot.txt")
        robots: List[RobotSpec] = []
        if os.path.exists(filename):
            with open(filename, "r", encoding="utf-8") as roster_file:
                for line_number, raw_line in enumerate(roster_file, 1):
                    line = raw_line.split("#", 1)[0].strip()
                    if not line:
                        continue
                    fields = [field.strip() for field in line.split(",")]
                    robot_id = fields[0]
                    if not robot_id:
                        raise ValueError(f"Invalid robot roster line {line_number}: missing robot id")
                    try:
                        robots.append(RobotSpec(
                            robot_id=robot_id,
                            host=fields[1] if len(fields) > 1 and fields[1] else f"{robot_id}.local",
                            port=int(fields[2]) if len(fields) > 2 and fields[2] else settings.UDP_PORT,
                            start_x=float(fields[3]) if len(fields) > 3 and fields[3] else 20.0,
                            start_y=float(fields[4]) if len(fields) > 4 and fields[4] else 20.0,
                            heading=float(fields[5]) if len(fields) > 5 and fields[5] else 0.0,
                        ))
                    except (TypeError, ValueError) as exc:
                        raise ValueError(f"Invalid robot roster line {line_number}: {raw_line.strip()}") from exc
        return cls(robots)


def assign_destinations(
    destinations: Iterable[Tuple[str, float, float, Optional[str]]],
    roster: RobotRoster,
    minimum_separation_cm: float = settings.ROBOT_LENGTH_CM + settings.ROBOT_SAFETY_MARGIN_CM * 2,
) -> List[DestinationAssignment]:
    """Assign destinations explicitly or by nearest robot.

    Explicit assignments may contain multiple waypoints for one robot. Automatic
    assignments use the nearest robot independently for each waypoint. Destinations
    that are closer than one robot's effective length are rejected to avoid
    colocating robots at the same goal.
    """
    requested = list(destinations)
    if not requested:
        return []
    if not roster.robots:
        raise ValueError("No robots are configured in embedded_code/assigned_robot.txt")

    assignments: List[DestinationAssignment] = []
    for destination_id, x, y, requested_robot_id in requested:
        if requested_robot_id:
            robot = roster.get(requested_robot_id)
            if robot is None:
                raise ValueError(f"Unknown robot '{requested_robot_id}' for destination '{destination_id}'")
            assignments.append(DestinationAssignment(destination_id, float(x), float(y), robot.robot_id, True))

    automatic = [item for item in requested if not item[3]]
    for destination_id, x, y, _ in automatic:
        robot = min(roster.robots, key=lambda candidate: math.dist((x, y), candidate.start_position))
        assignments.append(DestinationAssignment(destination_id, float(x), float(y), robot.robot_id, False))

    for index, left in enumerate(assignments):
        for right in assignments[index + 1:]:
            if left.robot_id == right.robot_id:
                continue
            if math.dist((left.x, left.y), (right.x, right.y)) < minimum_separation_cm:
                raise ValueError(
                    f"Destinations '{left.destination_id}' and '{right.destination_id}' are too close "
                    f"for safe multi-robot operation ({minimum_separation_cm:.1f} cm minimum)"
                )
    return assignments