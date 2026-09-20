"""Path, PathPoint, and Robot Command data structures."""
from dataclasses import dataclass, field
import math
import time
from typing import List, Tuple, Dict, Any


@dataclass
class PathPoint:
    """A single spatial and temporal node in a trajectory."""
    x: float
    y: float
    heading: float = 0.0
    timestamp: float = 0.0
    action: str = "MOVE"  # "START", "MOVE", "TURN", "WAYPOINT"
    distance: float = 0.0
    rotation: float = 0.0

    def __post_init__(self):
        self.x = float(self.x)
        self.y = float(self.y)
        self.heading = float(self.heading)
        if self.timestamp == 0.0:
            self.timestamp = time.time()

    @property
    def as_tuple(self) -> Tuple[float, float]:
        return (self.x, self.y)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": round(self.timestamp, 3),
            "x": round(self.x, 2),
            "y": round(self.y, 2),
            "heading": round(self.heading, 2),
            "action": self.action,
            "distance": round(self.distance, 2),
            "rotation": round(self.rotation, 2)
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "PathPoint":
        return cls(
            timestamp=data.get("timestamp", 0.0),
            x=data.get("x", 0.0),
            y=data.get("y", 0.0),
            heading=data.get("heading", 0.0),
            action=data.get("action", "MOVE"),
            distance=data.get("distance", 0.0),
            rotation=data.get("rotation", 0.0)
        )


@dataclass
class Command:
    """Discrete executable hardware instruction for the robot."""
    type: str          # "MOVE" or "TURN"
    value: float       # distance in cm or turn in degrees
    direction: float = 1.0  # 1.0 = forward, -1.0 = backward (for MOVE)

    def to_udp_string(self) -> str:
        """Converts to exact existing hardware UDP protocol string."""
        if self.type == "MOVE":
            # Protocol: MOVE:<distance>:<direction>
            return f"MOVE:{abs(self.value):.1f}:{self.direction:.1f}"
        elif self.type == "TURN":
            # Protocol: TURN:<angle>
            return f"TURN:{self.value:.1f}"
        else:
            raise ValueError(f"Unknown command type: {self.type}")

    def to_dict(self) -> Dict[str, Any]:
        if self.type == "MOVE":
            return {
                "type": "MOVE",
                "distance_cm": round(self.value, 2),
                "direction": self.direction
            }
        else:
            return {
                "type": "TURN",
                "angle_deg": round(self.value, 2)
            }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Command":
        cmd_type = data.get("type", "MOVE")
        if cmd_type == "MOVE":
            return cls(
                type="MOVE",
                value=data.get("distance_cm", 0.0),
                direction=data.get("direction", 1.0)
            )
        elif cmd_type == "TURN":
            return cls(
                type="TURN",
                value=data.get("angle_deg", 0.0)
            )
        else:
            raise ValueError(f"Unsupported command dict format: {data}")


@dataclass
class Path:
    """Collection of path points representing a continuous trajectory."""
    points: List[PathPoint] = field(default_factory=list)

    def append(self, point: PathPoint):
        self.points.append(point)

    def clear(self):
        self.points.clear()

    def __len__(self) -> int:
        return len(self.points)

    def __iter__(self):
        return iter(self.points)

    def __getitem__(self, index):
        return self.points[index]

    @property
    def total_distance(self) -> float:
        """Compute cumulative Euclidean length of path in cm."""
        if len(self.points) < 2:
            return 0.0
        dist = 0.0
        for i in range(len(self.points) - 1):
            p1 = self.points[i]
            p2 = self.points[i + 1]
            dist += math.hypot(p2.x - p1.x, p2.y - p1.y)
        return dist

    def get_coordinates(self) -> List[Tuple[float, float]]:
        return [p.as_tuple for p in self.points]

    def to_commands(self) -> List[Command]:
        """Convert consecutive path nodes into discrete TURN and MOVE commands."""
        commands = []
        if len(self.points) < 2:
            return commands

        current_heading = self.points[0].heading
        for i in range(len(self.points) - 1):
            p1 = self.points[i]
            p2 = self.points[i + 1]

            dx = p2.x - p1.x
            dy = p2.y - p1.y
            dist = math.hypot(dx, dy)

            if dist < 0.1:
                continue

            target_heading = math.degrees(math.atan2(dy, dx))
            turn = target_heading - current_heading
            while turn > 180.0: turn -= 360.0
            while turn < -180.0: turn += 360.0

            if abs(turn) > 0.5:
                commands.append(Command(type="TURN", value=turn))
                current_heading = target_heading

            commands.append(Command(type="MOVE", value=dist, direction=1.0))

        return commands

    def to_dict_list(self) -> List[Dict[str, Any]]:
        return [p.to_dict() for p in self.points]

    @classmethod
    def from_dict_list(cls, data: List[Dict[str, Any]]) -> "Path":
        pts = [PathPoint.from_dict(d) for d in data]
        return cls(points=pts)
