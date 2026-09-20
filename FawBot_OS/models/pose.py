"""2D Robot Pose representation."""
from dataclasses import dataclass
import math
import time
from typing import Tuple, Dict, Any


def normalize_angle_deg(angle: float) -> float:
    """Normalize angle in degrees to range [-180.0, 180.0]."""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


@dataclass
class Pose:
    """Represents a 2D pose with position (x, y) in cm, heading in degrees, and timestamp."""
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    timestamp: float = 0.0

    def __post_init__(self):
        self.heading = normalize_angle_deg(float(self.heading))
        self.x = float(self.x)
        self.y = float(self.y)
        if self.timestamp == 0.0:
            self.timestamp = time.time()

    @property
    def as_tuple(self) -> Tuple[float, float]:
        """Return (x, y) coordinates."""
        return (self.x, self.y)

    @property
    def heading_rad(self) -> float:
        """Return heading in radians."""
        return math.radians(self.heading)

    def distance_to(self, other: "Pose") -> float:
        """Compute Euclidean distance to another pose in cm."""
        return math.hypot(other.x - self.x, other.y - self.y)

    def angle_to(self, other: "Pose") -> float:
        """Calculate absolute heading angle (degrees) pointing toward other pose."""
        dx = other.x - self.x
        dy = other.y - self.y
        return normalize_angle_deg(math.degrees(math.atan2(dy, dx)))

    def turn_angle_to(self, target_heading: float) -> float:
        """Calculate smallest relative turn angle (degrees) to match target_heading."""
        return normalize_angle_deg(target_heading - self.heading)

    def translated(self, distance_cm: float) -> "Pose":
        """Return new Pose moved forward by distance_cm along current heading."""
        rad = self.heading_rad
        return Pose(
            x=self.x + distance_cm * math.cos(rad),
            y=self.y + distance_cm * math.sin(rad),
            heading=self.heading,
            timestamp=time.time()
        )

    def rotated(self, delta_deg: float) -> "Pose":
        """Return new Pose rotated by delta_deg."""
        return Pose(
            x=self.x,
            y=self.y,
            heading=normalize_angle_deg(self.heading + delta_deg),
            timestamp=time.time()
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "x": round(self.x, 2),
            "y": round(self.y, 2),
            "heading": round(self.heading, 2),
            "timestamp": round(self.timestamp, 3)
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Pose":
        return cls(
            x=data.get("x", 0.0),
            y=data.get("y", 0.0),
            heading=data.get("heading", 0.0),
            timestamp=data.get("timestamp", 0.0)
        )
