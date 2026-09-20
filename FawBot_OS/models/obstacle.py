"""Boundary and Restricted (No-Go) Area data models."""
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Any


@dataclass
class Boundary:
    """Environment boundary defined as an optional closed polygon.

    Set ``enabled=True`` only when you want a hard wall to limit the robot
    to a specific region. By default the boundary is *disabled* so the map
    is unbounded and validation does not check containment.
    """
    width_cm: float = 150.0
    height_cm: float = 150.0
    polygon: List[Tuple[float, float]] = field(default_factory=list)
    enabled: bool = False  # Disabled by default — no restriction

    def __post_init__(self):
        # Only auto-generate the rectangle when explicitly enabled and no polygon given
        if self.enabled and not self.polygon:
            self.polygon = [
                (0.0, 0.0),
                (self.width_cm, 0.0),
                (self.width_cm, self.height_cm),
                (0.0, self.height_cm)
            ]
        else:
            self.polygon = [(float(p[0]), float(p[1])) for p in self.polygon]

    @property
    def bounding_box(self) -> Tuple[float, float, float, float]:
        """Returns (min_x, min_y, max_x, max_y). Falls back to large canvas if disabled."""
        if not self.enabled or not self.polygon:
            return (-10000.0, -10000.0, 10000.0, 10000.0)
        xs = [p[0] for p in self.polygon]
        ys = [p[1] for p in self.polygon]
        return (min(xs), min(ys), max(xs), max(ys))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "width_cm": self.width_cm,
            "height_cm": self.height_cm,
            "enabled": self.enabled,
            "boundary": [[round(p[0], 2), round(p[1], 2)] for p in self.polygon]
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Boundary":
        width = float(data.get("width_cm", 150.0))
        height = float(data.get("height_cm", 150.0))
        enabled = bool(data.get("enabled", False))
        pts = data.get("boundary", [])
        polygon = [(float(p[0]), float(p[1])) for p in pts] if pts else []
        return cls(width_cm=width, height_cm=height, polygon=polygon, enabled=enabled)


@dataclass
class RestrictedArea:
    """Restricted / No-Go polygon area where robot navigation is forbidden."""
    id: str
    name: str = ""
    polygon: List[Tuple[float, float]] = field(default_factory=list)
    color: str = "#ff3366"
    active: bool = True

    def __post_init__(self):
        if not self.name:
            self.name = self.id
        self.polygon = [(float(p[0]), float(p[1])) for p in self.polygon]

    @property
    def is_valid_polygon(self) -> bool:
        """A valid 2D polygon must have at least 3 distinct vertices."""
        return len(self.polygon) >= 3

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "polygon": [[round(p[0], 2), round(p[1], 2)] for p in self.polygon],
            "color": self.color,
            "active": self.active
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "RestrictedArea":
        raw_pts = data.get("polygon", [])
        pts = [(float(p[0]), float(p[1])) for p in raw_pts]
        return cls(
            id=data.get("id", "restricted_zone"),
            name=data.get("name", ""),
            polygon=pts,
            color=data.get("color", "#ff3366"),
            active=data.get("active", True)
        )
