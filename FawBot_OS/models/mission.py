"""Mission data model with versioned JSON schema serialization."""
from dataclasses import dataclass, field
import datetime
from typing import List, Dict, Any, Optional

from models.pose import Pose
from models.obstacle import Boundary, RestrictedArea
from models.path import Path, Command, PathPoint
import config.settings as settings


@dataclass
class MissionMetadata:
    name: str = "new_mission"
    description: str = ""
    created_at: str = ""
    updated_at: str = ""

    def __post_init__(self):
        now_str = datetime.datetime.now().isoformat()
        if not self.created_at:
            self.created_at = now_str
        if not self.updated_at:
            self.updated_at = now_str


@dataclass
class EnvironmentModel:
    width_cm: float = settings.ENV_WIDTH_CM
    height_cm: float = settings.ENV_HEIGHT_CM
    boundary: Boundary = field(default_factory=Boundary)
    restricted_areas: List[RestrictedArea] = field(default_factory=list)


@dataclass
class RobotModel:
    length_cm: float = settings.ROBOT_LENGTH_CM
    width_cm: float = settings.ROBOT_WIDTH_CM
    safety_margin_cm: float = settings.ROBOT_SAFETY_MARGIN_CM
    wheel_base_cm: float = settings.WHEEL_BASE_CM


@dataclass
class MissionDestination:
    """A requested destination, optionally pinned to a robot ID."""
    destination_id: str
    x: float
    y: float
    robot_id: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.destination_id,
            "x": self.x,
            "y": self.y,
            "robot_id": self.robot_id,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MissionDestination":
        return cls(
            destination_id=data.get("id", "destination"),
            x=data.get("x", 0.0),
            y=data.get("y", 0.0),
            robot_id=data.get("robot_id"),
        )


@dataclass
class Mission:
    """Primary Mission entity encapsulating environment, robot parameters, trajectory, and commands."""
    version: int = 1
    metadata: MissionMetadata = field(default_factory=MissionMetadata)
    environment: EnvironmentModel = field(default_factory=EnvironmentModel)
    robot: RobotModel = field(default_factory=RobotModel)
    start_pose: Pose = field(default_factory=Pose)
    path: Path = field(default_factory=Path)
    commands: List[Command] = field(default_factory=list)
    destinations: List[MissionDestination] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        """Convert entire mission to versioned JSON dictionary."""
        return {
            "version": self.version,
            "mission": {
                "name": self.metadata.name,
                "description": self.metadata.description,
                "created_at": self.metadata.created_at,
                "updated_at": datetime.datetime.now().isoformat()
            },
            "environment": {
                "width_cm": self.environment.width_cm,
                "height_cm": self.environment.height_cm,
                "boundary": self.environment.boundary.polygon,
                "restricted_areas": [ra.to_dict() for ra in self.environment.restricted_areas]
            },
            "robot": {
                "length_cm": self.robot.length_cm,
                "width_cm": self.robot.width_cm,
                "safety_margin_cm": self.robot.safety_margin_cm,
                "wheel_base_cm": self.robot.wheel_base_cm
            },
            "start_pose": self.start_pose.to_dict(),
            "path": self.path.to_dict_list(),
            "commands": [c.to_dict() for c in self.commands],
            "destinations": [destination.to_dict() for destination in self.destinations]
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Mission":
        """Reconstruct Mission instance from JSON dictionary."""
        version = data.get("version", 1)

        # Metadata
        m_meta = data.get("mission", {})
        metadata = MissionMetadata(
            name=m_meta.get("name", "unnamed_mission"),
            description=m_meta.get("description", ""),
            created_at=m_meta.get("created_at", ""),
            updated_at=m_meta.get("updated_at", "")
        )

        # Environment
        env_dict = data.get("environment", {})
        width = env_dict.get("width_cm", settings.ENV_WIDTH_CM)
        height = env_dict.get("height_cm", settings.ENV_HEIGHT_CM)
        b_pts = env_dict.get("boundary", [])
        boundary = Boundary(width_cm=width, height_cm=height, polygon=b_pts)

        restricted = [
            RestrictedArea.from_dict(ra)
            for ra in env_dict.get("restricted_areas", [])
        ]
        environment = EnvironmentModel(
            width_cm=width,
            height_cm=height,
            boundary=boundary,
            restricted_areas=restricted
        )

        # Robot specs
        r_dict = data.get("robot", {})
        robot = RobotModel(
            length_cm=r_dict.get("length_cm", settings.ROBOT_LENGTH_CM),
            width_cm=r_dict.get("width_cm", settings.ROBOT_WIDTH_CM),
            safety_margin_cm=r_dict.get("safety_margin_cm", settings.ROBOT_SAFETY_MARGIN_CM),
            wheel_base_cm=r_dict.get("wheel_base_cm", settings.WHEEL_BASE_CM)
        )

        # Start pose
        start_pose = Pose.from_dict(data.get("start_pose", {}))

        # Path
        path = Path.from_dict_list(data.get("path", []))

        # Commands
        commands = [
            Command.from_dict(c)
            for c in data.get("commands", [])
        ]
        destinations = [
            MissionDestination.from_dict(destination)
            for destination in data.get("destinations", [])
        ]

        return cls(
            version=version,
            metadata=metadata,
            environment=environment,
            robot=robot,
            start_pose=start_pose,
            path=path,
            commands=commands,
            destinations=destinations
        )
