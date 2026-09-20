"""Mission Manager orchestrating missions, recording, validation, and execution."""
import logging
from typing import Tuple, List, Optional
from PyQt5.QtCore import QObject, pyqtSignal

from models.mission import Mission, EnvironmentModel, RobotModel, MissionMetadata, MissionDestination
from models.pose import Pose
from models.path import Path, Command
from models.obstacle import Boundary, RestrictedArea
from storage.mission_storage import MissionStorage, MissionStorageError
from navigation.path_recorder import PathRecorder
from navigation.navigation_executor import NavigationExecutor
from navigation.geometry import validate_trajectory
from robot.robot_controller import RobotController
from robot.robot_state import RobotState
from robot.fleet import RobotRoster, assign_destinations, DestinationAssignment
from robot.fleet_controller import RobotFleet
from models.pose import normalize_angle_deg
import math

logger = logging.getLogger(__name__)


class MissionManager(QObject):
    """
    Coordinates mission lifecycle: creation, file persistence, teach-mode recording,
    footprint collision validation, and autonomous navigation execution.
    """
    mission_loaded = pyqtSignal(object)                     # Emits active Mission
    validation_completed = pyqtSignal(bool, list)           # is_valid, list of error strings
    teach_mode_changed = pyqtSignal(bool)
    status_message = pyqtSignal(str)

    def __init__(self, controller: RobotController, state: RobotState, roster: Optional[RobotRoster] = None):
        super().__init__()
        self.controller = controller
        self.state = state
        self.roster = roster or RobotRoster([])

        self.storage = MissionStorage()
        self.recorder = PathRecorder()
        self.executor = NavigationExecutor(self.controller, self.state)
        self.fleet: Optional[RobotFleet] = None
        self.auto_return_home = True
        self._fleet_assignments: List[DestinationAssignment] = []
        self._fleet_routes = {}
        self._fleet_executors = {}
        self._fleet_completed = set()

        self._active_mission = Mission()

        # Connect recorder signals
        self.recorder.recording_state_changed.connect(self._on_recording_state_changed)
        self.recorder.recording_state_changed.connect(self.state.set_recording)

    @property
    def active_mission(self) -> Mission:
        return self._active_mission

    @property
    def is_recording(self) -> bool:
        return self.recorder.is_recording

    def new_mission(self, name: str = "new_mission") -> Mission:
        """Create a fresh empty mission with default environment and robot settings."""
        self._active_mission = Mission(
            metadata=MissionMetadata(name=name),
            environment=EnvironmentModel(),
            robot=RobotModel(),
            start_pose=self.state.pose if self.state.pose else Pose(10.0, 10.0, 0.0),
            path=Path(),
            commands=[]
        )
        self.recorder.clear()
        logger.info(f"Created new mission: '{name}'")
        self.mission_loaded.emit(self._active_mission)
        self.status_message.emit(f"New mission created: '{name}'")
        return self._active_mission

    def load_mission(self, filename: str) -> Mission:
        """Load a mission from storage and make it active."""
        mission = self.storage.load_mission(filename)
        self._active_mission = mission
        self.recorder.clear()
        self.executor.set_commands(mission.commands)

        logger.info(f"Loaded mission '{mission.metadata.name}' with {len(mission.path)} points.")
        self.mission_loaded.emit(self._active_mission)
        self.status_message.emit(f"Loaded mission '{mission.metadata.name}' ({len(mission.path)} points).")
        return mission

    def save_mission(self, filename: Optional[str] = None) -> str:
        """Save the active mission to storage."""
        # Ensure commands are synced with path
        if self._active_mission.path.points and not self._active_mission.commands:
            self._active_mission.commands = self._active_mission.path.to_commands()

        filepath = self.storage.save_mission(self._active_mission, filename)
        self.status_message.emit(f"Saved mission to '{filepath}'")
        return filepath

    def start_teach_mode(self):
        """Begin manual driving path recording."""
        start_pose = self.state.pose if self.state.pose else Pose(10.0, 10.0, 0.0)
        self._active_mission.start_pose = start_pose
        self._active_mission.path.clear()
        self._active_mission.commands.clear()

        self.recorder.start_recording(start_pose)
        self.status_message.emit("Teach Mode ACTIVE | Manual driving path is being recorded...")

    def stop_teach_mode(self) -> Path:
        """Stop path recording and synthesize discrete commands."""
        recorded_path = self.recorder.stop_recording()
        self._active_mission.path = recorded_path
        self._active_mission.commands = recorded_path.to_commands()
        self.executor.set_commands(self._active_mission.commands)

        msg = f"Teach Mode STOPPED | Captured {len(recorded_path)} points ({len(self._active_mission.commands)} commands)."
        self.status_message.emit(msg)
        logger.info(msg)
        self.mission_loaded.emit(self._active_mission)
        return recorded_path

    def record_drive_step(
        self,
        x: float,
        y: float,
        heading: float,
        action: str = "MOVE",
        distance: float = 0.0,
        rotation: float = 0.0
    ):
        """Record a manual drive step if teach mode is active."""
        if self.recorder.is_recording:
            self.recorder.record_manual_event(x, y, heading, action, distance, rotation)

    @property
    def is_multi_robot(self) -> bool:
        return len(self.roster.robots) > 1

    def add_destination(self, x: float, y: float, robot_id: Optional[str] = None) -> MissionDestination:
        """Add a destination selected in the planner for explicit/automatic allocation."""
        destination = MissionDestination(
            destination_id=f"destination_{len(self._active_mission.destinations) + 1}",
            x=x,
            y=y,
            robot_id=robot_id,
        )
        self._active_mission.destinations.append(destination)
        return destination

    def plan_destination_assignments(self) -> List[DestinationAssignment]:
        """Resolve explicit IDs and nearest-robot fallback for mission destinations."""
        return assign_destinations([
            (destination.destination_id, destination.x, destination.y, destination.robot_id)
            for destination in self._active_mission.destinations
        ], self.roster)

    def validate_mission(self, mission: Optional[Mission] = None) -> Tuple[bool, List[str]]:
        """
        Validate path against environment boundary and restricted no-go zones
        accounting for the robot footprint and safety margin.
        """
        target_mission = mission if mission is not None else self._active_mission

        if target_mission.destinations and self.is_multi_robot:
            try:
                assignments = self.plan_destination_assignments()
            except ValueError as exc:
                res = (False, [str(exc)])
                self.validation_completed.emit(res[0], res[1])
                return res
            res = (True, [f"{len(assignments)} destinations assigned without overlap."])
            self.validation_completed.emit(res[0], res[1])
            return res

        path_coords = []
        if target_mission.path.points:
            path_coords = [(p.x, p.y, p.heading) for p in target_mission.path.points]
        elif target_mission.start_pose:
            path_coords = [(target_mission.start_pose.x, target_mission.start_pose.y, target_mission.start_pose.heading)]

        if not path_coords:
            res = (False, ["Mission path is empty. Record or generate a path first."])
            self.validation_completed.emit(res[0], res[1])
            return res

        is_valid, errors = validate_trajectory(
            path_points=path_coords,
            robot_length=target_mission.robot.length_cm,
            robot_width=target_mission.robot.width_cm,
            safety_margin=target_mission.robot.safety_margin_cm,
            boundary=target_mission.environment.boundary,
            restricted_areas=target_mission.environment.restricted_areas
        )

        self.validation_completed.emit(is_valid, errors)
        if is_valid:
            self.status_message.emit("Mission Validation PASSED: Path is collision-free!")
            logger.info("Mission validation passed.")
        else:
            self.status_message.emit(f"Mission Validation FAILED: {len(errors)} collision error(s) detected.")
            logger.warning(f"Mission validation failed with errors: {errors}")

        return is_valid, errors

    def execute_mission(self) -> bool:
        """Validate and execute the active mission on physical hardware."""
        if self._active_mission.destinations and self.is_multi_robot:
            try:
                self._fleet_assignments = self.plan_destination_assignments()
            except ValueError as exc:
                self.status_message.emit(f"Cannot execute: {exc}")
                return False
            self._fleet_completed = set()
            self._fleet_executors = {}
            self._start_fleet_assignments()
            return True

        is_valid, errors = self.validate_mission()
        if not is_valid:
            self.status_message.emit(f"Cannot execute: Validation failed ({errors[0]})")
            return False

        if not self._active_mission.commands:
            self._active_mission.commands = self._active_mission.path.to_commands()

        if not self._active_mission.commands:
            self.status_message.emit("Cannot execute: Mission has no commands.")
            return False

        self.executor.set_commands(self._active_mission.commands)
        self.executor.start()
        return True

    def set_fleet(self, fleet: RobotFleet):
        """Attach the independently constructed fleet used for multi-robot runs."""
        self.fleet = fleet

    def set_auto_return_home(self, enabled: bool):
        """Enable or disable automatic return after a fleet destination."""
        self.auto_return_home = bool(enabled)

    def _start_fleet_assignments(self):
        """Start every robot destination leg in the same Qt event cycle."""
        if self.fleet is None:
            self.status_message.emit("Cannot execute: robot fleet is not initialized.")
            return

        self._fleet_routes = {}
        for assignment in self._fleet_assignments:
            self._fleet_routes.setdefault(assignment.robot_id, []).append(assignment)
        for robot_id, route in self._fleet_routes.items():
            self._start_fleet_leg(route[0], "destination", (route[0].x, route[0].y))

    def _start_fleet_leg(self, assignment: DestinationAssignment, phase: str, target):
        controller = self.fleet.controllers.get(assignment.robot_id)
        state = self.fleet.states.get(assignment.robot_id)
        if controller is None or state is None or state.pose is None:
            self.status_message.emit(f"Cannot execute: robot '{assignment.robot_id}' is unavailable.")
            return

        delta_x = target[0] - state.x
        delta_y = target[1] - state.y
        distance = math.hypot(delta_x, delta_y)
        target_heading = math.degrees(math.atan2(delta_y, delta_x)) if distance else state.heading
        turn = normalize_angle_deg(target_heading - state.heading)
        commands = []
        if abs(turn) > 0.1:
            commands.append(Command("TURN", turn))
        if distance > 0.1:
            commands.append(Command("MOVE", distance, 1.0))
        executor = NavigationExecutor(controller, state)
        executor.set_commands(commands)
        executor.completed.connect(
            lambda assignment=assignment, phase=phase: self._on_fleet_leg_completed(assignment, phase)
        )
        self._fleet_executors[assignment.robot_id] = executor
        state.set_returning_home(phase == "home")
        label = "home" if phase == "home" else assignment.destination_id
        logger.info(
            "Fleet leg started: robot=%s phase=%s current=(%.1f, %.1f) target=(%.1f, %.1f) commands=%s",
            assignment.robot_id,
            phase,
            state.x,
            state.y,
            target[0],
            target[1],
            [command.to_udp_string() for command in commands],
        )
        self.status_message.emit(f"Robot {assignment.robot_id} navigating to {label}.")
        if commands:
            executor.start()
        else:
            self._on_fleet_leg_completed(assignment, phase)

    def _on_fleet_leg_completed(self, assignment: DestinationAssignment, phase: str):
        state = self.fleet.states.get(assignment.robot_id) if self.fleet else None
        home_pose = state.initial_pose if state else None
        if home_pose is None and self.fleet:
            spec = self.fleet.roster.get(assignment.robot_id)
            if spec:
                home_pose = Pose(spec.start_x, spec.start_y, spec.heading)
        if phase == "destination":
            route = self._fleet_routes.get(assignment.robot_id, [])
            current_index = next(
                (index for index, item in enumerate(route) if item.destination_id == assignment.destination_id),
                len(route) - 1,
            )
            if current_index + 1 < len(route):
                next_assignment = route[current_index + 1]
                self._start_fleet_leg(next_assignment, "destination", (next_assignment.x, next_assignment.y))
                return
        if phase == "destination" and self.auto_return_home and home_pose:
            self._start_fleet_leg(
                assignment,
                "home",
                (home_pose.x, home_pose.y),
            )
            return
        if state:
            state.set_returning_home(False)
        logger.info("Fleet leg completed: robot=%s phase=%s pose=(%.1f, %.1f)", assignment.robot_id, phase, state.x, state.y)
        self._fleet_completed.add(assignment.robot_id)
        if len(self._fleet_completed) == len(self._fleet_routes):
            if self.auto_return_home:
                self.status_message.emit("All robots completed their destinations and returned home.")
            else:
                self.status_message.emit("All robots completed their destinations.")

    def return_all_robots_home(self):
        """Stop active fleet legs and send every configured robot to its home pose."""
        if self.fleet is None:
            self.status_message.emit("Cannot return home: robot fleet is not initialized.")
            return

        for executor in self._fleet_executors.values():
            executor.stop()
        self._fleet_executors.clear()
        self._fleet_completed.clear()
        self._fleet_routes = {}
        home_assignments = []
        for robot_id, state in self.fleet.states.items():
            home_pose = state.initial_pose
            if home_pose is None:
                spec = self.fleet.roster.get(robot_id)
                if spec:
                    home_pose = Pose(spec.start_x, spec.start_y, spec.heading)
            if home_pose is None:
                continue
            assignment = DestinationAssignment("manual_home", state.x, state.y, robot_id, True)
            home_assignments.append((assignment, home_pose))

        # Build the complete snapshot before starting any leg. This prevents a
        # zero-distance robot from completing synchronously while the list is
        # still being populated for the other robots.
        self._fleet_assignments = [assignment for assignment, _ in home_assignments]
        self._fleet_routes = {assignment.robot_id: [assignment] for assignment, _ in home_assignments}
        for assignment, home_pose in home_assignments:
            self._start_fleet_leg(assignment, "home", (home_pose.x, home_pose.y))

    def backtrack_all_robots(self):
        """Run each robot back through its own recorded trail concurrently."""
        if self.fleet is None:
            self.status_message.emit("Cannot backtrack: robot fleet is not initialized.")
            return

        for executor in self._fleet_executors.values():
            executor.stop()
        self._fleet_executors.clear()
        self._fleet_completed.clear()
        self._fleet_routes = {}

        started = 0
        for robot_id, state in self.fleet.states.items():
            if state.pose is None or not state.path_history:
                continue
            raw_nodes = []
            for stroke in state.path_history:
                for point in stroke:
                    if not raw_nodes or math.dist(point, raw_nodes[-1]) > 0.5:
                        raw_nodes.append(point)
            targets = list(reversed(raw_nodes))
            if targets and math.dist((state.x, state.y), targets[0]) < 0.5:
                targets.pop(0)
            if not targets:
                continue

            commands = []
            current_x, current_y, current_heading = state.x, state.y, state.heading
            for target_x, target_y in targets:
                distance = math.hypot(target_x - current_x, target_y - current_y)
                if distance <= 0.1:
                    continue
                target_heading = math.degrees(math.atan2(target_y - current_y, target_x - current_x))
                turn = normalize_angle_deg(target_heading - current_heading)
                if abs(turn) > 0.1:
                    commands.append(Command("TURN", turn))
                commands.append(Command("MOVE", distance, 1.0))
                current_x, current_y, current_heading = target_x, target_y, target_heading

            home_pose = state.initial_pose
            if home_pose:
                final_turn = normalize_angle_deg(home_pose.heading - current_heading)
                if abs(final_turn) > 0.1:
                    commands.append(Command("TURN", final_turn))
            if not commands:
                continue

            executor = NavigationExecutor(self.fleet.controllers[robot_id], state)
            executor.set_commands(commands)
            executor.completed.connect(
                lambda robot_id=robot_id, state=state: self._on_fleet_backtrack_completed(robot_id, state)
            )
            self._fleet_executors[robot_id] = executor
            state.set_backtracking(True)
            state.set_executing(True)
            executor.start()
            started += 1

        self.status_message.emit(f"Backtracking started for {started} robot(s).")

    def _on_fleet_backtrack_completed(self, robot_id: str, state: RobotState):
        state.set_backtracking(False)
        state.set_executing(False)
        self._fleet_completed.add(robot_id)
        if len(self._fleet_completed) == len(self._fleet_executors):
            self.status_message.emit("All robots completed backtrack via their recorded paths.")

    def pause_mission(self):
        self.executor.pause()

    def resume_mission(self):
        self.executor.resume()

    def stop_mission(self):
        self.executor.stop()
        for executor in self._fleet_executors.values():
            executor.stop()

    def reset_software(self):
        """Stop motion and restore every robot and mission to startup state."""
        self.executor.stop()
        for executor in self._fleet_executors.values():
            executor.stop()
        self._fleet_executors.clear()
        self._fleet_completed.clear()
        self._fleet_routes = {}
        if self.fleet:
            for robot_id, state in self.fleet.states.items():
                spec = self.fleet.roster.get(robot_id)
                state.reset_world()
                if spec:
                    state.set_pose(spec.start_x, spec.start_y, spec.heading, set_initial_if_unset=True)
        self.new_mission("new_mission")
        self.status_message.emit("Software reset to initial condition.")

    def _on_recording_state_changed(self, is_rec: bool):
        self.teach_mode_changed.emit(is_rec)
