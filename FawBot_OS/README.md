# FawBot OS - Modular Robot Control & Mission Path Planning Station

FawBot OS is a modular Python/PyQt5 desktop workstation for manual teleoperation and autonomous mission path planning. It refactors the legacy monolithic Turtlesim application into a clean multi-tiered architecture while preserving 100% of existing robot hardware UDP communication, motion kinematics, and Gazebo-inspired dark workbench styling.

---

## 1. Directory Structure

```text
FawBot_OS/
│
├── main.py                          # Primary application entry point
├── Advance_turtlesim.py             # Backward-compatible wrapper
├── requirements.txt                 # Project dependencies (PyQt5, matplotlib, numpy)
├── README.md                        # Complete architecture & operation documentation
│
├── config/                          # Centralized configuration & physical constants
│   ├── __init__.py
│   └── settings.py                  # Robot host/port, kinematics, dimensions, thresholds
│
├── models/                          # Data models and schemas
│   ├── __init__.py
│   ├── pose.py                      # 2D Pose (x, y, heading) with normalization & vector math
│   ├── obstacle.py                  # Boundary & RestrictedArea (No-Go polygon) models
│   ├── path.py                      # PathPoint, Command (MOVE, TURN), and Path models
│   └── mission.py                   # Full Mission schema with versioned JSON serialization
│
├── robot/                           # Physical hardware interface layer
│   ├── __init__.py
│   ├── udp_communication.py         # UDP socket manager & QThread receiver listener
│   ├── robot_state.py               # Central observable RobotState with PyQt signals
│   └── robot_controller.py          # Command dispatching, kinematics math & 30 FPS animation
│
├── navigation/                      # Navigation, geometry, planning & execution
│   ├── __init__.py
│   ├── geometry.py                  # 2D polygon collision, footprint calculation, swept checks
│   ├── path_recorder.py             # Teach/Record mode with threshold duplicate suppression
│   ├── path_planner.py              # Abstract PathPlanner with RecordedPathPlanner & future hooks
│   ├── navigation_executor.py       # Asynchronous, non-blocking hardware execution worker
│   └── mission_manager.py           # Mission lifecycle coordinator
│
├── storage/                         # Mission persistence layer
│   ├── __init__.py
│   └── mission_storage.py           # JSON save, load, list, delete & schema validation
│
├── ui/                              # User interface layer
│   ├── __init__.py
│   ├── styles.py                    # Gazebo Simulation dark theme stylesheet
│   ├── map_widget.py                # Reusable CAD grid viewport with zoom, pan & footprint
│   ├── manual_control_page.py       # Refactored Manual Control (100% backward compatible)
│   ├── mission_planner_page.py      # New Mission Path Planning studio
│   └── main_window.py               # Main window with tab navigation & global telemetry
│
├── missions/                        # Directory for saved JSON mission files
│   └── warehouse_demo_01.json       # Demo patrol mission
│
├── logs/                            # Application logs
│   └── fawbot.log                   # Structured event logs
│
└── tests/                           # Automated unit & integration tests
    ├── __init__.py
    ├── test_geometry.py             # Polygon footprint, ray-casting, segment intersection tests
    ├── test_mission_storage.py      # Serialization, corrupt JSON, versioning tests
    ├── test_path_recorder.py        # Teach mode duplicate throttling & command synthesis
    └── test_mission_manager.py      # End-to-end mission workflow & validation tests
```

---

## 2. Layered Architecture

The application enforces a strict unidirectional dependency architecture:

```text
                  UI Layer
  (MainWindow, ManualControlPage, MissionPlannerPage, MapWidget)
                         │
                         ▼
                 Navigation Layer
 (MissionManager, PathRecorder, PathPlanner, NavigationExecutor, Geometry)
                         │
                         ▼
                   Robot Layer
          (RobotController, RobotState)
                         │
                         ▼
                Communication Layer
                (UDPCommunication)
```

- **UI Layer**: Only handles user input and visual updates via Qt signals. It does not construct raw UDP packets or manipulate JSON directly.
- **Navigation Layer**: Coordinates missions, checks footprint collisions, records human driving, and manages asynchronous command dispatching.
- **Robot Controller**: Calculates stepper motor steps, turn arcs, and run durations; handles 30 FPS motion interpolation.
- **Robot State**: Single source of truth for robot pose `(x, y, heading)`, motion state, home position, and safety halt flags.
- **UDP Communication**: Dedicated socket manager and background listener thread communicating with `fawbot.local:8888`.

---

## 3. Preserved Robot Protocol & Hardware Kinematics

All hardware commands and telemetry feedback strings are strictly preserved without modification:

### Commands Sent to Robot
- **Linear Move**: `MOVE:<distance_cm>:<direction>`
  - Example: `MOVE:10.0:1.0` (Forward 10 cm), `MOVE:10.0:-1.0` (Backward 10 cm)
- **In-Place Turn**: `TURN:<angle_deg>`
  - Example: `TURN:15.0` (Turn Left 15°), `TURN:-15.0` (Turn Right 15°)

### Feedback Messages Received
- `COMPLETED:TURN` — Signals completion of physical turn
- `COMPLETED:MOVE` — Signals completion of physical linear move
- `ALERT:SAFETY_HALT` — Hardware safety sensor triggered; immediately halts execution across all pages

### Kinematics Constants
- **Wheel Base**: `10.0 cm`
- **Steps per CM**: `256.81`
- **Delay per Step**: `850.0 µs`

---

## 4. Mission Path Planning Features

### Teach / Record Mode
1. Place or spawn the robot on the grid.
2. Click **Start Recording** (turns into red "Stop Recording" button).
3. Drive the robot manually using the on-screen D-Pad or Arrow/WASD keys.
4. The system records poses, distances, and headings while intelligently suppressing duplicate/jitter points using configurable thresholds (`> 1.0 cm`, `> 2.0°`, `> 0.1 s`).
5. Click **Stop Recording**. Discrete hardware `TURN` and `MOVE` commands are synthesized automatically.

### Boundary & Restricted (No-Go) Areas
- **Boundary**: Rectangular or polygonal outer envelope (default `150 × 150 cm`).
- **Restricted Areas**: Defined as 2D polygons (hatched red on the map).
- Use **+ Add Area** to define custom obstacle zones with specified dimensions and positions.

### True Footprint & Safety Margin Validation
Validation does not treat the robot as a zero-radius mathematical point:
- Generates the oriented bounding box of the robot at each pose:
  $$\text{Effective Length} = \text{ROBOT\_LENGTH\_CM} + 2 \times \text{SAFETY\_MARGIN\_CM} = 12.0 + 4.0 = 16.0\text{ cm}$$
  $$\text{Effective Width} = \text{ROBOT\_WIDTH\_CM} + 2 \times \text{SAFETY\_MARGIN\_CM} = 9.0 + 4.0 = 13.0\text{ cm}$$
- Performs swept-volume footprint collision detection along every path segment.
- Validates that the footprint stays completely inside the environment boundary and outside all restricted polygons.
- Provides informative error messages specifying exact failing segments (e.g. `Path segment 2 collides with Restricted Area 'Storage Rack A' near (45.0, 50.0)`).

### Dry-Run Preview Animation
- Click **▶ Preview Mission (Dry Run)** to simulate the complete planned trajectory on the map using a translucent preview ghost pose before sending any commands to physical hardware.

### Independent Execution Engine
- Execution is handled by `NavigationExecutor` in a non-blocking worker pattern.
- Supports **Execute**, **Pause**, **Resume**, and **Stop**.
- Reacts immediately to `ALERT:SAFETY_HALT`.

---

## 5. Versioned Mission JSON Schema

Saved under `missions/<mission_name>.json`:

```json
{
    "version": 1,
    "mission": {
        "name": "warehouse_demo_01",
        "description": "Perimeter patrol avoiding central obstacle",
        "created_at": "2026-09-05T21:00:00.000000",
        "updated_at": "2026-09-05T21:00:00.000000"
    },
    "environment": {
        "width_cm": 150.0,
        "height_cm": 150.0,
        "boundary": [
            [0.0, 0.0],
            [150.0, 0.0],
            [150.0, 150.0],
            [0.0, 150.0]
        ],
        "restricted_areas": [
            {
                "id": "storage_rack_a",
                "name": "Storage Rack A",
                "polygon": [
                    [40.0, 40.0],
                    [75.0, 40.0],
                    [75.0, 75.0],
                    [40.0, 75.0]
                ],
                "color": "#ff3366",
                "active": true
            }
        ]
    },
    "robot": {
        "length_cm": 12.0,
        "width_cm": 9.0,
        "safety_margin_cm": 2.0,
        "wheel_base_cm": 10.0
    },
    "start_pose": {
        "x": 20.0,
        "y": 20.0,
        "heading": 0.0,
        "timestamp": 0.0
    },
    "path": [
        {
            "timestamp": 0.0,
            "x": 20.0,
            "y": 20.0,
            "heading": 0.0,
            "action": "START",
            "distance": 0.0,
            "rotation": 0.0
        }
    ],
    "commands": [
        {
            "type": "MOVE",
            "distance_cm": 85.0,
            "direction": 1.0
        },
        {
            "type": "TURN",
            "angle_deg": 90.0
        }
    ]
}
```

---

## 6. How to Run

### Activate the Environment
```bash
conda activate robotcar
```

### Launch FawBot OS
```bash
python main.py
```

### Backward-Compatible Launch
```bash
python Advance_turtlesim.py
```

### Run the Automated Test Suite
```bash
python -m unittest discover -s tests -p "test_*.py" -v
```

---

## 7. Future Extensibility
The modular architecture exposes clear abstract interfaces:
- **`PathPlanner` in `navigation/path_planner.py`**: Add algorithms like A*, RRT*, Dijkstra, or Potential Fields by inheriting from `PathPlanner` and implementing `plan(start, goal, environment)`.
- **Vision / Localization**: Overhead cameras, ArUco markers, or external odometry can update `RobotState.set_pose(x, y, heading)` directly without modifying any UI or navigation classes.
- **Multiple Robots**: Instantiate multiple `RobotController` and `RobotState` pairs under a swarm coordinator.
