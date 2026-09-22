# FawBot OS

FawBot OS is a Python/PyQt5 workstation for controlling FawBot ESP32 robots, planning missions, recording paths, managing a robot fleet, and building a live map from a front-mounted VL53L0X time-of-flight sensor.

The repository contains two cooperating applications:

1. **Desktop application**: Python, PyQt5, Matplotlib, and UDP networking.
2. **Robot firmware**: ESP32 Arduino firmware built with PlatformIO.

The desktop application does not directly drive motors. It sends commands to the ESP32, and the ESP32 performs the physical motion, safety checks, sensor reads, and LiDAR sweep.

---

## Features

- Manual robot teleoperation with keyboard, D-pad, and UDP commands
- Mission planning with waypoints, boundaries, restricted areas, and validation
- Teach/record mode for creating missions from manual driving
- Dry-run mission preview
- Multi-robot fleet configuration
- Asynchronous mission execution with pause, resume, and stop
- IR floor/cliff sensing and ultrasonic obstacle sensing
- VL53L0X LiDAR scanning over I2C
- Continuous LiDAR mapping: sweep, move forward, confirm position, repeat
- Persistent wall points and ash-gray measurement rays
- Open-space white rays for zero/invalid range readings
- Adjacent wall-segment rendering with gap filtering
- Click-to-goal navigation around measured wall-line obstacles
- Dedicated LiDAR-page emergency stop
- Front-mounted sensor geometry and 11 x 9 cm robot footprint
- LiDAR map zoom, pan, and Fit View controls

---

## Repository Structure

```text
FawBot_OS/
├── main.py                         Python GUI entry point
├── Advance_turtlesim.py            Backward-compatible launcher
├── requirements.txt                Python dependencies
├── README.md                       This documentation
│
├── config/
│   ├── __init__.py
│   └── settings.py                 Network, geometry, timing, and UI constants
│
├── models/
│   ├── __init__.py
│   ├── pose.py                     X/Y/heading pose and angle normalization
│   ├── obstacle.py                 Boundary and restricted-area models
│   ├── path.py                     Path points, MOVE/TURN commands, and paths
│   └── mission.py                  Versioned mission data and JSON serialization
│
├── robot/
│   ├── __init__.py
│   ├── udp_communication.py        UDP socket and background receive thread
│   ├── robot_state.py              Observable pose, flags, safety, and trails
│   ├── robot_controller.py         Motion commands and virtual animation
│   ├── fleet.py                    Robot roster and robot specifications
│   └── fleet_controller.py         Independent communication/state per robot
│
├── navigation/
│   ├── __init__.py
│   ├── geometry.py                 Footprints, polygons, collisions, validation
│   ├── lidar_protocol.py           LiDAR packet decoding and text parsing
│   ├── path_recorder.py            Teach/record mode point filtering
│   ├── path_planner.py             Planner interface and recorded-path planner
│   ├── navigation_executor.py     Background mission execution worker
│   └── mission_manager.py          Mission lifecycle and fleet coordination
│
├── storage/
│   ├── __init__.py
│   └── mission_storage.py          Save, load, list, delete, and validate missions
│
├── ui/
│   ├── __init__.py
│   ├── styles.py                   PyQt and Matplotlib dark workbench theme
│   ├── map_widget.py               Mission/manual CAD map widget
│   ├── manual_control_page.py      Manual driving page
│   ├── mission_planner_page.py     Mission planning page
│   ├── swarming_page.py            Fleet/swarming page
│   ├── lidar_mapping_page.py       Live VL53L0X mapping page
│   └── main_window.py              Tabs, shared state, and global telemetry
│
├── missions/                       Saved mission JSON files
├── logs/                            Runtime log files
├── tests/                           Python unit and integration tests
│
└── embedded_code/
    ├── platformio.ini              ESP32 environments and dependencies
    ├── assigned_robot.txt          Robot ID, hostname, port, and start pose
    ├── README.md                   Embedded hardware and API documentation
    ├── include/config.h            Pins, constants, shared variables, prototypes
    ├── src/main.cpp                 ESP32 setup and main loop
    ├── src/sensors.cpp              IR, ultrasonic, and VL53L0X LiDAR logic
    ├── src/stepper_control.cpp      Motor stepping, movement, and turns
    ├── src/web_portal.cpp            HTTP dashboard, SSE, and UDP protocol
    ├── lib/                         Private PlatformIO libraries
    └── test/                        PlatformIO test directory
```

### Python module reference

| Module | Responsibility |
|---|---|
| `main.py` | Creates the Qt application, configures logging, and opens `MainWindow`. |
| `Advance_turtlesim.py` | Compatibility launcher for older workflows. |
| `config/settings.py` | Central network settings, dimensions, kinematics, thresholds, timing, and directories. |
| `models/pose.py` | Pose values, heading normalization, and vector/angle helpers. |
| `models/obstacle.py` | Boundary and restricted-area polygon data models. |
| `models/path.py` | Path points and executable `MOVE`/`TURN` command conversion. |
| `models/mission.py` | Mission, environment, robot, path, and versioned JSON schema. |
| `robot/udp_communication.py` | UDP socket creation, background listener thread, feedback signals, and command transmission. |
| `robot/robot_state.py` | Observable pose, execution flags, safety state, initial pose, and travel trails. |
| `robot/robot_controller.py` | Sends motion commands and interpolates the virtual robot for ordinary manual/mission motion. |
| `robot/fleet.py` | Parses `assigned_robot.txt` and defines robot identities/start poses. |
| `robot/fleet_controller.py` | Owns independent communication, state, and controller objects for each robot. |
| `navigation/geometry.py` | Footprint generation, polygon tests, segment tests, and swept trajectory validation. |
| `navigation/lidar_protocol.py` | Decodes firmware 5-byte hexadecimal LiDAR packets. |
| `navigation/path_recorder.py` | Filters jitter and converts teach-mode driving into mission points. |
| `navigation/path_planner.py` | Planner abstraction and recorded-path implementation; autonomous planner hooks are reserved here. |
| `navigation/navigation_executor.py` | Runs mission command execution without blocking the GUI. |
| `navigation/mission_manager.py` | Coordinates mission lifecycle, validation, execution, and fleet operations. |
| `storage/mission_storage.py` | Saves, loads, lists, deletes, and validates mission JSON files. |
| `ui/main_window.py` | Builds tabs, global status bar, keyboard teleoperation, shared state wiring, and shutdown. |
| `ui/manual_control_page.py` | Manual drive controls, robot map, home/backtrack controls, and telemetry. |
| `ui/mission_planner_page.py` | Mission editor, waypoints, boundaries, restricted areas, validation, preview, and execution controls. |
| `ui/swarming_page.py` | Multi-robot destination assignment and fleet operations. |
| `ui/map_widget.py` | Reusable Matplotlib CAD viewport with grid, robot footprints, trails, targets, and mission overlays. |
| `ui/lidar_mapping_page.py` | VL53L0X stream, dynamic rays/walls, sweep state, pose handshake, route planning, goal navigation, and emergency stop. |
| `ui/styles.py` | Shared PyQt and Matplotlib dark workbench styling. |

### Embedded module reference

| File | Responsibility |
|---|---|
| `embedded_code/platformio.ini` | ESP32 board/framework settings, libraries, robot build environments, and per-robot pin/name flags. |
| `embedded_code/assigned_robot.txt` | Desktop fleet roster: robot ID, mDNS host, UDP port, initial X/Y, and heading. |
| `embedded_code/include/config.h` | Shared includes, motor/sensor pins, I2C pins, mechanical constants, LiDAR defaults, globals, and function declarations. |
| `embedded_code/src/main.cpp` | `setup()`, main loop, 100 ms sensor updates, safety halt evaluation, LiDAR job dispatch, and ordinary motion dispatch. |
| `embedded_code/src/sensors.cpp` | IR/ultrasonic reads, VL53L0X initialization, sweep stepping, packet encoding, continuous mapping, and confirmed pose feedback. |
| `embedded_code/src/stepper_control.cpp` | 8-state half-step motor sequence, speed delay, centimeter movement, turn movement, manual stepping, and motor stop. |
| `embedded_code/src/web_portal.cpp` | Wi-Fi/mDNS, HTTP dashboard routes, SSE event stream, UDP command parser, UDP feedback, and stop handling. |
| `embedded_code/.pio/` | PlatformIO generated dependencies/build products; do not edit manually. |
| `embedded_code/lib/` | Location for project-private PlatformIO libraries. |
| `embedded_code/test/` | PlatformIO test-runner directory. |

---

## System Architecture

```text
+-----------------------+       UDP commands / feedback       +----------------------+
| Python FawBot OS GUI  | <---------------------------------> | ESP32 robot firmware |
| PyQt5 + Matplotlib    |                                    | Motors + sensors     |
+-----------+-----------+                                    +----------+-----------+
            |                                                           |
            | local application state                                   |
            v                                                           v
   RobotState / MissionManager                              Stepper motors / VL53L0X
   Map and LiDAR renderer                                   IR / ultrasonic safety

ESP32 HTTP/SSE stream  ----------------------------------->  LiDAR packets + pose events
```

### Python application layers

- **UI** receives user input and renders state.
- **Navigation** plans, records, validates, and executes paths.
- **Robot** owns commands, observable state, and fleet communication.
- **Models** define poses, obstacles, paths, and missions.
- **Storage** handles mission JSON files.

### Firmware layers

- `main.cpp` runs setup, network polling, safety checks, LiDAR jobs, and motion dispatch.
- `sensors.cpp` reads IR, HC-SR04 ultrasonic, and VL53L0X sensors.
- `stepper_control.cpp` converts centimeters/degrees into motor steps.
- `web_portal.cpp` hosts the dashboard, SSE stream, HTTP routes, and UDP server.

---

## Robot Hardware

The firmware targets an ESP32 Dev Module with:

- Two 28BYJ-48 stepper motors
- Two ULN2003 motor drivers
- Two active-low IR floor/cliff sensors
- HC-SR04 ultrasonic sensor
- VL53L0X time-of-flight sensor on I2C

Default LiDAR pins:

| Signal | ESP32 pin |
|---|---:|
| VL53L0X SDA | GPIO 21 |
| VL53L0X SCL | GPIO 22 |

The LiDAR is mounted at the **front center** of the robot. The GUI uses an 11 cm robot length and 9 cm width. The sensor origin is therefore fixed 5.5 cm in front of the robot center along the robot heading; only the measurement ray direction changes during a sweep.

A LiDAR distance of 1 cm means the wall is 1 cm beyond the front-mounted sensor, not 1 cm from the robot center.

### Motion and kinematics

The firmware drives both 28BYJ-48 motors with an 8-state half-step sequence.
Current embedded defaults are:

| Value | Default |
|---|---:|
| Wheel diameter | `5.0 cm` |
| Wheel base | `10.2 cm` |
| Calibrated steps per centimeter | `256.81` |
| Startup step delay | `1000 us` |
| Speed delay range | `3000 us` to `650 us` |
| LiDAR step angle | `1.0 deg` |

Linear movement is sent as `MOVE:distance:direction`. In-place rotation is
sent as `TURN:angle`. The desktop mission controller uses mathematical headings
where `0 deg` points along +X and `90 deg` points along +Y. The LiDAR goal
planner applies the configured route turn sign when converting that heading into
the physical motor command for the current wiring profile.

---

## Network Configuration

Python reads robot entries from `embedded_code/assigned_robot.txt`:

```csv
robot_id,hostname,udp_port,start_x,start_y,heading
fawbot_31,fawbot_31.local,8888,60,20,90
fawbot_22,fawbot_22.local,8888,0,20,90
fawbot_23,fawbot_23.local,8888,30,20,90
```

Each configured robot receives its own `UDPCommunication`, `RobotState`, and `RobotController` instance.

Important configuration files:

- Python network defaults: `config/settings.py`
- Robot roster: `embedded_code/assigned_robot.txt`
- Firmware robot profile: `embedded_code/platformio.ini`
- Firmware pins/constants: `embedded_code/include/config.h`

---

## Communication Between Python and Robot

### UDP transport

The Python application creates a UDP socket for each robot and listens in a Qt background thread. The firmware listens on UDP port `8888` and remembers the most recent Python sender so it can return feedback to the same address and port.

### Commands sent from Python

```text
MOVE:<distance_cm>:<direction>
TURN:<angle_degrees>
STOP
TOGGLE:ir
TOGGLE:us
CFG:delay:<microseconds>
CFG:obs:<distance_cm>
CFG:lidar_step:<degrees>
LIDAR:<sweep_degrees>:<forward_cm>:<x>:<y>:<heading>
```

Examples:

```text
MOVE:10.0:1.0
TURN:15.0
STOP
CFG:lidar_step:1
LIDAR:180:10:60.00:20.00:90.00
```

### Feedback sent by the firmware

```text
COMPLETED:MOVE:<distance_cm>
COMPLETED:TURN:<angle_degrees>
ACK:CFG
ACK:TOGGLE
ACK:LIDAR
ALERT:EMERGENCY_STOP
ALERT:SAFETY_HALT
REACHED:LIDAR:<x>:<y>:<heading>
```

The Python controller uses completion and alert messages to update execution state. The LiDAR page uses the LiDAR-specific messages described below.

The firmware also publishes these SSE event names on `/events`:

| Event | Payload | Consumer |
|---|---|---|
| `sensor_data` | IR left/right state and ultrasonic distance JSON | Web dashboard and telemetry UI |
| `log` | Human-readable firmware log | Web dashboard |
| `lidar_packet` | Encoded VL53L0X measurement | LiDAR mapping page |
| `robot_pose` | Confirmed `REACHED:LIDAR` pose | LiDAR mapping page |

---

## LiDAR Data and Mapping Protocol

### LiDAR packet format

For every VL53L0X measurement, the firmware creates a 5-byte packet and sends it as 10 hexadecimal characters.

The packet contains:

- Packet type marker
- Angle encoded in Q6 fixed-point format
- Distance encoded from millimeters in Q2 fixed-point format

The Python decoder is in `navigation/lidar_protocol.py`.

LiDAR packets are delivered through both channels:

1. UDP:

```text
LIDAR_PACKET:<10_hex_characters>
```

2. HTTP Server-Sent Events:

```text
event: lidar_packet
data: <10_hex_characters>
```

The GUI accepts both channels and suppresses duplicate packets.

### Starting continuous mapping

The GUI sends this command:

```text
LIDAR:<sweep_degrees>:<forward_cm>:<x>:<y>:<heading>
```

Example:

```text
LIDAR:180:10:60.00:20.00:90.00
```

The firmware then repeatedly:

1. Rotates through the first half of the sweep.
2. Reads and publishes LiDAR measurements.
3. Rotates through the second half of the sweep.
4. Returns to the original heading.
5. Moves forward by the requested distance.
6. Reports the confirmed new pose.
7. Starts the next sweep.

### Confirmed pose handshake

The firmware sends a pose only after the physical forward move completes:

```text
REACHED:LIDAR:<x>:<y>:<heading>
```

The same event is also sent through SSE as:

```text
event: robot_pose
data: REACHED:LIDAR:<x>:<y>:<heading>
```

The GUI does **not** advance the virtual robot when a scan packet arrives. It advances only after `REACHED:LIDAR` feedback. UDP and SSE duplicates are ignored.

This handshake keeps the virtual map pose synchronized with the real robot.

The GUI also listens for `LIDAR_PACKET:<hex>` on UDP because UDP is the command
socket used by the desktop process. The same packet is available through SSE;
duplicate UDP/SSE packets and duplicate pose confirmations are ignored.

### LiDAR step angle

The firmware default is configured by:

```cpp
#define LIDAR_STEP_ANGLE_DEG 1.0f
```

The step can also be changed at runtime from the firmware webpage or the GUI LiDAR page. The GUI sends:

```text
/config?param=lidar_step&val=1
```

The GUI must use the same step angle as the firmware. A mismatch causes wall points to appear mirrored or shifted.

### Map rendering

The LiDAR page stores each measurement in world coordinates:

- Ash-gray line: sensor-to-measured-point ray
- Red endpoint: detected wall return
- White max-range line: no wall/invalid range for that angle
- Ash-gray solid trail: robot travel path
- Yellow footprint: robot at 11 x 9 cm
- Red adjacent wall segments: completed sweep wall returns

Raw wall points are sorted by sweep angle. Adjacent points are connected only
when their distance is at most `1.0 cm`; gaps remain open and the scan is never
closed into a last-point-to-first-point loop. Raw points are removed after the
completed sweep segments are generated, and the latest segment set replaces the
previous one.

The page supports:

- Mouse-wheel zoom
- Middle-button pan
- Fit View
- Clear map
- Maximum range filtering
- Sweep angle and forward distance controls

### Click-to-goal navigation

The LiDAR page records the first confirmed robot pose as **Home**. While a map
is available, click a location inside the plot to select a **Goal**, then click
**Navigate to Goal** to begin navigation. Selecting a point does not move the
robot by itself. The page
inflates the red wall-line segments by the robot footprint and safety clearance,
then uses a fine 2 cm, 8-connected A* search. Diagonal corner cutting is
blocked, and a line-of-sight simplifier removes unnecessary grid corners. The
result is a sequence of straight route legs drawn in blue before motion starts.

The planner treats each red wall segment as an obstacle. A route leg is accepted
only when its sampled line remains outside the inflated wall segments.

For each LiDAR sweep, the mapper keeps the nearest return for each angular ray.
A farther return behind a nearer wall is discarded, while points from previous
robot positions remain in the accumulated map.

The continuous LiDAR routine is paused before navigation because the current
firmware sweep is a blocking operation. The GUI sends sequential `TURN` and
`MOVE` commands for the planned route and updates the virtual robot only after
the corresponding `COMPLETED` feedback arrives. The firmware IR and ultrasonic
safety halt remains active during the route. Route turn signs are calibrated
separately from the mathematical map heading so the physical robot and virtual
robot face the same direction. Restart continuous mapping after the robot
reaches the goal if a new map is required.

Clicking a map position only selects a goal. The robot starts only after
**Navigate to Goal** is pressed. **Emergency Stop** sends `STOP`, cancels the
queued route and continuous mapping, and leaves the current map visible.

---

## Firmware Web Dashboard

The firmware hosts a dashboard at:

```text
http://<robot-hostname>/Fawbot
```

Example:

```text
http://fawbot_31.local/Fawbot
```

The dashboard provides:

- Manual D-pad control
- MOVE and TURN controls
- IR and ultrasonic toggles
- LiDAR start/stop
- Continuous LiDAR sweep angle
- LiDAR step angle
- Forward step distance
- Live sensor and log events through SSE

The web dashboard and Python GUI share the same robot firmware. Do not run two independent continuous LiDAR scans at the same time.

---

## Running the Project

### 1. Create or activate the Python environment

```bash
conda activate robotcar
```

Install dependencies:

```bash
python -m pip install -r requirements.txt
```

### 2. Configure the robot roster

Edit:

```text
embedded_code/assigned_robot.txt
```

Confirm the robot hostname, UDP port, starting X/Y position, and heading.

### 3. Build the firmware

From the project root:

```bash
cd embedded_code
pio run -e fawbot31
```

Other configured environments include `fawbot22` and `fawbot23`.

### 4. Upload the firmware

Connect the ESP32 by USB and run:

```bash
pio run -e fawbot31 -t upload
```

Reflash after changing firmware files. Python-only GUI changes do not require reflashing.

### 5. Start the Python GUI

From the project root:

```bash
python main.py
```

Backward-compatible launcher:

```bash
python Advance_turtlesim.py
```

---

## Operating the LiDAR Mapper

1. Power the robot and wait for the `VL53L0X lidar ready` firmware log.
2. Confirm the robot is connected to the same network as the computer.
3. Start the Python GUI.
4. Open the **LiDAR Mapping** tab.
5. Set the same LiDAR step angle used by the firmware, normally `1 deg`.
6. Set the sweep angle, normally `180 deg`.
7. Set the forward distance, for example `10 cm`.
8. Click **Clear** before a new mapping run.
9. Click **Start 180 deg / 10 cm**.
10. Watch for LiDAR rays, red wall endpoints, robot movement, and confirmed pose updates.
11. Click **Stop Continuous Scan** to stop the robot.
12. Use **Emergency Stop** to immediately send `STOP` and cancel scanning or route navigation.
13. Use **Fit View** to fit the accumulated map.

Start with a small forward distance in an open area. Keep the physical robot clear of obstacles and be ready to use the stop control.

---

## Safety Behavior

The firmware can stop movement when:

- Either IR sensor reports loss of floor
- Ultrasonic distance falls below `obstacleThreshold`
- A `STOP` command is received
- A LiDAR continuous scan is stopped

The Python GUI also reacts to safety feedback and updates the global safety indicator.

The LiDAR page emergency-stop button is independent of map clearing: it stops
motion and queued commands but leaves the rendered map available for diagnosis.

The LiDAR sensor is used for mapping in the current implementation. IR and ultrasonic sensors provide the primary firmware safety halt behavior.

---

## Mission Planning Workflow

1. Open **Manual Control** or **Mission Path Planning**.
2. Place the robot on the map or use the configured starting pose.
3. Define the environment boundary.
4. Add restricted/no-go polygons.
5. Record a path manually or add waypoints.
6. Validate the path against robot footprint and safety margin.
7. Preview the mission with the dry-run animation.
8. Execute only after validation succeeds.

The planner uses the robot footprint rather than treating the robot as a point. Collision checks include polygon overlap and swept path checks.

---

## Tests

Run the Python test suite:

```bash
python -m unittest discover -s tests -p "test_*.py" -v
```

Important test areas:

- `test_geometry.py`: footprint and collision geometry
- `test_mission_storage.py`: JSON persistence and schema behavior
- `test_path_recorder.py`: teach mode filtering and command creation
- `test_mission_manager.py`: mission lifecycle and validation
- `test_robot_fleet.py`: roster and multi-robot assignment
- `test_lidar_mapping.py`: LiDAR packet decoding

Build the embedded firmware with PlatformIO to validate C++ changes:

```bash
cd embedded_code
pio run -e fawbot31
```

---

## Troubleshooting

### GUI does not start

Check the selected Python environment and install the dependencies:

```bash
python -m pip install -r requirements.txt
```

### Robot is offline

Check Wi-Fi, mDNS hostname, `assigned_robot.txt`, and UDP port `8888`. The robot and computer must be on the same network.

### LiDAR page shows no points

Check:

- Firmware log says `VL53L0X lidar ready`.
- The updated firmware is flashed.
- The GUI host matches the robot hostname.
- The LiDAR step angle matches on the GUI and firmware.
- The SSE endpoint `http://<robot>/events` is reachable.
- The robot is not already running a scan from the web dashboard.

### Map points are mirrored or shifted

Stop the scan, clear the map, and use the same sweep and step angle on both sides. The GUI assumes the sensor is mounted at the front center and uses the 11 x 9 cm footprint.

### Virtual robot does not move

The GUI moves only after receiving:

```text
REACHED:LIDAR:x:y:heading
```

Reflash the firmware containing the confirmed pose handshake and ensure the Python process is the active UDP client.

### GUI becomes slow during scanning

The LiDAR page batches redraws and uses a Matplotlib `LineCollection`. Clear the map before a new run and avoid allowing an unlimited scan to accumulate indefinitely.

### Emergency stop

Use the GUI stop button, the firmware web dashboard stop button, or send:

```text
STOP
```

---

## Extension Points

- Add A*, RRT*, Dijkstra, or potential-field planning through `navigation/path_planner.py`.
- Add camera or ArUco localization by updating `RobotState.set_pose()`.
- Add persistent map storage from the LiDAR page.
- Add a dedicated LiDAR motor/sensor mount instead of rotating the full robot.
- Extend `RobotFleet` for additional robots and independent map layers.

---

## License and Contributions

This repository is an internal robotics project. Keep protocol changes synchronized between the Python GUI, firmware, and this README. Build and test both applications when changing communication messages or LiDAR behavior.
