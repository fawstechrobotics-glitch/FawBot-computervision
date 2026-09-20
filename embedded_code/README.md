# FawBot Embedded Firmware

Firmware for the **FawBot** differential-drive mobile robot fleet, built on the **ESP32** microcontroller using **PlatformIO** and the **Arduino** framework.

This embedded firmware provides dual 28BYJ-48 stepper motor actuation via ULN2003 drivers, real-time edge/cliff and ultrasonic obstacle detection, mDNS local network discovery, an asynchronous HTTP web dashboard with Server-Sent Events (SSE), and a low-latency bidirectional UDP socket server designed for computer vision, ROS, and Sim2Real trajectory execution.

---

## Table of Contents
1. [System Architecture](#system-architecture)
2. [Hardware Specifications & Pinout](#hardware-specifications--pinout)
3. [Fleet Configuration & Build Environments](#fleet-configuration--build-environments)
4. [Kinematics & Motion Control](#kinematics--motion-control)
5. [Safety & Obstacle Avoidance](#safety--obstacle-avoidance)
6. [Robot Control APIs](#robot-control-apis)
   - [UDP Socket API (Port 8888)](#1-udp-socket-api-port-8888---primary-automation-interface)
   - [HTTP REST API (Port 80)](#2-http-rest-api-port-80)
   - [Server-Sent Events (SSE) Stream](#3-server-sent-events-sse-stream-events)
   - [Web Dashboard UI](#4-web-dashboard-ui-fawbot)
7. [Python Client Example](#python-client-example)
8. [Building, Flashing, and Setup](#building-flashing-and-setup)
9. [Troubleshooting & Calibration](#troubleshooting--calibration)

---

## System Architecture

```
                      +-----------------------------+
                      |   Computer Vision / Host    |
                      |   (Python / OpenCV / ROS)   |
                      +--------------+--------------+
                                     |
                          Bidirectional UDP (8888)
                                     |
                                     v
+-------------------------------------------------------------------------+
| ESP32 Microcontroller (FawBot Firmware)                                 |
|                                                                         |
|  +-------------------------+            +----------------------------+  |
|  |       WiFi & mDNS       |            |     ESPAsyncWebServer      |  |
|  |  (<ROBOT_NAME>.local)   |            |  - /Fawbot (Dashboard UI)  |  |
|  +------------+------------+            |  - REST Control Endpoints  |  |
|               |                         |  - /events (SSE Telemetry) |  |
|               v                         +----------------------------+  |
|  +-------------------------+                                            |
|  |    handleUDP() Server   |                                            |
|  |  - MOVE / TURN Commands |                                            |
|  |  - STOP / Emergency     |                                            |
|  |  - D-Pad Continuous     |                                            |
|  |  - Sensor / CFG Toggles |                                            |
|  +------------+------------+                                            |
|               |                                                         |
|               v                                                         |
|  +-------------------------------------+   +-------------------------+  |
|  |     Motion & Kinematics Engine      |   | Sensor Subsystem (10Hz) |  |
|  |  - 8-Step Half-Stepping Sequence    |   | - Left/Right IR Floor   |  |
|  |  - Distance & Angle Calculations    |   | - Ultrasonic HC-SR04    |  |
|  |  - Sub-step Obstacle Polling (20st) |<--| - Dynamic Safety Halt   |  |
|  |  - Mid-motion Emergency Stop Poll   |   +-------------------------+  |
|  +------------------+------------------+                                |
+---------------------|---------------------------------------------------+
                      |
        +-------------+-------------+
        |                           |
        v                           v
+-------------------+       +-------------------+
| ULN2003 Driver M1 |       | ULN2003 Driver M2 |
| Left Stepper      |       | Right Stepper     |
+-------------------+       +-------------------+
```

---

## Hardware Specifications & Pinout

### Core Specifications
- **MCU**: ESP32 Dev Module (Espressif ESP32-WROOM-32, Dual-core Xtensa LX6 @ 240MHz).
- **Drive System**: Differential-drive using two **28BYJ-48 (5V)** unipolar stepper motors.
- **Motor Drivers**: Dual **ULN2003A** Darlington transistor arrays.
- **Sensors**:
  - 2x Active-LOW Infrared reflectance sensors (for floor detection / cliff avoidance / line tracing).
  - 1x HC-SR04 Ultrasonic Distance Sensor.

### Pinout Matrix

| Function | Default Pin | Fawbot_23 Override | Type | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Motor 1 IN1** (Left) | `GPIO 13` | `GPIO 13` | Output | Phase A |
| **Motor 1 IN2** (Left) | `GPIO 12` | `GPIO 12` | Output | Phase B |
| **Motor 1 IN3** (Left) | `GPIO 14` | `GPIO 14` | Output | Phase C |
| **Motor 1 IN4** (Left) | `GPIO 27` | `GPIO 27` | Output | Phase D |
| **Motor 2 IN5** (Right) | `GPIO 26` | `GPIO 32` | Output | Phase A (Swapped on Fawbot_23) |
| **Motor 2 IN6** (Right) | `GPIO 25` | `GPIO 33` | Output | Phase B (Swapped on Fawbot_23) |
| **Motor 2 IN7** (Right) | `GPIO 33` | `GPIO 25` | Output | Phase C (Swapped on Fawbot_23) |
| **Motor 2 IN8** (Right) | `GPIO 32` | `GPIO 26` | Output | Phase D (Swapped on Fawbot_23) |
| **IR Left** | `GPIO 34` | `GPIO 34` | Input | Active-LOW (`LOW` = Floor present) |
| **IR Right** | `GPIO 35` | `GPIO 35` | Input | Active-LOW (`LOW` = Floor present) |
| **Ultrasonic TRIG** | `GPIO 5` | `GPIO 5` | Output | 10 µs pulse |
| **Ultrasonic ECHO** | `GPIO 18` | `GPIO 18` | Input | Measured via `pulseIn()` (15 ms timeout) |

> **Note on Stepper Pinouts**: Pins can be overridden per robot profile in `platformio.ini` using `-D` build flags without modifying the C++ source code.

---

## Fleet Configuration & Build Environments

Multiple robots operate concurrently on the same local network. Each robot is differentiated by its `ROBOT_NAME`, mDNS hostname, and optional pin overrides.

### PlatformIO Environments (`platformio.ini`)

- **`env:fawbot31`**:
  ```ini
  [env:fawbot31]
  extends = env:esp32dev
  build_flags =
      -D ROBOT_NAME=\"Fawbot_31\"
  ```
- **`env:fawbot22`**:
  ```ini
  [env:fawbot22]
  extends = env:esp32dev
  build_flags =
      -D ROBOT_NAME=\"Fawbot_22\"
  ```
- **`env:fawbot23`**:
  ```ini
  [env:fawbot23]
  extends = env:esp32dev
  lib_deps =
      ${env:esp32dev.lib_deps}
      arduino-libraries/Stepper@^1.1.3
  build_flags =
      -D ROBOT_NAME=\"Fawbot_23\"
      -D IN5=32
      -D IN6=33
      -D IN7=25
      -D IN8=26
  ```

### Fleet Roster (`assigned_robot.txt`)

The assigned fleet roster is structured as:
`ROBOT_ID,MDNS_HOSTNAME,UDP_PORT,INITIAL_X,INITIAL_Y,INITIAL_THETA`

```csv
fawbot_31,fawbot_31.local,8888,60,20,90
fawbot_22,fawbot_22.local,8888,0,20,90
fawbot_23,fawbot_23.local,8888,30,20,90
```

---

## Kinematics & Motion Control

### Mechanical Constants (`config.h`)
- **Steps per Motor Revolution**: `4096.0` steps (in 8-step half-step mode with internal gear reduction).
- **Wheel Diameter ($D$)**: `6.5 cm`
- **Wheel Circumference ($C$)**: $\pi \times 6.5 \approx 20.42035\text{ cm}$
- **Wheelbase Width ($W$)**: `10.0 cm` (distance between left and right wheel contact points).
- **Calibrated Resolution**: **`256.81 steps / cm`** (`STEPS_PER_CM`)

### Stepping Sequence
Motors use an 8-state half-stepping sequence (`stepSequence`) to maximize torque and resolution while minimizing vibration:
```cpp
const uint8_t stepSequence[8] = {
    0b1000, 0b1100, 0b0100, 0b0110,
    0b0010, 0b0011, 0b0001, 0b1001
};
```

### Motion Equations

1. **Linear Translation (`moveRobotCm(distanceCm, direction)`):**
   $$\text{Target Steps} = |\text{distanceCm} \times 256.81|$$
   - Forward (`direction > 0`): Left Motor = `-1`, Right Motor = `-1`
   - Backward (`direction <= 0`): Left Motor = `+1`, Right Motor = `+1`

2. **In-Place Rotation (`turnRobot(degrees)`):**
   $$\text{Arc Length (cm)} = (\pi \times W) \times \frac{|\text{degrees}|}{360.0} = (31.4159\text{ cm}) \times \frac{|\text{degrees}|}{360.0}$$
   $$\text{Target Steps} = |\text{Arc Length} \times 256.81|$$
   - Clockwise / Turn Right (`degrees > 0`): Left Motor = `+1`, Right Motor = `-1`
   - Counter-Clockwise / Turn Left (`degrees < 0`): Left Motor = `-1`, Right Motor = `+1`

3. **Speed Regulation:**
   - User speed ranges from `1%` to `100%`.
   - Step delay mapped between `3000 µs` (slowest) and `650 µs` (fastest):
     $$\text{speedDelayUs} = \text{map}(\text{speedPercent}, 1, 100, 3000, 650)$$

4. **Coil Protection (`stopMotors()`):**
   When motion completes or stops, all 8 driver output pins are written `LOW`. This eliminates holding current through the ULN2003 darlingtons, preventing stepper heating and conserving battery life.

---

## Safety & Obstacle Avoidance

The safety subsystem prevents collisions and falls through three layers of protection:

1. **Sensor Health Polling (Every 100 ms)**
   - Left and right IR sensors are read. `LOW` means surface is present; `HIGH` indicates the floor is absent (cliff edge detected) or an obstacle is reached.
   - Ultrasonic distance is computed using standard sound propagation:
     $$\text{Distance (cm)} = \frac{\text{Echo Pulse Duration (\mu s)} \times 0.034}{2.0}$$
   - If distance drops below `obstacleThreshold` (default `20 cm`), `safetyHalt` is set to `true`.

2. **Sub-Step Obstacle Detection (Every 20 Steps)**
   - Inside `stepMotors()`, sensors are polled every 20 steps. If an obstacle or floor loss is detected during forward motion, execution breaks immediately and enters safety halt.

3. **Interruptible Emergency Stop (`serviceEmergencyStop()`)**
   - Even when executing a long blocking move or turn, the inner stepping loop calls `serviceEmergencyStop()`, polling UDP packets on every step.
   - If a `"STOP"` datagram arrives, `emergencyStop` is immediately raised, motor coils are depowered, and the move terminates within microseconds.

---

## Robot Control APIs

FawBot provides multiple interfaces for control:

### 1. UDP Socket API (Port 8888) - Primary Automation Interface

The UDP interface is the fastest, lowest-latency protocol designed for closed-loop Python controllers, OpenCV ArUco tracking, and automated navigation scripts.

When a command packet is received, FawBot records the sender's IP and port, executes the action, and immediately replies with feedback.

| Command Datagram | Parameters | Description | Response / Acknowledgment |
| :--- | :--- | :--- | :--- |
| `MOVE:<cm>[:<dir>]` | `cm` (float): Distance in cm<br>`dir` (float, optional): `1` for forward (default), `-1` for backward | Moves the robot by the specified distance in centimeters. | `COMPLETED:MOVE:<cm>`<br>`ALERT:SAFETY_HALT`<br>`ALERT:EMERGENCY_STOP` |
| `TURN:<deg>` | `deg` (float): Angle in degrees. Positive = Right/CW, Negative = Left/CCW | Rotates robot in place by specified angle. | `COMPLETED:TURN:<deg>`<br>`ALERT:SAFETY_HALT`<br>`ALERT:EMERGENCY_STOP` |
| `STOP` | *None* | Immediate emergency stop. Halts active motion mid-step. | `ALERT:EMERGENCY_STOP` |
| `F` | *None* | Continuous manual forward step. | *None* |
| `B` | *None* | Continuous manual backward step. | *None* |
| `L` | *None* | Continuous manual turn left step. | *None* |
| `R` | *None* | Continuous manual turn right step. | *None* |
| `S` | *None* | Stop continuous manual motion. | *None* |
| `TOGGLE:<sensor>` | `sensor`: `ir` or `us` | Enables or disables IR or Ultrasonic sensor safety checks. | `ACK:TOGGLE` |
| `CFG:<param>:<val>` | `param`: `delay` or `obs`<br>`val`: float value | Adjusts speed delay (`delay` in µs) or obstacle distance threshold (`obs` in cm). | `ACK:CFG` |

#### UDP Example Commands (Raw Strings)
- Drive forward 25.4 cm: `MOVE:25.4` or `MOVE:25.4:1`
- Drive backward 10 cm: `MOVE:10:-1`
- Turn 90 degrees right: `TURN:90`
- Turn 45 degrees left: `TURN:-45`
- Emergency stop: `STOP`
- Disable IR sensors: `TOGGLE:ir`
- Set speed delay to 800 µs: `CFG:delay:800`
- Set obstacle threshold to 15 cm: `CFG:obs:15`

---

### 2. HTTP REST API (Port 80)

All HTTP endpoints run on port 80 using `ESPAsyncWebServer`.

#### `GET /manual?cmd=<CMD>`
Controls continuous manual stepping.
- **Query Parameter**: `cmd` (`F`, `B`, `L`, `R`, or `S`).
- **Response**: `200 OK`

#### `GET /move_cm?cm=<DIST>&dir=<DIR>`
Commands autonomous distance drive.
- **Query Parameters**:
  - `cm` (float, required): Distance in centimeters.
  - `dir` (float, optional): `1.0` (forward) or `-1.0` (backward).
- **Response**: `200 OK` (or `400 Bad Request` if `cm` is missing).

#### `GET /turn?deg=<DEGREES>`
Commands autonomous in-place rotation.
- **Query Parameter**: `deg` (float, required): Turn angle (+ for right, - for left).
- **Response**: `200 OK`

#### `GET /stop`
Triggers an immediate emergency stop.
- **Response**: `200 OK`

#### `GET /config?param=<PARAM>&val=<VALUE>`
Adjusts runtime settings.
- **Query Parameters**:
  - `param`: `delay` or `obs`
  - `val`: Numeric value.
- **Response**: `200 OK`

#### `GET /toggle?type=<TYPE>`
Toggles sensor bypass.
- **Query Parameter**: `type`: `ir` or `us`
- **Response**: `200 OK` with body `"1"` (enabled) or `"0"` (disabled).

---

### 3. Server-Sent Events (SSE) Stream (`/events`)

Connect to `http://<ROBOT_NAME>.local/events` to stream real-time telemetry into web pages or client applications.

#### Event: `sensor_data`
Pushed automatically every 100 ms.
- **Payload Format (JSON)**:
  ```json
  {
    "l": 1,
    "r": 1,
    "d": 42.5
  }
  ```
  - `l`: Left IR sensor (`1` = floor detected / OK, `0` = edge / cliff)
  - `r`: Right IR sensor (`1` = floor detected / OK, `0` = edge / cliff)
  - `d`: Current ultrasonic distance in centimeters.

#### Event: `log`
Pushed whenever an event, motion, safety alert, or configuration change occurs.
- **Payload Format**: Plain text message (e.g., `Moving 10.00 cm (2568 steps)` or `SAFETY HALT: Obstacle Detected!`).

---

### 4. Web Dashboard UI (`/Fawbot`)

Navigate to `http://<ROBOT_NAME>.local/Fawbot` (e.g. `http://fawbot_23.local/Fawbot`) on any phone, tablet, or desktop browser on the same Wi-Fi network.

The web UI includes:
- **Sensor Telemetry Bar**: Live indicator dots for Left & Right IR and real-time ultrasonic distance.
- **Virtual D-Pad**: Touch/click controls for manual drive (`▲`, `◀`, `▶`, `▼`, and `■ STOP`).
- **Autonomous Motion Controls**: Input fields for distance (cm) and angle (degrees) with Forward, Backward, and Turn buttons.
- **Configuration Panel**: On-the-fly step delay adjustment, obstacle distance threshold, and sensor bypass toggles.
- **Terminal Console**: Live scrolling log stream showing system status and execution confirmations.

---

## Python Client Example

Below is a complete, lightweight Python controller class demonstrating how to connect, command, and handle feedback over UDP:

```python
import socket
import time

class FawBotUDPClient:
    def __init__(self, hostname="fawbot_23.local", port=8888, timeout=10.0):
        self.hostname = hostname
        self.port = port
        self.timeout = timeout
        
        # Resolve mDNS hostname to IP
        self.ip = socket.gethostbyname(hostname)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)
        print(f"[FawBot] Connected to {hostname} ({self.ip}:{port})")

    def send_command(self, cmd: str, wait_for_ack=True) -> str:
        """Sends a UDP command and optionally waits for robot response."""
        self.sock.sendto(cmd.encode("utf-8"), (self.ip, self.port))
        
        if not wait_for_ack:
            return ""
            
        try:
            data, _ = self.sock.recvfrom(256)
            reply = data.decode("utf-8").strip()
            return reply
        except socket.timeout:
            return "ERROR:TIMEOUT"

    def move_cm(self, cm: float, direction: float = 1.0) -> str:
        """Move forward or backward by cm."""
        cmd = f"MOVE:{cm}:{direction}" if direction != 1.0 else f"MOVE:{cm}"
        return self.send_command(cmd)

    def turn(self, degrees: float) -> str:
        """Turn in place (+deg = CW / Right, -deg = CCW / Left)."""
        return self.send_command(f"TURN:{degrees}")

    def emergency_stop(self) -> str:
        """Sends immediate emergency stop."""
        return self.send_command("STOP")

    def toggle_sensor(self, sensor: str) -> str:
        """sensor: 'ir' or 'us'"""
        return self.send_command(f"TOGGLE:{sensor}")

    def set_speed_delay(self, delay_us: int) -> str:
        """Set step delay in microseconds (650 = fast, 3000 = slow)."""
        return self.send_command(f"CFG:delay:{delay_us}")

    def set_obstacle_threshold(self, threshold_cm: int) -> str:
        """Set obstacle stop distance in cm."""
        return self.send_command(f"CFG:obs:{threshold_cm}")

# Example Usage:
if __name__ == "__main__":
    bot = FawBotUDPClient("fawbot_23.local")

    # Set speed delay to 800us
    bot.set_speed_delay(800)

    # Drive forward 15 cm
    print("Moving 15 cm forward...")
    res = bot.move_cm(15)
    print("Result:", res)

    # Turn 90 degrees clockwise
    print("Turning 90 degrees...")
    res = bot.turn(90)
    print("Result:", res)

    # Emergency stop
    bot.emergency_stop()
```


---

## Overhead Camera ArUco Navigation (`robot_control.py`)

The companion Python visual servoing controller tracks FawBot (ID 23) from an overhead webcam mounted on a tripod and drives the robot to wherever you click on the camera feed.

### How It Works
1. **ArUco Pose Estimation**: Finds marker **ID 23**, calculates center $(x, y)$, and computes front heading angle from corner coordinates.
2. **Interactive Goal Setting**: Click anywhere in the video feed to set an immediate destination point (or `Shift + Click` to queue a sequence of waypoints).
3. **Closed-Loop Visual Servoing**:
   - Calculates target heading angle $\theta_{target} = \text{atan2}(\Delta y, \Delta x)$ and heading error $\Delta\theta$.
   - Aligns heading by pulsing rotational steps (`R` or `L`) over UDP with hysteresis deadbands ($22^\circ$ enter / $12^\circ$ exit) to prevent hunting.
   - Once aligned, drives forward (`F`) while continuously making minor heading corrections.
   - When within the arrival radius ($35\text{ px}$), sends `S` (Stop) and marks the goal reached.
4. **Auto-Scan ArUco Dictionaries**: Supports `DICT_ARUCO_ORIGINAL`, `DICT_4X4_50`, `DICT_5X5_50`, and `DICT_6X6_50`. If the default dictionary doesn't find ID 23, it automatically scans others. Press `D` to cycle manually.

### Running the Controller
Ensure your Python environment has `opencv-python` and `numpy` installed (e.g. `conda activate robotcar`):
```bash
python robot_control.py
```

Options:
```bash
python robot_control.py --camera 0 --id 23 --host fawbot_23.local --port 8888
```

### Controls in OpenCV Window:
- **Left Click**: Set destination point and start navigation.
- **Shift + Left Click**: Add waypoint to route queue.
- **Right Click** or **'C'**: Clear target / stop navigation.
- **'S'** or **SPACE**: Emergency STOP.
- **'D'**: Cycle ArUco dictionary.

---

## Multi-Robot Fleet Routing & Navigation (`multi_robot_fleet_control.py`)

Controls a fleet of FawBots (**IDs: 22, 23, 31**) concurrently under an overhead camera. You can click multiple destination points on the camera view, and pressing **SPACEBAR** automatically executes an optimal routing algorithm to assign each robot to its nearest route.

### Features
- **Concurrent Tracking**: Automatically identifies and tracks ArUco markers for Robot 22 (Cyan), Robot 23 (Green), and Robot 31 (Magenta).
- **Optimal Route Assignment**:
  - If number of points $\le$ number of robots: Computes the global minimum-distance 1-to-1 matching via the Hungarian algorithm (`linear_sum_assignment`).
  - If number of points $>$ number of robots: Groups points into spatial clusters nearest to each robot and sequences each cluster into an optimal, non-overlapping TSP (Traveling Salesperson) tour using nearest-neighbor optimization.
- **Parallel Closed-Loop Servoing**: All robots rotate and drive toward their assigned sequence of targets independently in real-time.
- **Inter-Robot Collision Avoidance**: Continuously monitors Euclidean distances between all active robots. If two robots get within $75\text{ px}$, the robot with lower priority or further distance yields (`send("S")`) until the other robot clears the area.
- **Dynamic Connection Management**: Runs background mDNS checks so robots can be powered on or rebooted at any time without restarting the script.

### Running the Multi-Robot Controller:
```bash
conda activate robotcar
python multi_robot_fleet_control.py
```

Optional arguments:
```bash
python multi_robot_fleet_control.py --camera 0 --ip22 192.168.1.12 --ip23 192.168.1.13 --ip31 192.168.1.11
```

### Controls:
- **Left Click**: Add goal points anywhere on the camera view.
- **Right Click**: Undo last placed point.
- **SPACEBAR**: Optimize nearest routes & **START** fleet movement.
- **'S'**: Emergency STOP all robots.
- **'C'**: Clear all points and halt all robots.
- **'D'**: Cycle ArUco dictionary.
- **'Q'** or **ESC**: Stop all robots and quit.

---

## Building, Flashing, and Setup

### Prerequisites
- [PlatformIO Core (CLI)](https://docs.platformio.org/page/core/index.html) or PlatformIO IDE extension for VSCode.
- ESP32 Development Board connected via USB-C or Micro-USB.

### Wi-Fi Configuration
Before flashing, verify or update the Wi-Fi credentials in [`src/web_portal.cpp`](src/web_portal.cpp):
```cpp
WiFi.mode(WIFI_STA);
WiFi.begin("YOUR_SSID", "YOUR_PASSWORD");
```

### Build and Upload Commands

From the `embedded_code` directory:

1. **Build a specific robot environment:**
   ```bash
   # For Fawbot 23:
   pio run -e fawbot23

   # For Fawbot 22:
   pio run -e fawbot22

   # For Fawbot 31:
   pio run -e fawbot31
   ```

2. **Upload firmware to ESP32:**
   ```bash
   pio run -e fawbot23 -t upload
   ```

3. **Open Serial Monitor (115200 baud):**
   ```bash
   pio device monitor -b 115200
   ```

Upon boot, the serial terminal will print:
```
==========================================
 Starting Fawbot_23
==========================================
 Motor Left  (IN1-IN4) : 13, 12, 14, 27
 Motor Right (IN5-IN8) : 32, 33, 25, 26
 IR Sensors (L / R)    : 34 / 35
 Ultrasonic (TRIG / ECHO): 5 / 18
==========================================

Connecting to WiFi 'saraths_Lab'.......
Connected! IP Address: 192.168.1.13
mDNS responder started!
Access at: http://Fawbot_23.local/Fawbot
UDP Listener active on port 8888
```

---

## Troubleshooting & Calibration

| Symptom | Cause | Solution |
| :--- | :--- | :--- |
| **Robot steps in reverse or turns the wrong way** | Stepper phase wires or motor sides swapped. | In `platformio.ini`, swap the motor pin assignments (e.g. swap `IN5`..`IN8` sequence or adjust the build flags). |
| **Robot stops immediately when moving forward** | IR cliff sensor triggered (no floor reflection) or ultrasonic object `< 20 cm`. | Check surface reflectivity. Test bypass with `TOGGLE:ir` or `TOGGLE:us`, or raise obstacle threshold with `CFG:obs:30`. |
| **Cannot resolve `<ROBOT_NAME>.local`** | mDNS resolution not supported or router isolates multicast traffic. | Ensure Bonjour is running (macOS / Linux Avahi). Alternatively, resolve directly via the static/assigned IP address. |
| **Motors vibrating without turning** | Speed delay too low (`speedDelayUs < 600 µs`) causing torque stall. | Increase delay using `CFG:delay:1000` or adjust `speedPercent`. 28BYJ-48 steppers stall if stepped faster than ~650 µs per step. |
| **Distance traveled is inaccurate** | Wheel diameter or surface traction differs from calibration. | Update `STEPS_PER_CM` in `include/config.h` ($Steps/cm = 4096 / (\pi \times \text{Wheel Diameter})$). Default is `256.81`. |
