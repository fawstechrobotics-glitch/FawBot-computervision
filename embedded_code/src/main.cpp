#include "config.h"

// Initialize shared variables
int speedPercent = 100;
int speedDelayUs = 1000;
int obstacleThreshold = 20;

volatile long posL = 0;
volatile long posR = 0;

volatile bool shouldMoveCm = false;
volatile bool shouldTurn = false;
volatile bool emergencyStop = false;
volatile bool safetyHalt = false;
volatile bool isManualMoving = false;

volatile float targetDistanceCm = 0;
volatile float targetDegrees = 0;
volatile float moveDirection = 1.0;

bool irLeftStatus = true;
bool irRightStatus = true;
float currentDistance = 100.0;

bool irEnabled = true;
bool ultrasonicEnabled = true;
volatile bool lidarScanRequested = false;
volatile bool lidarScanning = false;
volatile bool lidarContinuous = false;
volatile float lidarSweepDegrees = 360.0;
volatile float lidarStepAngleDegrees = LIDAR_STEP_ANGLE_DEG;
volatile float lidarStepDistanceCm = 0.0;
volatile float lidarPoseX = 0.0;
volatile float lidarPoseY = 0.0;
volatile float lidarPoseHeading = 0.0;

AsyncWebServer server(80);
AsyncEventSource events("/events");

unsigned long lastSensorTime = 0;

void setup() {
    Serial.begin(115200);
    Serial.printf("\n==========================================");
    Serial.printf("\n Starting %s", ROBOT_NAME);
    Serial.printf("\n==========================================");
    Serial.printf("\n Motor Left  (IN1-IN4) : %d, %d, %d, %d", IN1, IN2, IN3, IN4);
    Serial.printf("\n Motor Right (IN5-IN8) : %d, %d, %d, %d", IN5, IN6, IN7, IN8);
    Serial.printf("\n IR Sensors (L / R)    : %d / %d", IR_LEFT, IR_RIGHT);
    Serial.printf("\n Ultrasonic (TRIG / ECHO): %d / %d", TRIG_PIN, ECHO_PIN);
    Serial.printf("\n==========================================\n\n");
    initMotors();
    initSensors();
    startWebPortal();
}

void loop() {
    // Process fast non-blocking UDP packets from Python
    handleUDP();

    // Sensor update loop (100ms)
    if (millis() - lastSensorTime > 100) {
        lastSensorTime = millis();
        updateSensors();
        
        String json = "{\"l\":" + String(irLeftStatus) + 
                      ",\"r\":" + String(irRightStatus) + 
                      ",\"d\":" + String(currentDistance, 1) + "}";
        events.send(json.c_str(), "sensor_data", millis());
    }

    // Safety checks
    if ((irEnabled && (!irLeftStatus || !irRightStatus)) || 
        (ultrasonicEnabled && currentDistance < obstacleThreshold)) {
        if (!safetyHalt) {
            safetyHalt = true;
            stopMotors();
            sendLog("SAFETY HALT: Obstacle Detected!");
        }
    } else {
        safetyHalt = false;
    }

    if (lidarScanRequested && !lidarScanning) {
        processLidarScan();
    }

    // Motion execution
    if (!lidarScanning && !safetyHalt && !emergencyStop) {
        if (shouldMoveCm) {
            shouldMoveCm = false;
            moveRobotCm(targetDistanceCm, moveDirection);
        } else if (shouldTurn) {
            shouldTurn = false;
            turnRobot(targetDegrees);
        } else if (isManualMoving) {
            if (moveDirection == -1.0)      moveManualStep(-1, -1); // Forward
            else if (moveDirection == 1.0)  moveManualStep(1, 1);   // Backward
            else if (moveDirection == -2.0) moveManualStep(1, -1);  // Left
            else if (moveDirection == 2.0)  moveManualStep(-1, 1);  // Right
        }
    }

    if (emergencyStop) {
        stopMotors();
        emergencyStop = false;
        isManualMoving = false;
        sendLog("EMERGENCY STOP EXECUTED");
    }
}

void sendLog(String msg) {
    events.send(msg.c_str(), "log", millis());
    Serial.println(msg);
}