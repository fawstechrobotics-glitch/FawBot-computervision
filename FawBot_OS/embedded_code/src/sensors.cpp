#include "config.h"

VL53L0X lidar;
bool lidarReady = false;

static void sendLidarPacket(uint16_t angleDegrees, uint16_t distanceMm) {
    uint16_t angleQ6 = (uint16_t)(angleDegrees * 64U);
    uint16_t distanceQ2 = (uint16_t)min((uint32_t)distanceMm * 4U, 65535U);
    uint8_t packet[5] = {
        (uint8_t)((15U << 2) | 0x01),
        (uint8_t)(((angleQ6 << 1) & 0xFEU) | 0x01U),
        (uint8_t)(angleQ6 >> 7),
        (uint8_t)(distanceQ2 & 0xFFU),
        (uint8_t)(distanceQ2 >> 8)
    };

    char packetHex[11];
    snprintf(packetHex, sizeof(packetHex), "%02X%02X%02X%02X%02X",
             packet[0], packet[1], packet[2], packet[3], packet[4]);
    events.send(packetHex, "lidar_packet", millis());
    sendUDPFeedback("LIDAR_PACKET:" + String(packetHex));
}

void initSensors() {
    pinMode(IR_LEFT, INPUT);
    pinMode(IR_RIGHT, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    Wire.begin(LIDAR_SDA, LIDAR_SCL);
    lidar.setTimeout(100);
    lidarReady = lidar.init();
    if (lidarReady) {
        lidar.startContinuous();
        sendLog("VL53L0X lidar ready");
    } else {
        sendLog("VL53L0X lidar not found");
    }
}

void updateSensors() {
    // Process IR sensors (Active LOW logic: LOW = Floor detected)
    if (irEnabled) {
        irLeftStatus = (digitalRead(IR_LEFT) == LOW);
        irRightStatus = (digitalRead(IR_RIGHT) == LOW);
    } else {
        irLeftStatus = true;
        irRightStatus = true;
    }

    // Process Ultrasonic Ping
    if (ultrasonicEnabled) {
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);
        
        long duration = pulseIn(ECHO_PIN, HIGH, 15000); // 15ms timeout (~2.5m range)
        if (duration > 0) {
            currentDistance = duration * 0.034 / 2.0;
        } else {
            currentDistance = 400.0; 
        }
    } else {
        currentDistance = 100.0;
    }
}

void requestLidarScan() {
    if (lidarReady && !lidarScanning) {
        lidarContinuous = false;
        lidarScanRequested = true;
    }
}

void requestContinuousLidarScan(float sweepDegrees, float stepDistanceCm) {
    if (lidarReady && !lidarScanning && sweepDegrees > 0.0 &&
        sweepDegrees <= 360.0 && stepDistanceCm > 0.0) {
        lidarSweepDegrees = sweepDegrees;
        lidarStepDistanceCm = stepDistanceCm;
        lidarContinuous = true;
        lidarScanRequested = true;
    }
}

void stopLidarScan() {
    lidarScanRequested = false;
    lidarContinuous = false;
    emergencyStop = true;
}

static bool readAndSendLidar(uint16_t packetAngle) {
    uint16_t distanceMm = lidar.readRangeContinuousMillimeters();
    if (lidar.timeoutOccurred()) {
        distanceMm = 0;
    }
    sendLidarPacket(packetAngle, distanceMm);
    return !emergencyStop;
}

static bool processContinuousLidarCycle() {
    const float lidarStep = (lidarStepAngleDegrees > 0.0f) ? lidarStepAngleDegrees : LIDAR_STEP_ANGLE_DEG;
    const float halfSweep = lidarSweepDegrees / 2.0f;
    const uint16_t sampleCount = (uint16_t)(halfSweep / lidarStep);
    const uint16_t centerPacketAngle = (uint16_t)(halfSweep);

    for (uint16_t sample = 0; sample < sampleCount; sample++) {
        turnRobot(-lidarStep);
        if (!readAndSendLidar((uint16_t)(sample * lidarStep))) {
            return false;
        }
    }

    turnRobot(halfSweep);
    if (!readAndSendLidar(centerPacketAngle)) {
        return false;
    }

    for (uint16_t sample = 0; sample < sampleCount; sample++) {
        turnRobot(lidarStep);
        if (!readAndSendLidar(centerPacketAngle + (uint16_t)((sample + 1) * lidarStep))) {
            return false;
        }
    }

    turnRobot(-halfSweep);
    moveRobotCm(lidarStepDistanceCm, 1.0);
    if (emergencyStop || safetyHalt) {
        return false;
    }

    const float headingRadians = lidarPoseHeading * 0.01745329252f;
    lidarPoseX += lidarStepDistanceCm * cos(headingRadians);
    lidarPoseY += lidarStepDistanceCm * sin(headingRadians);
    String reached = "REACHED:LIDAR:" + String(lidarPoseX, 2) + ":" +
                     String(lidarPoseY, 2) + ":" + String(lidarPoseHeading, 2);
    sendUDPFeedback(reached);
    events.send(reached.c_str(), "robot_pose", millis());
    return true;
}

void processLidarScan() {
    if (!lidarReady) {
        sendLog("Lidar scan rejected: VL53L0X unavailable");
        lidarScanRequested = false;
        return;
    }

    lidarScanRequested = false;
    lidarScanning = true;
    emergencyStop = false;
    isManualMoving = false;
    shouldMoveCm = false;
    shouldTurn = false;

    if (lidarContinuous) {
        sendLog("Continuous lidar started: " + String(lidarSweepDegrees, 1) +
                " deg, " + String(lidarStepDistanceCm, 1) + " cm steps");
        while (!emergencyStop && processContinuousLidarCycle()) {
            serviceEmergencyStop();
        }
        stopMotors();
        lidarScanning = false;
        lidarContinuous = false;
        if (emergencyStop) {
            emergencyStop = false;
            sendLog("Continuous lidar stopped");
        }
        return;
    }

    sendLog("Lidar scan started: 360 degrees");

    for (uint16_t angle = 0; angle < 360 && !emergencyStop; angle += 5) {
        turnRobot(5.0);
        uint16_t distanceMm = lidar.readRangeContinuousMillimeters();
        if (lidar.timeoutOccurred()) {
            distanceMm = 0;
        }
        sendLidarPacket(angle, distanceMm);
    }

    stopMotors();
    lidarScanning = false;
    if (emergencyStop) {
        emergencyStop = false;
        sendLog("Lidar scan stopped");
    } else {
        sendLog("Lidar scan complete");
    }
}