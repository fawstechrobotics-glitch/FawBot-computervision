#include "config.h"

VL53L0X lidar;
bool lidarReady = false;

static void sendLidarPacket(uint16_t angleDegrees, uint16_t distanceMm, const char* dirStr = "CENTER") {
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
    String fullPacket = String(packetHex) + ":" + String(dirStr);
    events.send(fullPacket.c_str(), "lidar_packet", millis());
    sendUDPFeedback("LIDAR_PACKET:" + fullPacket);
}

void initSensors() {
    pinMode(IR_LEFT, INPUT);
    pinMode(IR_RIGHT, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    Wire.begin(LIDAR_SDA, LIDAR_SCL);
    lidar.setTimeout(200);
    lidarReady = lidar.init();
    if (lidarReady) {
        lidar.setMeasurementTimingBudget(33000);
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

void requestLidarScan(float sweepDegrees) {
    if (lidarReady && !lidarScanning) {
        if (sweepDegrees > 0.0f && sweepDegrees <= 360.0f) {
            lidarSweepDegrees = sweepDegrees;
        }
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

static uint16_t readLidarDistanceMm() {
    if (!lidarReady) {
        return 0;
    }
    uint16_t distanceMm = lidar.readRangeContinuousMillimeters();
    if (lidar.timeoutOccurred() || distanceMm == 65535) {
        // Continuous mode failed or timed out.
        // Clear latched interrupt register (0x0B = SYSTEM_INTERRUPT_CLEAR)
        lidar.writeReg(0x0B, 0x01);
        // Attempt single-shot measurement
        distanceMm = lidar.readRangeSingleMillimeters();
        if (lidar.timeoutOccurred() || distanceMm >= 8190) {
            distanceMm = 0;
        }
        // Re-arm continuous mode for next cycle
        lidar.writeReg(0x0B, 0x01);
        lidar.startContinuous();
    } else if (distanceMm >= 8190) {
        // ST VL53L0X returns >= 8190 when out-of-range or signal fail
        distanceMm = 0;
    }
    return distanceMm;
}

static bool readAndSendLidar(float relAngleDeg, const char* dirStr, uint16_t packetAngle) {
    uint16_t distanceMm = readLidarDistanceMm();
    float distCm = distanceMm / 10.0f;
    String ptMsg = "LIDAR_POINT:" + String(dirStr) + ":" + String(relAngleDeg, 1) + ":" + String(distCm, 1);
    sendUDPFeedback(ptMsg);
    events.send(ptMsg.c_str(), "lidar_point", millis());
    sendLidarPacket(packetAngle, distanceMm, dirStr);
    return !emergencyStop;
}

static bool executeLidarSweep(float sweepDegrees) {
    const float lidarStep = (lidarStepAngleDegrees > 0.0f) ? lidarStepAngleDegrees : LIDAR_STEP_ANGLE_DEG;

    sendUDPFeedback("SWEEP_START:" + String(sweepDegrees, 1));
    events.send(String(sweepDegrees, 1).c_str(), "sweep_start", millis());

    if (sweepDegrees >= 350.0f) {
        // Full 360-degree rotation: turn Anti-Clockwise (CCW) in a full circle
        sendUDPFeedback("SWEEP_DIR:CCW");
        events.send("CCW", "sweep_dir", millis());
        sendUDPFeedback("ROTATION_DIR:CCW");
        events.send("CCW", "rotation_dir", millis());
        const uint16_t totalSteps = (uint16_t)(360.0f / lidarStep);
        for (uint16_t sample = 0; sample < totalSteps; sample++) {
            turnRobot(lidarStep); // Anti-Clockwise / Left
            float currentRelAngle = (float)(sample + 1) * lidarStep;
            if (!readAndSendLidar(currentRelAngle, "CCW", (uint16_t)currentRelAngle)) {
                lastLidarSweepCompletedTime = millis();
                sendUDPFeedback("SWEEP_DIR:DONE");
                events.send("DONE", "sweep_dir", millis());
                sendUDPFeedback("ROTATION_DIR:CENTER");
                events.send("CENTER", "rotation_dir", millis());
                return false;
            }
        }
        lastLidarSweepCompletedTime = millis();
        sendUDPFeedback("SWEEP_DIR:DONE");
        events.send("DONE", "sweep_dir", millis());
        sendUDPFeedback("ROTATION_DIR:CENTER");
        events.send("CENTER", "rotation_dir", millis());
        return !emergencyStop;
    }

    const float halfSweep = sweepDegrees / 2.0f;
    const uint16_t sampleCount = (uint16_t)(halfSweep / lidarStep);
    const float actualTurn = sampleCount * lidarStep;
    const uint16_t centerPacketAngle = (uint16_t)(halfSweep);

    // Phase 1: Clockwise (CW) rotation to the right
    sendUDPFeedback("SWEEP_DIR:CW");
    events.send("CW", "sweep_dir", millis());
    sendUDPFeedback("ROTATION_DIR:CW");
    events.send("CW", "rotation_dir", millis());
    for (uint16_t sample = 0; sample < sampleCount; sample++) {
        turnRobot(-lidarStep); // Clockwise / Right
        float currentRelAngle = -(float)(sample + 1) * lidarStep;
        if (!readAndSendLidar(currentRelAngle, "CW", (uint16_t)(sample * lidarStep))) {
            turnRobot((sample + 1) * lidarStep);
            lastLidarSweepCompletedTime = millis();
            sendUDPFeedback("SWEEP_DIR:DONE");
            events.send("DONE", "sweep_dir", millis());
            sendUDPFeedback("ROTATION_DIR:CENTER");
            events.send("CENTER", "rotation_dir", millis());
            return false;
        }
    }

    // Return to Center (0 deg relative) by turning Anti-Clockwise (CCW)
    sendUDPFeedback("ROTATION_DIR:CCW");
    events.send("CCW", "rotation_dir", millis());
    turnRobot(actualTurn);

    sendUDPFeedback("SWEEP_DIR:CENTER");
    events.send("CENTER", "sweep_dir", millis());
    sendUDPFeedback("ROTATION_DIR:CENTER");
    events.send("CENTER", "rotation_dir", millis());
    if (!readAndSendLidar(0.0f, "CENTER", centerPacketAngle)) {
        lastLidarSweepCompletedTime = millis();
        sendUDPFeedback("SWEEP_DIR:DONE");
        events.send("DONE", "sweep_dir", millis());
        sendUDPFeedback("ROTATION_DIR:CENTER");
        events.send("CENTER", "rotation_dir", millis());
        return false;
    }

    // Phase 2: Anti-Clockwise (CCW) rotation to the left
    sendUDPFeedback("SWEEP_DIR:CCW");
    events.send("CCW", "sweep_dir", millis());
    sendUDPFeedback("ROTATION_DIR:CCW");
    events.send("CCW", "rotation_dir", millis());
    for (uint16_t sample = 0; sample < sampleCount; sample++) {
        turnRobot(lidarStep); // Anti-Clockwise / Left
        float currentRelAngle = (float)(sample + 1) * lidarStep;
        if (!readAndSendLidar(currentRelAngle, "CCW", centerPacketAngle + (uint16_t)((sample + 1) * lidarStep))) {
            turnRobot(-(sample + 1) * lidarStep);
            lastLidarSweepCompletedTime = millis();
            sendUDPFeedback("SWEEP_DIR:DONE");
            events.send("DONE", "sweep_dir", millis());
            sendUDPFeedback("ROTATION_DIR:CENTER");
            events.send("CENTER", "rotation_dir", millis());
            return false;
        }
    }

    // Return to Center (0 deg relative) by turning Clockwise (CW)
    sendUDPFeedback("ROTATION_DIR:CW");
    events.send("CW", "rotation_dir", millis());
    turnRobot(-actualTurn);

    lastLidarSweepCompletedTime = millis();
    sendUDPFeedback("SWEEP_DIR:DONE");
    events.send("DONE", "sweep_dir", millis());
    sendUDPFeedback("ROTATION_DIR:CENTER");
    events.send("CENTER", "rotation_dir", millis());
    return !emergencyStop;
}

static bool processContinuousLidarCycle() {
    if (!executeLidarSweep(lidarSweepDegrees)) {
        return false;
    }

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

    // Ensure continuous mode is fresh and armed before scan
    lidar.writeReg(0x0B, 0x01);
    lidar.startContinuous();

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

    sendLog("Lidar sweep scan started: " + String(lidarSweepDegrees, 1) + " degrees");
    executeLidarSweep(lidarSweepDegrees);

    stopMotors();
    lidarScanning = false;
    if (emergencyStop) {
        emergencyStop = false;
        sendLog("Lidar scan stopped");
    } else {
        sendLog("Lidar scan complete");
    }
}