#include "config.h"

const uint8_t stepSequence[8] = {
    0b1000, 0b1100, 0b0100, 0b0110,
    0b0010, 0b0011, 0b0001, 0b1001
};

int stepIndex1 = 0;
int stepIndex2 = 0;

void updateSpeedDelay() {
    speedPercent = constrain(speedPercent, 1, 100);
    speedDelayUs = map(speedPercent, 1, 100, 3000, 650);
}

void stopMotors() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    digitalWrite(IN5, LOW); digitalWrite(IN6, LOW);
    digitalWrite(IN7, LOW); digitalWrite(IN8, LOW);
}

void initMotors() {
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT);
    pinMode(IN7, OUTPUT); pinMode(IN8, OUTPUT);
    stopMotors();
    updateSpeedDelay();
}

static inline void writeMotorPins(uint8_t m1Pattern, uint8_t m2Pattern) {
    digitalWrite(IN1, (m1Pattern >> 3) & 0x01);
    digitalWrite(IN2, (m1Pattern >> 2) & 0x01);
    digitalWrite(IN3, (m1Pattern >> 1) & 0x01);
    digitalWrite(IN4, (m1Pattern >> 0) & 0x01);

    digitalWrite(IN5, (m2Pattern >> 3) & 0x01);
    digitalWrite(IN6, (m2Pattern >> 2) & 0x01);
    digitalWrite(IN7, (m2Pattern >> 1) & 0x01);
    digitalWrite(IN8, (m2Pattern >> 0) & 0x01);
}

void stepMotors(int steps, int dirL, int dirR) {
    safetyHalt = false;

    for (int i = 0; i < steps; i++) {
        serviceEmergencyStop();

        if (i % 20 == 0) {
            updateSensors();
            if (dirL > 0 && dirR > 0) { 
                if (!irLeftStatus || !irRightStatus || currentDistance < obstacleThreshold) {
                    safetyHalt = true;
                    stopMotors();
                    sendLog("OBSTACLE/EDGE DETECTED! Stopping movement.");
                    break;
                }
            }
        }

        if (emergencyStop || safetyHalt) {
            stopMotors();
            break;
        }

        stepIndex1 = (stepIndex1 + dirL + 8) % 8;
        stepIndex2 = (stepIndex2 + dirR + 8) % 8;

        writeMotorPins(stepSequence[stepIndex1], stepSequence[stepIndex2]);
        
        posL += dirL;
        posR += dirR;

        delayMicroseconds(speedDelayUs);
    }
    stopMotors();
}

void moveRobotCm(float distanceCm, float direction) {
    long targetSteps = abs((long)(distanceCm * STEPS_PER_CM));
    
    int dirL = (direction > 0) ? -1 : 1;
    int dirR = (direction > 0) ? -1 : 1;

    sendLog("Moving " + String(distanceCm) + " cm (" + String(targetSteps) + " steps)");
    stepMotors(targetSteps, dirL, dirR);
}

void turnRobot(float degrees) {
    float arcLengthCm = (3.14159 * WHEEL_BASE_CM) * (abs(degrees) / 360.0);
    long targetSteps = abs((long)(arcLengthCm * STEPS_PER_CM));

    int dirL = (degrees > 0) ? 1 : -1;
    int dirR = (degrees > 0) ? -1 : 1;

    sendLog("Turning " + String(degrees) + " deg (" + String(targetSteps) + " steps)");
    stepMotors(targetSteps, dirL, dirR);
}

void moveManualStep(int dirL, int dirR) {
    stepIndex1 = (stepIndex1 + dirL + 8) % 8;
    stepIndex2 = (stepIndex2 + dirR + 8) % 8;
    writeMotorPins(stepSequence[stepIndex1], stepSequence[stepIndex2]);
    
    posL += dirL;
    posR += dirR;
    delayMicroseconds(speedDelayUs);
}

