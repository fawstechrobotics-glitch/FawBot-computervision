#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <VL53L0X.h>

// Set per robot, or override with PlatformIO build_flags (for example Fawbot_31).
#ifndef ROBOT_NAME
#define ROBOT_NAME "Fawbot_23"
#endif

// --- MOTOR PINS ---
// Motor 1 (Left)
#ifndef IN1
#define IN1 13
#endif

#ifndef IN2
#define IN2 12
#endif

#ifndef IN3
#define IN3 14
#endif

#ifndef IN4
#define IN4 27
#endif

// Motor 2 (Right)
#ifndef IN5
#define IN5 26
#endif

#ifndef IN6
#define IN6 25
#endif

#ifndef IN7
#define IN7 33
#endif

#ifndef IN8
#define IN8 32
#endif

// --- SENSOR PINS ---
#ifndef IR_LEFT
#define IR_LEFT 34
#endif

#ifndef IR_RIGHT
#define IR_RIGHT 35
#endif

#ifndef TRIG_PIN
#define TRIG_PIN 5
#endif

#ifndef ECHO_PIN
#define ECHO_PIN 18
#endif

#ifndef LIDAR_SDA
#define LIDAR_SDA 21
#endif

#ifndef LIDAR_SCL
#define LIDAR_SCL 22
#endif

#ifndef LIDAR_STEP_ANGLE_DEG
#define LIDAR_STEP_ANGLE_DEG 1.0f
#endif

// --- MECHANICAL CONFIGURATION ---
#define STEPS_PER_REV 4096.0
#define WHEEL_DIAMETER_CM 5
#define WHEEL_BASE_CM     10.2 
#define WHEEL_CIRCUMFERENCE (3.14159 * WHEEL_DIAMETER_CM)

// --- HARDCODED CALIBRATED STEPS PER CM ---
#define STEPS_PER_CM 256.81f

// --- SHARED CONTROL VARIABLES ---
extern int speedPercent;       
extern int speedDelayUs;       
extern int obstacleThreshold; 

extern volatile long posL;     
extern volatile long posR;

extern volatile bool shouldMoveCm;
extern volatile bool shouldTurn;
extern volatile bool emergencyStop;
extern volatile bool safetyHalt;
extern volatile bool isManualMoving;

extern volatile float targetDistanceCm;
extern volatile float targetDegrees;
extern volatile float moveDirection;

extern bool irLeftStatus;
extern bool irRightStatus;
extern float currentDistance;

extern bool irEnabled;
extern bool ultrasonicEnabled;
extern volatile bool lidarScanRequested;
extern volatile bool lidarScanning;
extern volatile bool lidarContinuous;
extern volatile float lidarSweepDegrees;
extern volatile float lidarStepAngleDegrees;
extern volatile float lidarStepDistanceCm;
extern volatile float lidarPoseX;
extern volatile float lidarPoseY;
extern volatile float lidarPoseHeading;

extern AsyncWebServer server;
extern AsyncEventSource events;
extern WiFiUDP udp;

// --- FUNCTION PROTOTYPES ---
void initMotors();
void stopMotors();
void updateSpeedDelay();
void stepMotors(int steps, int dirL, int dirR);
void moveRobotCm(float distanceCm, float direction);
void turnRobot(float degrees);
void moveManualStep(int dirL, int dirR);

void initSensors();
void updateSensors();
void requestLidarScan();
void requestContinuousLidarScan(float sweepDegrees, float stepDistanceCm);
void stopLidarScan();
void processLidarScan();

void startWebPortal();
void handleUDP();
void serviceEmergencyStop();
void sendUDPFeedback(String message);
void sendLog(String msg);

#endif