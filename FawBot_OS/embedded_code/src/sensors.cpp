#include "config.h"

void initSensors() {
    pinMode(IR_LEFT, INPUT);
    pinMode(IR_RIGHT, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
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