#include "motors.h"

static bool motorsArmed = false; 

// Pin lookup table — index 0 = motor 1, index 3 = motor 4
static const uint8_t motorPins[4] = {
    MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN
};

static void writeMotor(uint8_t index, uint8_t duty) {
    if (duty > MOTOR_MAX) duty = MOTOR_MAX;
    ledcWrite(motorPins[index], duty);
}

void initMotors() {
    for (int i = 0; i < 4; i++) {
        ledcAttach(motorPins[i], PWM_FREQ, PWM_RES);
    }
    stopAllMotors(); 
    motorsArmed = false;
    Serial.println("[MOTORS] Initialized — all outputs at 0, disarmed.");
}

// Prevents the inrush current spike that can reset or brown-out the ESP32.
void armMotors() {
    Serial.println("[MOTORS] Arming — soft ramp starting...");
    for (uint8_t duty = 0; duty <= MOTOR_ARM_TARGET; duty++) {
        for (int i = 0; i < 4; i++) writeMotor(i, duty);
        delay(10); // 10ms per step × ~40 steps = ~400ms ramp
    }

    motorsArmed = true;
    Serial.printf("[MOTORS] Armed — running at idle duty %d/255.\n", MOTOR_ARM_TARGET);
}

void stopAllMotors() {
    for (int i = 0; i < 4; i++) ledcWrite(motorPins[i], 0);
    motorsArmed = false;
    Serial.println("[MOTORS] STOPPED — all motors off, disarmed.");
}

void setMotorSpeed(uint8_t motorNum, uint8_t speed) {
    if (!motorsArmed) return; 
    if (motorNum < 1 || motorNum > 4) return; 
    if (speed > 0 && speed < MOTOR_MIN) speed = MOTOR_MIN;

    writeMotor(motorNum - 1, speed); 
}

//Convenience function to update all four motors in one call.
void setMotorSpeeds(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4) {
    setMotorSpeed(1, m1);
    setMotorSpeed(2, m2);
    setMotorSpeed(3, m3);
    setMotorSpeed(4, m4);
}

bool motorsAreArmed() {
    return motorsArmed;
}