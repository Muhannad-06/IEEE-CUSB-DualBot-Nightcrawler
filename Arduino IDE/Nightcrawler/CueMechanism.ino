#include <Arduino.h>
#include <ESP32Servo.h>
#include "CueMechanism.h"

Servo myServo;
static bool lastButtonHeld = false;

enum CueState { CUE_IDLE, CUE_SPINNING };
static CueState cueState = CUE_IDLE;
static unsigned long spinStartTime = 0;

const int CUE_SPIN_PULSE_US       = 2000;  // full speed, one direction
const int CUE_STOP_PULSE_US       = 1500;  // neutral / stop
const unsigned long CUE_ROTATION_MS = 1000;  // <-- TUNE THIS on your hardware

void initMechanism(int servoPin) {
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    myServo.setPeriodHertz(50);
    myServo.attach(servoPin, 500, 2500);
    myServo.writeMicroseconds(CUE_STOP_PULSE_US);  // make sure it starts stopped, not spinning
}

void updateMechanism(bool buttonHeld) {
    bool pressedNow = (buttonHeld && !lastButtonHeld);
    lastButtonHeld = buttonHeld;

    // Start a new cycle only if idle (ignores presses mid-spin)
    if (pressedNow && cueState == CUE_IDLE) {
        cueState = CUE_SPINNING;
        spinStartTime = millis();
        myServo.writeMicroseconds(CUE_SPIN_PULSE_US);
        Serial.println("Cue Mechanism: X pressed - starting 360 rotation");
    }

    if (cueState == CUE_SPINNING) {
        if (millis() - spinStartTime >= CUE_ROTATION_MS) {
            myServo.writeMicroseconds(CUE_STOP_PULSE_US);
            cueState = CUE_IDLE;
            Serial.println("Cue Mechanism: rotation complete - stopped");
        }
    }
    // CUE_IDLE: do nothing, servo stays stopped
}
