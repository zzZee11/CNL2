#pragma once

#include <Arduino.h>
#include "config.h"

// Motion States (supports single and multi-key compound directions)
enum DriveState {
    STATE_STOP,
    STATE_FWD,         // W
    STATE_REV,         // S
    STATE_LEFT,        // A
    STATE_RIGHT,       // D
    STATE_FWD_LEFT,    // W + A
    STATE_FWD_RIGHT,   // W + D
    STATE_REV_LEFT,    // S + A
    STATE_REV_RIGHT,   // S + D
    STATE_BRAKE        // Space
};

class MotorDriver {
public:
    MotorDriver();

    void init();
    void setCommand(DriveState state, uint8_t speed);
    void setRawMotors(int16_t leftPwm, int16_t rightPwm);
    void setState(DriveState state) { _currentState = state; }
    void stop();
    void brake();

    DriveState getCurrentState() const { return _currentState; }
    uint8_t getCurrentSpeed() const { return _currentSpeed; }
    const char* getStateString() const;

private:
    DriveState _currentState;
    uint8_t _currentSpeed;

    void _writeLeftPwm(uint8_t pwm);
    void _writeRightPwm(uint8_t pwm);
};
