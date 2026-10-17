#pragma once

#include <Arduino.h>
#include "config.h"
#include "motor_driver.h"

class RobotController {
public:
    RobotController(MotorDriver& motorDriver);

    void init();
    void setCommand(DriveState state, uint8_t speed);
    void update(unsigned long nowMs, float rpmL, float rpmR, int64_t ticksL, int64_t ticksR);
    void reset();

    DriveState getCommandedState() const { return _targetState; }
    uint8_t getCommandedSpeed() const { return _targetSpeed; }

    int16_t getOutputPwmL() const { return _lastPwmL; }
    int16_t getOutputPwmR() const { return _lastPwmR; }

private:
    MotorDriver& _motors;

    DriveState _targetState;
    uint8_t _targetSpeed;
    unsigned long _lastUpdateTime;

    int64_t _startSyncTicksL;
    int64_t _startSyncTicksR;
    bool _syncActive;

    int16_t _lastPwmL;
    int16_t _lastPwmR;
};
