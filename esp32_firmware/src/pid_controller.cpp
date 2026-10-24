#include "pid_controller.h"

RobotController::RobotController(MotorDriver& motorDriver)
    : _motors(motorDriver),
      _targetState(STATE_STOP),
      _targetSpeed(0),
      _lastUpdateTime(0),
      _startSyncTicksL(0),
      _startSyncTicksR(0),
      _syncActive(false),
      _lastPwmL(0),
      _lastPwmR(0) {}

void RobotController::init() {
    reset();
}

void RobotController::reset() {
    _targetState = STATE_STOP;
    _targetSpeed = 0;
    _syncActive = false;
    _lastPwmL = 0;
    _lastPwmR = 0;
    _lastUpdateTime = millis();
    _motors.stop();
}

void RobotController::setRawMotors(int16_t leftPwm, int16_t rightPwm) {
    _lastPwmL = leftPwm;
    _lastPwmR = rightPwm;
    _syncActive = false;
    if (leftPwm == 0 && rightPwm == 0) {
        _targetState = STATE_STOP;
    } else if (leftPwm > 0 && rightPwm > 0) {
        _targetState = STATE_FWD;
    } else if (leftPwm < 0 && rightPwm < 0) {
        _targetState = STATE_REV;
    } else {
        _targetState = (leftPwm < rightPwm) ? STATE_LEFT : STATE_RIGHT;
    }
    _motors.setRawMotors(leftPwm, rightPwm);
}

void RobotController::setCommand(DriveState state, uint8_t speed) {
    _targetState = state;
    _targetSpeed = speed;
    _motors.setState(state);
    _syncActive = false; // Reset sync baseline on any state transition

    switch (state) {
        case STATE_FWD: {
            _lastPwmL = (int16_t)speed;
            _lastPwmR = (int16_t)speed;
            _motors.setRawMotors(_lastPwmL, _lastPwmR);
            break;
        }

        case STATE_REV: {
            _lastPwmL = -(int16_t)speed;
            _lastPwmR = -(int16_t)speed;
            _motors.setRawMotors(_lastPwmL, _lastPwmR);
            break;
        }

        case STATE_LEFT: {
#if TURN_A_DRIVES_LEFT_WHEEL
            // 'A' key: Left wheel only
            _lastPwmL = (int16_t)speed;
            _lastPwmR = 0;
#else
            _lastPwmL = 0;
            _lastPwmR = (int16_t)speed;
#endif
            _motors.setRawMotors(_lastPwmL, _lastPwmR);
            break;
        }

        case STATE_RIGHT: {
#if TURN_A_DRIVES_LEFT_WHEEL
            // 'D' key: Right wheel only
            _lastPwmL = 0;
            _lastPwmR = (int16_t)speed;
#else
            _lastPwmL = (int16_t)speed;
            _lastPwmR = 0;
#endif
            _motors.setRawMotors(_lastPwmL, _lastPwmR);
            break;
        }

        case STATE_FWD_LEFT: {
            // W + A: Forward-Left arc turn
            _lastPwmL = (int16_t)(speed / 3);
            _lastPwmR = (int16_t)speed;
            _motors.setRawMotors(_lastPwmL, _lastPwmR);
            break;
        }

        case STATE_FWD_RIGHT: {
            // W + D: Forward-Right arc turn
            _lastPwmL = (int16_t)speed;
            _lastPwmR = (int16_t)(speed / 3);
            _motors.setRawMotors(_lastPwmL, _lastPwmR);
            break;
        }

        case STATE_REV_LEFT: {
            // S + A: Reverse-Left arc turn
            _lastPwmL = -(int16_t)(speed / 3);
            _lastPwmR = -(int16_t)speed;
            _motors.setRawMotors(_lastPwmL, _lastPwmR);
            break;
        }

        case STATE_REV_RIGHT: {
            // S + D: Reverse-Right arc turn
            _lastPwmL = -(int16_t)speed;
            _lastPwmR = -(int16_t)(speed / 3);
            _motors.setRawMotors(_lastPwmL, _lastPwmR);
            break;
        }

        case STATE_BRAKE: {
            _lastPwmL = 0;
            _lastPwmR = 0;
            _motors.brake();
            break;
        }

        case STATE_STOP:
        default: {
            _lastPwmL = 0;
            _lastPwmR = 0;
            _motors.stop();
            break;
        }
    }
}

void RobotController::update(unsigned long nowMs, float rpmL, float rpmR, int64_t ticksL, int64_t ticksR) {
    if (_targetState == STATE_STOP) {
        _motors.stop();
        _lastPwmL = 0;
        _lastPwmR = 0;
        return;
    }
    if (_targetState == STATE_BRAKE) {
        _motors.brake();
        _lastPwmL = 0;
        _lastPwmR = 0;
        return;
    }

    // Straight-line encoder synchronization only runs during pure FWD or REV
    if (_targetState != STATE_FWD && _targetState != STATE_REV) {
        return;
    }

    unsigned long dtMs = nowMs - _lastUpdateTime;
    if (dtMs < SYNC_LOOP_INTERVAL_MS) {
        return;
    }
    _lastUpdateTime = nowMs;

    // Capture baseline ticks on first cycle of straight-line driving
    if (!_syncActive) {
        _startSyncTicksL = ticksL;
        _startSyncTicksR = ticksR;
        _syncActive = true;
        return;
    }

    if (_targetState == STATE_FWD) {
        int64_t deltaL = ticksL - _startSyncTicksL;
        int64_t deltaR = ticksR - _startSyncTicksR;
        int64_t tickDiff = deltaL - deltaR;

        // If Left is ahead (tickDiff > 0), slow down Left and speed up Right
        float rawTrim = SYNC_KP * (float)tickDiff;
        int16_t trim = (int16_t)constrain((int)round(rawTrim), -MAX_SYNC_TRIM_PWM, MAX_SYNC_TRIM_PWM);

        int16_t pwmL = (int16_t)constrain((int)_targetSpeed - trim, 0, 255);
        int16_t pwmR = (int16_t)constrain((int)_targetSpeed + trim, 0, 255);

        _lastPwmL = pwmL;
        _lastPwmR = pwmR;
        _motors.setRawMotors(pwmL, pwmR);
    }
    else if (_targetState == STATE_REV) {
        int64_t deltaL = abs(ticksL - _startSyncTicksL);
        int64_t deltaR = abs(ticksR - _startSyncTicksR);
        int64_t tickDiff = deltaL - deltaR;

        float rawTrim = SYNC_KP * (float)tickDiff;
        int16_t trim = (int16_t)constrain((int)round(rawTrim), -MAX_SYNC_TRIM_PWM, MAX_SYNC_TRIM_PWM);

        int16_t pwmL = (int16_t)constrain((int)_targetSpeed - trim, 0, 255);
        int16_t pwmR = (int16_t)constrain((int)_targetSpeed + trim, 0, 255);

        _lastPwmL = -pwmL;
        _lastPwmR = -pwmR;
        _motors.setRawMotors(-pwmL, -pwmR);
    }
}
