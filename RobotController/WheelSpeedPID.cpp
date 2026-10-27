
#include "WheelSpeedPID.h"
#include "Config.h"

void WheelSpeedPID::begin(
    float countsPerRevolution,
    uint32_t sampleTimeMs
) {
    _countsPerRevolution = countsPerRevolution > 0
        ? countsPerRevolution
        : TICKS_PER_REVOLUTION;

    _sampleTimeMs = sampleTimeMs > 0
        ? sampleTimeMs
        : PID_SAMPLE_TIME_MS;

    reset();
}

void WheelSpeedPID::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void WheelSpeedPID::reset(long leftTicks, long rightTicks) {
    _previousLeftTicks = leftTicks;
    _previousRightTicks = rightTicks;

    _lastUpdateMs = millis();

    _leftIntegral = 0.0f;
    _rightIntegral = 0.0f;

    _previousLeftError = 0.0f;
    _previousRightError = 0.0f;

    _leftRPM = 0.0f;
    _rightRPM = 0.0f;

    _leftOutput = 0.0f;
    _rightOutput = 0.0f;
}

bool WheelSpeedPID::update(
    float targetLeftRPM,
    float targetRightRPM,
    long leftTicks,
    long rightTicks
) {
    uint32_t now = millis();
    uint32_t elapsedMs = now - _lastUpdateMs;

    if (elapsedMs < _sampleTimeMs) {
        return false;
    }

    float dt = elapsedMs / 1000.0f;

    long leftDelta = leftTicks - _previousLeftTicks;
    long rightDelta = rightTicks - _previousRightTicks;

    _previousLeftTicks = leftTicks;
    _previousRightTicks = rightTicks;
    _lastUpdateMs = now;

    _leftRPM = (leftDelta * 60.0f) /
               (_countsPerRevolution * dt);

    _rightRPM = (rightDelta * 60.0f) /
                (_countsPerRevolution * dt);

    if (targetLeftRPM == 0.0f) {
        _leftIntegral = 0.0f;
        _previousLeftError = 0.0f;
        _leftOutput = 0.0f;
    } else {
        _leftOutput = calculatePID(
            targetLeftRPM,
            _leftRPM,
            dt,
            _leftIntegral,
            _previousLeftError
        );
    }

    if (targetRightRPM == 0.0f) {
        _rightIntegral = 0.0f;
        _previousRightError = 0.0f;
        _rightOutput = 0.0f;
    } else {
        _rightOutput = calculatePID(
            targetRightRPM,
            _rightRPM,
            dt,
            _rightIntegral,
            _previousRightError
        );
    }

    return true;
}

float WheelSpeedPID::calculatePID(
    float targetRPM,
    float actualRPM,
    float dt,
    float &integral,
    float &previousError
) {
    float error = targetRPM - actualRPM;

    // Accumulate integral with anti-windup clamping to full PWM authority
    integral += error * dt;

    if (_ki > 0.0f) {
        float maxIntegral = static_cast<float>(MOTOR_PWM_MAX) / _ki;
        integral = constrain(integral, -maxIntegral, maxIntegral);
    } else {
        integral = 0.0f;
    }

    float derivative = (dt > 0.0f) ? ((error - previousError) / dt) : 0.0f;

    float output =
        (_kp * error) +
        (_ki * integral) +
        (_kd * derivative);

    previousError = error;

    return constrain(
        output,
        -static_cast<float>(MOTOR_PWM_MAX),
        static_cast<float>(MOTOR_PWM_MAX)
    );
}

float WheelSpeedPID::getLeftOutput() const {
    return _leftOutput;
}

float WheelSpeedPID::getRightOutput() const {
    return _rightOutput;
}

float WheelSpeedPID::getLeftRPM() const {
    return _leftRPM;
}

float WheelSpeedPID::getRightRPM() const {
    return _rightRPM;
}
