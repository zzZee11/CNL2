
#ifndef WHEEL_SPEED_PID_H
#define WHEEL_SPEED_PID_H

#include <Arduino.h>

class WheelSpeedPID {
public:
    // Initialize PID timing and encoder count baseline.
    void begin(float countsPerRevolution, uint32_t sampleTimeMs);

    // Set PID gains.
    void setGains(float kp, float ki, float kd);

    // Update actual RPM and PID outputs when the sample interval expires.
    // Returns true when a new output is calculated.
    bool update(
        float targetLeftRPM,
        float targetRightRPM,
        long leftTicks,
        long rightTicks
    );

    float getLeftOutput() const;
    float getRightOutput() const;

    float getLeftRPM() const;
    float getRightRPM() const;

    // Reset PID state and synchronize encoder baselines.
    void reset(long leftTicks = 0, long rightTicks = 0);

private:
    float calculatePID(
        float targetRPM,
        float actualRPM,
        float dt,
        float &integral,
        float &previousError
    );

    float _countsPerRevolution = 360.0f;
    uint32_t _sampleTimeMs = 50;
    uint32_t _lastUpdateMs = 0;

    long _previousLeftTicks = 0;
    long _previousRightTicks = 0;

    float _kp = 2.0f;
    float _ki = 0.5f;
    float _kd = 0.0f;

    float _leftIntegral = 0.0f;
    float _rightIntegral = 0.0f;

    float _previousLeftError = 0.0f;
    float _previousRightError = 0.0f;

    float _leftRPM = 0.0f;
    float _rightRPM = 0.0f;

    float _leftOutput = 0.0f;
    float _rightOutput = 0.0f;
};

#endif
