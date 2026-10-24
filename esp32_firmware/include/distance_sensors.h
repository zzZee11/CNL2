#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

#if USE_VL53L0X_SENSORS
#include <VL53L0X.h>
#endif

class DistanceSensors {
public:
    DistanceSensors();

    void init();
    void update(unsigned long nowMs);

    int16_t getDistanceLeftMm() const { return _distLeftMm; }
    int16_t getDistanceCenterMm() const { return _distCenterMm; }
    int16_t getDistanceRightMm() const { return _distRightMm; }

    bool isLeftReady() const { return _sensorLeftReady; }
    bool isCenterReady() const { return _sensorCenterReady; }
    bool isRightReady() const { return _sensorRightReady; }

private:
#if USE_VL53L0X_SENSORS
    VL53L0X _sensorLeft;
    VL53L0X _sensorCenter;
    VL53L0X _sensorRight;

    bool initSingleSensor(VL53L0X& sensor, int xshutPin, uint8_t newAddress, const char* name);
#endif

    bool _sensorLeftReady;
    bool _sensorCenterReady;
    bool _sensorRightReady;

    int16_t _distLeftMm;
    int16_t _distCenterMm;
    int16_t _distRightMm;

    unsigned long _lastReadTime;
};
