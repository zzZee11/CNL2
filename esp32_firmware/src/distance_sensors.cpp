#include "distance_sensors.h"

DistanceSensors::DistanceSensors()
    : _sensorLeftReady(false)
    , _sensorCenterReady(false)
    , _sensorRightReady(false)
    , _distLeftMm(-1)
    , _distCenterMm(-1)
    , _distRightMm(-1)
    , _lastReadTime(0)
{
}

#if USE_VL53L0X_SENSORS
// Center sensor has NO XSHUT pin connected: it is always powered.
// It is the only sensor active on the bus initially (while Left & Right are held LOW).
bool DistanceSensors::initCenterSensor(VL53L0X& sensor, uint8_t newAddress, const char* name) {
    // Sensor is already at factory default 0x29
    if (!sensor.init()) {
        Serial.printf("# [ToF] %s failed to boot at default 0x29\n", name);
        return false;
    }

    // Assign custom address to free 0x29 for subsequent sensors
    sensor.setAddress(newAddress);
    sensor.setTimeout(100);
    sensor.startContinuous(50);

    Serial.printf("# [ToF] %s (no XSHUT) re-addressed 0x29 -> 0x%02X OK\n", name, newAddress);
    return true;
}

// Left and Right sensors have dedicated XSHUT pins.
bool DistanceSensors::initSensorWithXshut(VL53L0X& sensor, int xshutPin, uint8_t newAddress, const char* name) {
    // Release sensor from shutdown
    digitalWrite(xshutPin, HIGH);
    delay(20);

    // Sensor boots at default address 0x29
    if (!sensor.init()) {
        Serial.printf("# [ToF] %s failed to boot\n", name);
        return false;
    }

    // Set custom I2C address
    sensor.setAddress(newAddress);
    sensor.setTimeout(100);
    sensor.startContinuous(50);

    Serial.printf("# [ToF] %s initialized at 0x%02X\n", name, newAddress);
    return true;
}
#endif

void DistanceSensors::init() {
#if USE_VL53L0X_SENSORS
    // 1. Configure the 2 available XSHUT pins (Left and Right)
    pinMode(PIN_XSHUT_LEFT, OUTPUT);
    pinMode(PIN_XSHUT_RIGHT, OUTPUT);

    // 2. Hold Left and Right sensors in hardware shutdown (LOW).
    // The Center sensor has NO XSHUT connection, so it is the ONLY sensor active on I2C at 0x29.
    digitalWrite(PIN_XSHUT_LEFT, LOW);
    digitalWrite(PIN_XSHUT_RIGHT, LOW);
    delay(20);

    // 3. Start I2C bus on configured pins
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(100000);

    // 4. Initialize Center sensor first and move its address to I2C_ADDR_VL53L0X_CENTER (0x31)
    _sensorCenterReady = initCenterSensor(_sensorCenter, I2C_ADDR_VL53L0X_CENTER, "Center Sensor (S2)");

    // 5. Release Left sensor, initialize at 0x29, and move to I2C_ADDR_VL53L0X_LEFT (0x30)
    _sensorLeftReady = initSensorWithXshut(_sensorLeft, PIN_XSHUT_LEFT, I2C_ADDR_VL53L0X_LEFT, "Left Sensor (S1)");

    // 6. Release Right sensor, initialize at 0x29, and move to I2C_ADDR_VL53L0X_RIGHT (0x32)
    _sensorRightReady = initSensorWithXshut(_sensorRight, PIN_XSHUT_RIGHT, I2C_ADDR_VL53L0X_RIGHT, "Right Sensor (S3)");
#endif
}

void DistanceSensors::update(unsigned long nowMs) {
#if USE_VL53L0X_SENSORS
    if (nowMs - _lastReadTime < TOF_READ_INTERVAL_MS) {
        return;
    }
    _lastReadTime = nowMs;

    if (_sensorLeftReady) {
        uint16_t d = _sensorLeft.readRangeContinuousMillimeters();
        _distLeftMm = (_sensorLeft.timeoutOccurred() || d > 8190) ? -1 : (int16_t)d;
    } else {
        _distLeftMm = -1;
    }

    if (_sensorCenterReady) {
        uint16_t d = _sensorCenter.readRangeContinuousMillimeters();
        _distCenterMm = (_sensorCenter.timeoutOccurred() || d > 8190) ? -1 : (int16_t)d;
    } else {
        _distCenterMm = -1;
    }

    if (_sensorRightReady) {
        uint16_t d = _sensorRight.readRangeContinuousMillimeters();
        _distRightMm = (_sensorRight.timeoutOccurred() || d > 8190) ? -1 : (int16_t)d;
    } else {
        _distRightMm = -1;
    }
#endif
}

