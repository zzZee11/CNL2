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
bool DistanceSensors::initSingleSensor(VL53L0X& sensor, int xshutPin, uint8_t newAddress, const char* name) {
    // Keep sensor disabled
    digitalWrite(xshutPin, LOW);
    delay(10);

    // Enable this sensor
    digitalWrite(xshutPin, HIGH);
    delay(20);

    // Initialize at default address (0x29)
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
    pinMode(PIN_XSHUT_LEFT, OUTPUT);
    pinMode(PIN_XSHUT_CENTER, OUTPUT);
    pinMode(PIN_XSHUT_RIGHT, OUTPUT);

    // Disable all 3 sensors
    digitalWrite(PIN_XSHUT_LEFT, LOW);
    digitalWrite(PIN_XSHUT_CENTER, LOW);
    digitalWrite(PIN_XSHUT_RIGHT, LOW);
    delay(20);

    // Start I2C bus on configured pins
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(100000);

    // Sequentially initialize each sensor so they obtain their unique address
    _sensorLeftReady = initSingleSensor(_sensorLeft, PIN_XSHUT_LEFT, I2C_ADDR_VL53L0X_LEFT, "Left Sensor (S1)");
    _sensorCenterReady = initSingleSensor(_sensorCenter, PIN_XSHUT_CENTER, I2C_ADDR_VL53L0X_CENTER, "Center Sensor (S2)");
    _sensorRightReady = initSingleSensor(_sensorRight, PIN_XSHUT_RIGHT, I2C_ADDR_VL53L0X_RIGHT, "Right Sensor (S3)");
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
