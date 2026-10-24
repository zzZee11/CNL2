#include "distance_sensors.h"

DistanceSensors::DistanceSensors()
    : _sensorLeftReady(false)
    , _sensorCenterReady(false)
    , _sensorRightReady(false)
    , _distLeftMm(-1)
    , _distCenterMm(-1)
    , _distRightMm(-1)
    , _currentSlot(0)
    , _lastSlotTime(0)
    , _totalReads(0)
    , _totalTimeouts(0)
{
    for (int i = 0; i < 3; i++) {
        _consecutiveTimeouts[i] = 0;
        _lastValidTime[i] = 0;
        _offlineRetryTime[i] = 0;
    }
}

#if HAVE_VL53L0X_LIB
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
    sensor.setTimeout(TOF_READ_TIMEOUT_MS);
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
    sensor.setTimeout(TOF_READ_TIMEOUT_MS);
    sensor.startContinuous(50);

    Serial.printf("# [ToF] %s initialized at 0x%02X\n", name, newAddress);
    return true;
}

void DistanceSensors::_pollSensor(VL53L0X& sensor, uint8_t idx, int16_t& distOut, unsigned long nowMs) {
    // If sensor has repeatedly timed out (offline/unplugged), back off and probe only every 2 seconds
    if (_consecutiveTimeouts[idx] >= 3 && (nowMs < _offlineRetryTime[idx])) {
        distOut = -1;
        return;
    }

    uint16_t d = sensor.readRangeContinuousMillimeters();
    if (sensor.timeoutOccurred() || d > 8190) {
        _consecutiveTimeouts[idx]++;
        _totalTimeouts++;
        distOut = -1;
        if (_consecutiveTimeouts[idx] >= 3) {
            _offlineRetryTime[idx] = nowMs + 2000; // 2-second backoff to prevent bus stalling
        }
    } else {
        _consecutiveTimeouts[idx] = 0;
        _totalReads++;
        distOut = (int16_t)d;
        _lastValidTime[idx] = nowMs;
    }
}
#endif

void DistanceSensors::init() {
#if HAVE_VL53L0X_LIB
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
#else
    Serial.println("# [ToF] Notice: 'VL53L0X by Pololu' library not installed in Arduino IDE.");
    Serial.println("# [ToF] Install it via Library Manager: Tools -> Manage Libraries -> search 'VL53L0X'");
#endif
}

void DistanceSensors::update(unsigned long nowMs) {
#if HAVE_VL53L0X_LIB
    // Priority 3: Non-blocking round-robin time-sliced polling.
    // Services at most ONE sensor per time slot (20 ms), guaranteeing the ESP32
    // main loop is never blocked by concurrent I2C transactions.
    if (nowMs - _lastSlotTime >= TOF_SLOT_INTERVAL_MS) {
        _lastSlotTime = nowMs;
        uint8_t slot = _currentSlot;
        _currentSlot = (_currentSlot + 1) % 3;

        if (slot == 0 && _sensorLeftReady) {
            _pollSensor(_sensorLeft, 0, _distLeftMm, nowMs);
        } else if (slot == 1 && _sensorCenterReady) {
            _pollSensor(_sensorCenter, 1, _distCenterMm, nowMs);
        } else if (slot == 2 && _sensorRightReady) {
            _pollSensor(_sensorRight, 2, _distRightMm, nowMs);
        }
    }

    // Safety behavior: invalidate readings (mark -1 mm) if sensor data is stale (> 250 ms)
    if (_lastValidTime[0] > 0 && (nowMs - _lastValidTime[0] > TOF_STALE_TIMEOUT_MS)) {
        _distLeftMm = -1;
    }
    if (_lastValidTime[1] > 0 && (nowMs - _lastValidTime[1] > TOF_STALE_TIMEOUT_MS)) {
        _distCenterMm = -1;
    }
    if (_lastValidTime[2] > 0 && (nowMs - _lastValidTime[2] > TOF_STALE_TIMEOUT_MS)) {
        _distRightMm = -1;
    }
#endif
}

