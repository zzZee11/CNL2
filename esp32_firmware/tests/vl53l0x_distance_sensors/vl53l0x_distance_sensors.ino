#include <Wire.h>
#include <VL53L0X.h>

// ==================================================
// I2C CONFIGURATION
// (Note: Pins 4-9 are reserved for motors, 10-13 for encoders, 17-18 for Pi UART)
// ==================================================

#define SDA_PIN 15
#define SCL_PIN 16

// ==================================================
// XSHUT PINS (Only 2 pins: Middle sensor has NO XSHUT)
// ==================================================

#define XSHUT_LEFT   1   // Left sensor XSHUT (Sensor 1, +45 deg)
#define XSHUT_RIGHT  2   // Right sensor XSHUT (Sensor 3, -45 deg)
// Note: Middle sensor (Sensor 2) has NO XSHUT pin connected (always powered)

// ==================================================
// I2C ADDRESSES
// ==================================================

#define SENSOR_1_ADDRESS 0x30
#define SENSOR_2_ADDRESS 0x31
#define SENSOR_3_ADDRESS 0x32

// ==================================================
// SENSOR OBJECTS
// ==================================================

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

// ==================================================
// INITIALIZE SENSORS
// ==================================================

bool initializeSensorWithXshut(
  VL53L0X &sensor,
  int xshutPin,
  uint8_t newAddress,
  const char *sensorName
) {
  // Enable this sensor via XSHUT.
  digitalWrite(xshutPin, HIGH);
  delay(30);

  // Initialize the sensor at its default address (0x29).
  if (!sensor.init()) {
    Serial.print(sensorName);
    Serial.println(": Initialization FAILED");
    return false;
  }

  // Assign a unique I2C address.
  sensor.setAddress(newAddress);
  sensor.setTimeout(500);
  sensor.startContinuous(100);

  Serial.print(sensorName);
  Serial.print(": Initialized successfully. I2C address = 0x");
  Serial.println(newAddress, HEX);
  return true;
}

bool initializeCenterSensorNoXshut(
  VL53L0X &sensor,
  uint8_t newAddress,
  const char *sensorName
) {
  // Center sensor is already powered at default address (0x29).
  if (!sensor.init()) {
    Serial.print(sensorName);
    Serial.println(": Initialization FAILED at 0x29");
    return false;
  }

  // Assign unique I2C address to free 0x29 for Left and Right.
  sensor.setAddress(newAddress);
  sensor.setTimeout(500);
  sensor.startContinuous(100);

  Serial.print(sensorName);
  Serial.print(": Initialized successfully (no XSHUT). I2C address = 0x");
  Serial.println(newAddress, HEX);
  return true;
}

// ==================================================
// READ ONE SENSOR
// ==================================================

void readSensor(
  VL53L0X &sensor,
  const char *sensorName
) {
  uint16_t distance = sensor.readRangeContinuousMillimeters();

  Serial.print(sensorName);
  Serial.print(" | ");

  if (sensor.timeoutOccurred()) {
    Serial.println("TIMEOUT");
    return;
  }

  // Avoid unsigned integer underflow when distance < 10.
  int lowerDistance = max(0, (int)distance - 10);
  int upperDistance = (int)distance + 10;

  Serial.print("Distance = ");
  Serial.print(lowerDistance);
  Serial.print(" - ");
  Serial.print(upperDistance);
  Serial.println(" mm");
}

// ==================================================
// SETUP
// ==================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("================================");
  Serial.println("  THREE VL53L0X SENSOR SYSTEM");
  Serial.println("================================");

  // Configure the 2 XSHUT pins (Middle sensor has no XSHUT pin connected)
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  // Disable Left and Right sensors initially
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);

  delay(50);

  // Initialize the I2C bus.
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("I2C bus initialized.");

  // Step 1: Initialize Center sensor (Sensor 2) first at default 0x29, then re-address to 0x31
  bool sensor2OK = initializeCenterSensorNoXshut(
    sensor2,
    SENSOR_2_ADDRESS,
    "Center Sensor (S2)"
  );

  if (!sensor2OK) {
    Serial.println("WARNING: Center Sensor (S2) not detected at 0x29.");
  }

  // Step 2: Release Left sensor (Sensor 1), boots at default 0x29, then re-address to 0x30
  bool sensor1OK = initializeSensorWithXshut(
    sensor1,
    XSHUT_LEFT,
    SENSOR_1_ADDRESS,
    "Left Sensor (S1)"
  );

  if (!sensor1OK) {
    Serial.println("WARNING: Left Sensor (S1) failed.");
  }

  // Step 3: Release Right sensor (Sensor 3), boots at default 0x29, then re-address to 0x32
  bool sensor3OK = initializeSensorWithXshut(
    sensor3,
    XSHUT_RIGHT,
    SENSOR_3_ADDRESS,
    "Right Sensor (S3)"
  );

  if (!sensor3OK) {
    Serial.println("WARNING: Right Sensor (S3) failed.");
  }

  Serial.println();
  Serial.println("All three sensors initialized successfully.");
  Serial.println("Starting distance measurements...");
  Serial.println();
}

// ==================================================
// MAIN LOOP
// ==================================================

void loop() {
  readSensor(sensor1, "Sensor 1 (0x30)");
  readSensor(sensor2, "Sensor 2 (0x31)");
  readSensor(sensor3, "Sensor 3 (0x32)");

  Serial.println("--------------------------------");

  delay(4000);
}