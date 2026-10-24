#include <Wire.h>
#include <VL53L0X.h>

// ==================================================
// I2C CONFIGURATION
// (Note: Pins 4-9 are reserved for motors, 10-13 for encoders, 17-18 for Pi UART)
// ==================================================

#define SDA_PIN 15
#define SCL_PIN 16

// ==================================================
// XSHUT PINS
// ==================================================

#define XSHUT_1 1   // Sensor 1 (Left,   +45 deg)
#define XSHUT_2 2   // Sensor 2 (Center,   0 deg forward)
#define XSHUT_3 3   // Sensor 3 (Right,  -45 deg)

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
// INITIALIZE ONE SENSOR
// ==================================================

bool initializeSensor(
  VL53L0X &sensor,
  int xshutPin,
  uint8_t newAddress,
  const char *sensorName
) {
  // Keep the sensor disabled initially.
  digitalWrite(xshutPin, LOW);
  delay(20);

  // Enable this sensor.
  digitalWrite(xshutPin, HIGH);
  delay(50);

  // Initialize the sensor at its default address (0x29).
  if (!sensor.init()) {
    Serial.print(sensorName);
    Serial.println(": Initialization FAILED");
    return false;
  }

  // Assign a unique I2C address.
  sensor.setAddress(newAddress);

  // Configure timeout.
  sensor.setTimeout(1000);

  // Configure continuous distance measurements.
  sensor.startContinuous(100);

  Serial.print(sensorName);
  Serial.print(": Initialized successfully. I2C address = 0x");
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

  // Configure XSHUT pins.
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);

  // Disable all sensors before initialization.
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);

  delay(100);

  // Initialize the I2C bus.
  Wire.begin(SDA_PIN, SCL_PIN);

  Wire.setClock(100000);

  Serial.println("I2C bus initialized.");

  // Initialize sensors sequentially.
  // Each previously initialized sensor remains enabled
  // at its newly assigned address.

  bool sensor1OK = initializeSensor(
    sensor1,
    XSHUT_1,
    SENSOR_1_ADDRESS,
    "Sensor 1"
  );

  if (!sensor1OK) {
    Serial.println("ERROR: Sensor 1 failed. Check wiring.");
    while (true) {
      delay(1000);
    }
  }

  bool sensor2OK = initializeSensor(
    sensor2,
    XSHUT_2,
    SENSOR_2_ADDRESS,
    "Sensor 2"
  );

  if (!sensor2OK) {
    Serial.println("ERROR: Sensor 2 failed. Check wiring.");
    while (true) {
      delay(1000);
    }
  }

  bool sensor3OK = initializeSensor(
    sensor3,
    XSHUT_3,
    SENSOR_3_ADDRESS,
    "Sensor 3"
  );

  if (!sensor3OK) {
    Serial.println("ERROR: Sensor 3 failed. Check wiring.");
    while (true) {
      delay(1000);
    }
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