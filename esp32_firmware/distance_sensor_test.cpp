#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Define XSHUT pins for each sensor based on your wiring
const int XSHUT_U6 = 1; // Example GPIO for sensor U6 XSHUT
const int XSHUT_U7 = 2; // Example GPIO for sensor U7 XSHUT
const int XSHUT_U8 = 3; // Example GPIO for sensor U8 XSHUT

// Unique I2C addresses assigned during setup
#const uint8_t LOX1_ADDRESS = 0x30;
#const uint8_t LOX2_ADDRESS = 0x31;
#const uint8_t LOX3_ADDRESS = 0x32;

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

void setupSensors() {	
  pinMode(XSHUT_U6, OUTPUT);
  pinMode(XSHUT_U7, OUTPUT);
  pinMode(XSHUT_U8, OUTPUT);

  // Reset all sensors by pulling XSHUT low
  digitalWrite(XSHUT_U6, LOW);
  digitalWrite(XSHUT_U7, LOW);
  digitalWrite(XSHUT_U8, LOW);
  delay(10);

  // --- Initialize Sensor 1 (U6) ---
  digitalWrite(XSHUT_U6, HIGH);
  delay(10);
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot VL53L0X (U6)"));
    while(1);
  }

  // --- Initialize Sensor 2 (U7) ---
  digitalWrite(XSHUT_U7, HIGH);
  delay(10);
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot VL53L0X (U7)"));
    while(1);
  }

  // --- Initialize Sensor 3 (U8) ---
  digitalWrite(XSHUT_U8, HIGH);
  delay(10);
  if(!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot VL53L0X (U8)"));
    while(1);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Wire.begin();
  
  setupSensors();
  Serial.println("All VL53L0X sensors initialized successfully!");
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure1, measure2, measure3;

  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);

  Serial.print("U6 Distance: ");
  if (measure1.RangeStatus != 4) Serial.print(measure1.RangeMilliMeter);
  else Serial.print("Out of range");

  Serial.print(" | U7 Distance: ");
  if (measure2.RangeStatus != 4) Serial.print(measure2.RangeMilliMeter);
  else Serial.print("Out of range");

  Serial.print(" | U8 Distance: ");
  if (measure3.RangeStatus != 4) Serial.println(measure3.RangeMilliMeter);
  else Serial.println("Out of range");

  delay(500);
}