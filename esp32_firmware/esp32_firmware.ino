// Arduino IDE Entrypoint for ESP32-S3 Robot Car Firmware
// Subsystem modules and main setup/loop are in src/
// (src/config.h, src/motor_driver.cpp, src/encoder.cpp, src/pid_controller.cpp,
//  src/distance_sensors.cpp, src/main.cpp)

#include "src/config.h"
#include "src/motor_driver.h"
#include "src/encoder.h"
#include "src/pid_controller.h"
#include "src/distance_sensors.h"

// Note for Arduino IDE:
// 1. Board: "ESP32S3 Dev Module"
// 2. Tools -> USB CDC On Boot: "Enabled"
// 3. Tools -> Upload Mode: "UART0 / Hardware CDC"
// 4. (Optional) Install "VL53L0X by Pololu" from Library Manager for distance sensors
