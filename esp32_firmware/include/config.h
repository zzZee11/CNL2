#pragma once

#include <Arduino.h>

// ==============================================================================
// Hardware Pinout Configuration (ESP32-S3)
// ==============================================================================

// L298N Motor Driver Pins (Swapped Left <-> Right)
// Left Motor (Physical Channel B)
#define PIN_MOTOR_L_ENA     9   // PWM speed control (LEDC)
#define PIN_MOTOR_L_IN1     7   // Direction input 1
#define PIN_MOTOR_L_IN2     8   // Direction input 2

// Right Motor (Physical Channel A)
#define PIN_MOTOR_R_ENB     4   // PWM speed control (LEDC)
#define PIN_MOTOR_R_IN3     5   // Direction input 3
#define PIN_MOTOR_R_IN4     6   // Direction input 4

// Quadrature Encoders (Swapped Left <-> Right)
// Left Wheel Encoder
#define PIN_ENC_L_A         12  // Phase A (Interrupt)
#define PIN_ENC_L_B         13  // Phase B (Direction)

// Right Wheel Encoder
#define PIN_ENC_R_A         10  // Phase A (Interrupt)
#define PIN_ENC_R_B         11  // Phase B (Direction)

// ==============================================================================
// Hardware UART for Raspberry Pi 4B GPIO Serial Connection
// ==============================================================================
// Direct 3.3V logic connection between ESP32-S3 and Raspberry Pi 4B GPIO header:
//   ESP32-S3 PIN_UART_RPI_RX (GPIO 18) <---> Raspberry Pi GPIO 14 (Physical Pin 8,  TXD)
//   ESP32-S3 PIN_UART_RPI_TX (GPIO 17) <---> Raspberry Pi GPIO 15 (Physical Pin 10, RXD)
//   ESP32-S3 GND                       <---> Raspberry Pi GND     (Physical Pin 6 or 9)
#define USE_HARDWARE_UART_RPI   true
#define PIN_UART_RPI_RX         18  // ESP32 RX pin (receives from Pi TX)
#define PIN_UART_RPI_TX         17  // ESP32 TX pin (transmits to Pi RX)

// ==============================================================================
// 3x VL53L0X Time-of-Flight (ToF) Distance Sensors Configuration
// ==============================================================================
// 3-sensor obstacle sensing: Left (+45 deg), Center (0 deg forward), Right (-45 deg)
// Hardware Setup (2 XSHUT Pins):
//   - Left Sensor (S1)  : XSHUT connected to PIN_XSHUT_LEFT
//   - Center Sensor (S2): NO XSHUT connection (always powered, boot-configured first at 0x29)
//   - Right Sensor (S3) : XSHUT connected to PIN_XSHUT_RIGHT
#define USE_VL53L0X_SENSORS     true

#define PIN_I2C_SDA             15  // ESP32 I2C SDA (configurable to any free GPIO)
#define PIN_I2C_SCL             16  // ESP32 I2C SCL (configurable to any free GPIO)

// Only 2 XSHUT pins required (Center sensor has no XSHUT connected)
#define PIN_XSHUT_LEFT          1   // Left sensor XSHUT (Sensor 1, +45 deg)
#define PIN_XSHUT_RIGHT         2   // Right sensor XSHUT (Sensor 3, -45 deg)

// Unique I2C addresses assigned sequentially during boot
#define I2C_ADDR_VL53L0X_LEFT   0x30
#define I2C_ADDR_VL53L0X_CENTER 0x31  // Center is re-addressed from 0x29 -> 0x31 first
#define I2C_ADDR_VL53L0X_RIGHT  0x32

#define TOF_READ_INTERVAL_MS    50  // 20 Hz distance sampling rate

// ==============================================================================
// Motor & Inversion Settings
// ==============================================================================
#define INVERT_LEFT_MOTOR     false
#define INVERT_RIGHT_MOTOR    false

// Steering behavior for A and D keys:
// When true:
//   'A' -> ONLY Left wheel turns forward (Right wheel stays stopped at 0)
//   'D' -> ONLY Right wheel turns forward (Left wheel stays stopped at 0)
// When false:
//   'A' -> Right wheel turns forward (swings car body left)
//   'D' -> Left wheel turns forward (swings car body right)
#define TURN_A_DRIVES_LEFT_WHEEL   true

// PWM settings
#define PWM_FREQUENCY       20000   // 20 kHz ultrasonic frequency (eliminates audible motor whine)
#define PWM_RESOLUTION      8       // 8-bit resolution (0 - 255)
#define PWM_CHANNEL_L       0       // LEDC Channel for Left Motor
#define PWM_CHANNEL_R       1       // LEDC Channel for Right Motor

#define DEFAULT_DRIVE_SPEED 200     // Default drive speed PWM (0 - 255)
#define DEFAULT_TURN_SPEED  180     // Default turn speed PWM (0 - 255)

// ==============================================================================
// Encoder & Odometry Parameters
// ==============================================================================
// Pulses Per Revolution (PPR) of the wheel (1 full wheel turn)
#define WHEEL_PULSES_PER_REV    330.0f
#define TOTAL_TICKS_PER_REV     WHEEL_PULSES_PER_REV
#define PULSES_PER_REVOLUTION   WHEEL_PULSES_PER_REV

#define INVERT_LEFT_ENCODER   true
#define INVERT_RIGHT_ENCODER  false

// ==============================================================================
// Closed-Loop Synchronization Parameters
// ==============================================================================
#define SYNC_LOOP_INTERVAL_MS   30      // Loop interval (~33 Hz)

// Straight-line encoder synchronization gain:
// Proportional trim based on tick difference: trim = SYNC_KP * (deltaL - deltaR)
#define SYNC_KP                 0.5f

// Maximum allowed sync trim (prevents runaway/motor saturation)
#define MAX_SYNC_TRIM_PWM       40

// ==============================================================================
// Safety Watchdog & Telemetry Rates
// ==============================================================================
#define WATCHDOG_TIMEOUT_MS     350     // Auto-stop motors if no serial command in 350 ms
#define TELEMETRY_INTERVAL_MS   50      // 20 Hz telemetry stream
#define RPM_CALC_INTERVAL_MS    50      // Rolling window for RPM calculations
#define SERIAL_BAUD_RATE        115200  // USB Serial baud rate
