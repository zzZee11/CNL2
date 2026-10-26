
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// MOTOR DRIVER PINS
// ==========================================

// Left motor
constexpr int LEFT_MOTOR_IN1 = 25;
constexpr int LEFT_MOTOR_IN2 = 26;
constexpr int LEFT_MOTOR_PWM = 27;

// Right motor
constexpr int RIGHT_MOTOR_IN1 = 32;
constexpr int RIGHT_MOTOR_IN2 = 33;
constexpr int RIGHT_MOTOR_PWM = 23;


// ==========================================
// ENCODER PINS
// ==========================================

// Left encoder
constexpr int LEFT_ENCODER_A = 18;
constexpr int LEFT_ENCODER_B = 19;

// Right encoder
constexpr int RIGHT_ENCODER_A = 21;
constexpr int RIGHT_ENCODER_B = 22;


// ==========================================
// WHEEL CONFIGURATION
// ==========================================

// Wheel radius in metres
constexpr float WHEEL_RADIUS_M = 0.05f;

// Distance between left and right wheel centres
constexpr float WHEEL_BASE_M = 0.20f;

// Actual encoder counts per wheel revolution.
// IMPORTANT: Replace with your measured value.
constexpr float TICKS_PER_REVOLUTION = 360.0f;


// ==========================================
// PID CONFIGURATION
// ==========================================

// PID update interval
constexpr uint32_t PID_SAMPLE_TIME_MS = 50;

// Initial tuning values; adjust experimentally.
constexpr float PID_KP = 2.0f;
constexpr float PID_KI = 0.5f;
constexpr float PID_KD = 0.0f;


// ==========================================
// MOTOR PWM CONFIGURATION
// ==========================================

constexpr uint32_t MOTOR_PWM_FREQUENCY = 20000;
constexpr uint8_t MOTOR_PWM_RESOLUTION = 8;
constexpr int MOTOR_PWM_MAX = 255;


// ==========================================
// RASPBERRY PI UART CONFIGURATION
// ==========================================

// UART port used for Pi communication.
// ESP32-S3 has multiple hardware UARTs.
constexpr uint32_t UART_BAUD_RATE = 115200;

// Example UART pins; verify against your board.
constexpr int UART_RX_PIN = 16;
constexpr int UART_TX_PIN = 17;

// Stop accepting stale commands after this interval.
constexpr uint32_t COMMAND_TIMEOUT_MS = 500;


// ==========================================
// SERIAL TELEMETRY
// ==========================================

constexpr uint32_t TELEMETRY_INTERVAL_MS = 100;

#endif // CONFIG_H
