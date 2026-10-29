#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <Arduino.h>

class EncoderReader {
public:
    // Initialize encoder pins and interrupts.
    void begin();

    // Get the current cumulative encoder tick counts.
    long getLeftTicks();
    long getRightTicks();

    // Reset both tick counts to zero.
    void reset();

    // Set commanded motor directions for fallback when Phase B is disconnected/stuck
    static void setMotorDirections(bool leftForward, bool rightForward);

    // Diagnostic methods to verify physical encoder signals
    static uint32_t getLeftTransitionsA();
    static uint32_t getLeftTransitionsB();
    static bool isLeftPhaseBHealthy();

private:
    static volatile long _leftTicks;
    static volatile long _rightTicks;

    static volatile bool _leftCommandedForward;
    static volatile bool _rightCommandedForward;

    static volatile uint32_t _leftTransitionsA;
    static volatile uint32_t _leftTransitionsB;
    static volatile uint32_t _rightTransitionsA;
    static volatile uint32_t _rightTransitionsB;

    static void IRAM_ATTR leftEncoderISR();
    static void IRAM_ATTR leftEncoderB_ISR();
    static void IRAM_ATTR rightEncoderISR();
    static void IRAM_ATTR rightEncoderB_ISR();
};

#endif
