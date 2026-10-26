
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

private:
    static volatile long _leftTicks;
    static volatile long _rightTicks;

    static void IRAM_ATTR leftEncoderISR();
    static void IRAM_ATTR rightEncoderISR();
};

#endif
