#pragma once

#include <Arduino.h>
#include "config.h"

class EncoderManager {
public:
    EncoderManager();

    void init();
    void update(unsigned long nowMs);

    int64_t getTicksLeft();
    int64_t getTicksRight();

    float getRpmLeft() const { return _rpmLeft; }
    float getRpmRight() const { return _rpmRight; }

    void reset();

    // ISR routines (must be static or free functions)
    static void IRAM_ATTR isrLeft();
    static void IRAM_ATTR isrRight();

private:
    static volatile int64_t _ticksLeft;
    static volatile int64_t _ticksRight;
    static portMUX_TYPE _mux;

    int64_t _prevTicksLeft;
    int64_t _prevTicksRight;
    unsigned long _lastCalcTime;

    float _rpmLeft;
    float _rpmRight;
};

