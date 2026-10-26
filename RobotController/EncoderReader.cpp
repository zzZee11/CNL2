#include "EncoderReader.h"
#include "Config.h"

// Shared tick counters updated by interrupts.
volatile long EncoderReader::_leftTicks = 0;
volatile long EncoderReader::_rightTicks = 0;

// Protect tick counters when the main program reads or resets them.
static portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

void EncoderReader::begin() {
    pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
    pinMode(LEFT_ENCODER_B, INPUT_PULLUP);

    pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(LEFT_ENCODER_A),
        leftEncoderISR,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(RIGHT_ENCODER_A),
        rightEncoderISR,
        CHANGE
    );
}

void IRAM_ATTR EncoderReader::leftEncoderISR() {
    portENTER_CRITICAL_ISR(&encoderMux);

    if (digitalRead(LEFT_ENCODER_A) ==
        digitalRead(LEFT_ENCODER_B)) {
        _leftTicks++;
    } else {
        _leftTicks--;
    }

    portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR EncoderReader::rightEncoderISR() {
    portENTER_CRITICAL_ISR(&encoderMux);

    if (digitalRead(RIGHT_ENCODER_A) ==
        digitalRead(RIGHT_ENCODER_B)) {
        _rightTicks++;
    } else {
        _rightTicks--;
    }

    portEXIT_CRITICAL_ISR(&encoderMux);
}

long EncoderReader::getLeftTicks() {
    portENTER_CRITICAL(&encoderMux);
    long ticks = _leftTicks;
    portEXIT_CRITICAL(&encoderMux);

    return ticks;
}

long EncoderReader::getRightTicks() {
    portENTER_CRITICAL(&encoderMux);
    long ticks = _rightTicks;
    portEXIT_CRITICAL(&encoderMux);

    return ticks;
}

void EncoderReader::reset() {
    portENTER_CRITICAL(&encoderMux);
    _leftTicks = 0;
    _rightTicks = 0;
    portEXIT_CRITICAL(&encoderMux);
}
