#include "EncoderReader.h"
#include "Config.h"

// Shared tick counters updated by interrupts.
volatile long EncoderReader::_leftTicks = 0;
volatile long EncoderReader::_rightTicks = 0;

volatile bool EncoderReader::_leftCommandedForward = true;
volatile bool EncoderReader::_rightCommandedForward = true;

volatile uint32_t EncoderReader::_leftTransitionsA = 0;
volatile uint32_t EncoderReader::_leftTransitionsB = 0;
volatile uint32_t EncoderReader::_rightTransitionsA = 0;
volatile uint32_t EncoderReader::_rightTransitionsB = 0;

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
        digitalPinToInterrupt(LEFT_ENCODER_B),
        leftEncoderB_ISR,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(RIGHT_ENCODER_A),
        rightEncoderISR,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(RIGHT_ENCODER_B),
        rightEncoderB_ISR,
        CHANGE
    );
}

void IRAM_ATTR EncoderReader::leftEncoderB_ISR() {
    _leftTransitionsB++;
}

void IRAM_ATTR EncoderReader::rightEncoderB_ISR() {
    _rightTransitionsB++;
}

void IRAM_ATTR EncoderReader::leftEncoderISR() {
    portENTER_CRITICAL_ISR(&encoderMux);
    _leftTransitionsA++;

    // Check if Phase B is healthy and actively toggling.
    // If Phase B wire is disconnected or dead, _leftTransitionsB will be 0 or frozen.
    bool phaseBHealthy = (_leftTransitionsB > 0 && (_leftTransitionsA <= _leftTransitionsB * 3));

    bool forward;
    if (phaseBHealthy) {
        // Full dual-channel quadrature decoding
        forward = (digitalRead(LEFT_ENCODER_A) == digitalRead(LEFT_ENCODER_B));
        if (INVERT_LEFT_ENCODER) {
            forward = !forward;
        }
    } else {
        // Automatic Fallback: Phase B is not toggling.
        // Use commanded motor direction to prevent quadrature cancellation!
        forward = _leftCommandedForward;
    }

    if (forward) {
        _leftTicks++;
    } else {
        _leftTicks--;
    }

    portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR EncoderReader::rightEncoderISR() {
    portENTER_CRITICAL_ISR(&encoderMux);
    _rightTransitionsA++;

    bool phaseBHealthy = (_rightTransitionsB > 0 && (_rightTransitionsA <= _rightTransitionsB * 3));

    bool forward;
    if (phaseBHealthy) {
        forward = (digitalRead(RIGHT_ENCODER_A) == digitalRead(RIGHT_ENCODER_B));
        if (INVERT_RIGHT_ENCODER) {
            forward = !forward;
        }
    } else {
        forward = _rightCommandedForward;
    }

    if (forward) {
        _rightTicks++;
    } else {
        _rightTicks--;
    }

    portEXIT_CRITICAL_ISR(&encoderMux);
}

void EncoderReader::setMotorDirections(bool leftForward, bool rightForward) {
    portENTER_CRITICAL(&encoderMux);
    _leftCommandedForward = leftForward;
    _rightCommandedForward = rightForward;
    portEXIT_CRITICAL(&encoderMux);
}

uint32_t EncoderReader::getLeftTransitionsA() {
    return _leftTransitionsA;
}

uint32_t EncoderReader::getLeftTransitionsB() {
    return _leftTransitionsB;
}

bool EncoderReader::isLeftPhaseBHealthy() {
    return (_leftTransitionsB > 0 && (_leftTransitionsA <= _leftTransitionsB * 3));
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
    _leftTransitionsA = 0;
    _leftTransitionsB = 0;
    _rightTransitionsA = 0;
    _rightTransitionsB = 0;
    portEXIT_CRITICAL(&encoderMux);
}
