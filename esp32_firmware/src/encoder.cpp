#include "encoder.h"

// Define static members
volatile int64_t EncoderManager::_ticksLeft = 0;
volatile int64_t EncoderManager::_ticksRight = 0;
portMUX_TYPE EncoderManager::_mux = portMUX_INITIALIZER_UNLOCKED;

// Glitch filter: 100 microseconds debounce window filters out 20 kHz PWM inductive noise
// while allowing accurate pulse counting up to 1,800 RPM (well above 200 RPM motor max).
static volatile unsigned long _lastMicrosLeft = 0;
static volatile unsigned long _lastMicrosRight = 0;
#define ENCODER_DEBOUNCE_US 100

EncoderManager::EncoderManager()
    : _prevTicksLeft(0), _prevTicksRight(0), _lastCalcTime(0),
      _rpmLeft(0.0f), _rpmRight(0.0f) {}

void EncoderManager::init() {
    // Configure encoder input pins with internal pullups for noise immunity
    pinMode(PIN_ENC_L_A, INPUT_PULLUP);
    pinMode(PIN_ENC_L_B, INPUT_PULLUP);
    pinMode(PIN_ENC_R_A, INPUT_PULLUP);
    pinMode(PIN_ENC_R_B, INPUT_PULLUP);

    // Attach hardware interrupts on Phase A pins
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRight, RISING);

    reset();
}

void IRAM_ATTR EncoderManager::isrLeft() {
    unsigned long now = micros();
    if (now - _lastMicrosLeft < ENCODER_DEBOUNCE_US) {
        return; // Noise / inductive PWM bounce rejection
    }
    _lastMicrosLeft = now;

    portENTER_CRITICAL_ISR(&_mux);
    // If Phase B is LOW on Phase A rising edge, wheel is rotating forward
    bool dir = digitalRead(PIN_ENC_L_B);
    if (INVERT_LEFT_ENCODER) dir = !dir;

    if (dir) {
        _ticksLeft++;
    } else {
        _ticksLeft--;
    }
    portEXIT_CRITICAL_ISR(&_mux);
}

void IRAM_ATTR EncoderManager::isrRight() {
    unsigned long now = micros();
    if (now - _lastMicrosRight < ENCODER_DEBOUNCE_US) {
        return; // Noise / inductive PWM bounce rejection
    }
    _lastMicrosRight = now;

    portENTER_CRITICAL_ISR(&_mux);
    // If Phase B is LOW on Phase A rising edge, wheel is rotating forward
    bool dir = digitalRead(PIN_ENC_R_B);
    if (INVERT_RIGHT_ENCODER) dir = !dir;

    if (dir) {
        _ticksRight++;
    } else {
        _ticksRight--;
    }
    portEXIT_CRITICAL_ISR(&_mux);
}

int64_t EncoderManager::getTicksLeft() {
    portENTER_CRITICAL(&_mux);
    int64_t val = _ticksLeft;
    portEXIT_CRITICAL(&_mux);
    return val;
}

int64_t EncoderManager::getTicksRight() {
    portENTER_CRITICAL(&_mux);
    int64_t val = _ticksRight;
    portEXIT_CRITICAL(&_mux);
    return val;
}

void EncoderManager::reset() {
    portENTER_CRITICAL(&_mux);
    _ticksLeft = 0;
    _ticksRight = 0;
    portEXIT_CRITICAL(&_mux);

    _prevTicksLeft = 0;
    _prevTicksRight = 0;
    _lastCalcTime = millis();
    _lastMicrosLeft = 0;
    _lastMicrosRight = 0;
    _rpmLeft = 0.0f;
    _rpmRight = 0.0f;
}

void EncoderManager::update(unsigned long nowMs) {
    unsigned long dt = nowMs - _lastCalcTime;
    if (dt < RPM_CALC_INTERVAL_MS) {
        return;
    }

    int64_t curLeft = getTicksLeft();
    int64_t curRight = getTicksRight();

    int64_t deltaLeft = curLeft - _prevTicksLeft;
    int64_t deltaRight = curRight - _prevTicksRight;

    _prevTicksLeft = curLeft;
    _prevTicksRight = curRight;
    _lastCalcTime = nowMs;

    // RPM formula: (deltaTicks / WHEEL_PULSES_PER_REV) * (60,000 / dt_ms)
    float rawRpmL = ((float)deltaLeft / (float)WHEEL_PULSES_PER_REV) * (60000.0f / (float)dt);
    float rawRpmR = ((float)deltaRight / (float)WHEEL_PULSES_PER_REV) * (60000.0f / (float)dt);

    // Apply exponential moving average filter (alpha = 0.6) to smooth encoder jitter
    const float alpha = 0.6f;
    _rpmLeft = (alpha * rawRpmL) + ((1.0f - alpha) * _rpmLeft);
    _rpmRight = (alpha * rawRpmR) + ((1.0f - alpha) * _rpmRight);
}

