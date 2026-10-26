
#include "SerialComms.h"
#include "Config.h"

#include <stdlib.h>
#include <math.h>

void SerialComms::begin() {
    // UART1 communicates with the Raspberry Pi.
    _serial = &Serial1;

    _serial->begin(
        UART_BAUD_RATE,
        SERIAL_8N1,
        UART_RX_PIN,
        UART_TX_PIN
    );

    _leftVelocity = 0.0f;
    _rightVelocity = 0.0f;

    _bufferIndex = 0;
    _lastCommandMs = millis();
    _hasCommand = false;
}

void SerialComms::update() {
    if (_serial == nullptr) {
        return;
    }

    while (_serial->available() > 0) {
        char c = static_cast<char>(_serial->read());

        if (c == '\n') {
            _buffer[_bufferIndex] = '\0';
            processLine();
            _bufferIndex = 0;
        }
        else if (c != '\r') {
            if (_bufferIndex < sizeof(_buffer) - 1) {
                _buffer[_bufferIndex++] = c;
            }
            else {
                // Discard an oversized or malformed line.
                _bufferIndex = 0;
            }
        }
    }

    // Stop accepting stale velocity commands.
    if (!isCommandFresh()) {
        _leftVelocity = 0.0f;
        _rightVelocity = 0.0f;
    }
}

void SerialComms::processLine() {
    char* comma = strchr(_buffer, ',');

    if (comma == nullptr) {
        return;
    }

    *comma = '\0';

    char* leftEnd = nullptr;
    char* rightEnd = nullptr;

    float left = strtof(_buffer, &leftEnd);
    float right = strtof(comma + 1, &rightEnd);

    // Reject empty or malformed numbers.
    if (leftEnd == _buffer ||
        rightEnd == comma + 1 ||
        *leftEnd != '\0' ||
        *rightEnd != '\0') {
        return;
    }

    if (!isfinite(left) || !isfinite(right)) {
        return;
    }

    // Basic command limit: adjust for your robot.
    if (fabsf(left) > 2.0f || fabsf(right) > 2.0f) {
        return;
    }

    _leftVelocity = left;
    _rightVelocity = right;

    _lastCommandMs = millis();
    _hasCommand = true;
}

float SerialComms::getLeftVelocity() const {
    return _leftVelocity;
}

float SerialComms::getRightVelocity() const {
    return _rightVelocity;
}

bool SerialComms::isCommandFresh() const {
    return _hasCommand &&
           (uint32_t)(millis() - _lastCommandMs)
               <= COMMAND_TIMEOUT_MS;
}
