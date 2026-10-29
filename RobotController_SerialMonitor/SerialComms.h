
#ifndef SERIAL_COMMS_H
#define SERIAL_COMMS_H

#include <Arduino.h>

class SerialComms {
public:
    // Initialize UART communication.
    void begin();

    // Read incoming commands without blocking.
    void update();

    // Get the latest target wheel velocities in m/s.
    float getLeftVelocity() const;
    float getRightVelocity() const;

    // Returns false if no valid command arrived recently.
    bool isCommandFresh() const;

private:
    HardwareSerial* _serial = nullptr;

    float _leftVelocity = 0.0f;
    float _rightVelocity = 0.0f;

    char _buffer[64] = {};
    size_t _bufferIndex = 0;

    uint32_t _lastCommandMs = 0;
    bool _hasCommand = false;

    void processLine();
};

#endif
