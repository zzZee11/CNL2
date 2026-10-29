
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

class MotorController {
public:
    // Initialize motor direction pins and PWM.
    bool begin();

    // Set left and right motor PWM commands (-255 to 255).
    // Positive = forward, negative = reverse, zero = stop.
    void setSpeeds(int leftSpeed, int rightSpeed);

    // Stop both motors.
    void stop();

private:
    void setMotor(int in1, int in2, int pwmPin, int speed);
};

#endif
