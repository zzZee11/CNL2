
#include "MotorController.h"
#include "Config.h"

bool MotorController::begin() {
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);

    // Configure PWM using the Arduino-ESP32 Core 3.x API.
    bool leftPWM = ledcAttach(
        LEFT_MOTOR_PWM,
        MOTOR_PWM_FREQUENCY,
        MOTOR_PWM_RESOLUTION
    );

    bool rightPWM = ledcAttach(
        RIGHT_MOTOR_PWM,
        MOTOR_PWM_FREQUENCY,
        MOTOR_PWM_RESOLUTION
    );

    stop();

    return leftPWM && rightPWM;
}

void MotorController::setSpeeds(int leftSpeed, int rightSpeed) {
    if (INVERT_LEFT_MOTOR) {
        leftSpeed = -leftSpeed;
    }
    if (INVERT_RIGHT_MOTOR) {
        rightSpeed = -rightSpeed;
    }

    setMotor(
        LEFT_MOTOR_IN1,
        LEFT_MOTOR_IN2,
        LEFT_MOTOR_PWM,
        leftSpeed
    );

    setMotor(
        RIGHT_MOTOR_IN1,
        RIGHT_MOTOR_IN2,
        RIGHT_MOTOR_PWM,
        rightSpeed
    );
}

void MotorController::setMotor(
    int in1,
    int in2,
    int pwmPin,
    int speed
) {
    speed = constrain(speed, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);

    if (speed > 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } 
    else if (speed < 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } 
    else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }

    // PWM duty must be non-negative.
    ledcWrite(pwmPin, abs(speed));
}

void MotorController::stop() {
    setSpeeds(0, 0);
}
