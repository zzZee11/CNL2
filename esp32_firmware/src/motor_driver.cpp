#include "motor_driver.h"

#if defined(ESP_ARDUINO_VERSION_VAL)
  #define IS_ESP_CORE_3 (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
#else
  #define IS_ESP_CORE_3 0
#endif

MotorDriver::MotorDriver()
    : _currentState(STATE_STOP), _currentSpeed(0) {}

void MotorDriver::init() {
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN3, OUTPUT);
    pinMode(PIN_MOTOR_R_IN4, OUTPUT);

    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);
    digitalWrite(PIN_MOTOR_R_IN3, LOW);
    digitalWrite(PIN_MOTOR_R_IN4, LOW);

#if IS_ESP_CORE_3
    ledcAttach(PIN_MOTOR_L_ENA, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(PIN_MOTOR_R_ENB, PWM_FREQUENCY, PWM_RESOLUTION);
#else
    ledcSetup(PWM_CHANNEL_L, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_L_ENA, PWM_CHANNEL_L);
    ledcSetup(PWM_CHANNEL_R, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_R_ENB, PWM_CHANNEL_R);
#endif

    stop();
}

void MotorDriver::_writeLeftPwm(uint8_t pwm) {
#if IS_ESP_CORE_3
    ledcWrite(PIN_MOTOR_L_ENA, pwm);
#else
    ledcWrite(PWM_CHANNEL_L, pwm);
#endif
}

void MotorDriver::_writeRightPwm(uint8_t pwm) {
#if IS_ESP_CORE_3
    ledcWrite(PIN_MOTOR_R_ENB, pwm);
#else
    ledcWrite(PWM_CHANNEL_R, pwm);
#endif
}

void MotorDriver::setCommand(DriveState state, uint8_t speed) {
    _currentState = state;
    _currentSpeed = speed;

    switch (state) {
        case STATE_FWD:
            setRawMotors((int16_t)speed, (int16_t)speed);
            break;

        case STATE_REV:
            setRawMotors(-(int16_t)speed, -(int16_t)speed);
            break;

        case STATE_LEFT:
#if TURN_A_DRIVES_LEFT_WHEEL
            setRawMotors((int16_t)speed, 0);
#else
            setRawMotors(0, (int16_t)speed);
#endif
            break;

        case STATE_RIGHT:
#if TURN_A_DRIVES_LEFT_WHEEL
            setRawMotors(0, (int16_t)speed);
#else
            setRawMotors((int16_t)speed, 0);
#endif
            break;

        case STATE_FWD_LEFT:
            // Arc forward-left: Right motor full forward, Left motor reduced forward
            setRawMotors((int16_t)(speed / 3), (int16_t)speed);
            break;

        case STATE_FWD_RIGHT:
            // Arc forward-right: Left motor full forward, Right motor reduced forward
            setRawMotors((int16_t)speed, (int16_t)(speed / 3));
            break;

        case STATE_REV_LEFT:
            // Arc reverse-left
            setRawMotors(-(int16_t)(speed / 3), -(int16_t)speed);
            break;

        case STATE_REV_RIGHT:
            // Arc reverse-right
            setRawMotors(-(int16_t)speed, -(int16_t)(speed / 3));
            break;

        case STATE_BRAKE:
            brake();
            break;

        case STATE_STOP:
        default:
            stop();
            break;
    }
}

void MotorDriver::setRawMotors(int16_t leftPwm, int16_t rightPwm) {
    if (INVERT_LEFT_MOTOR)  leftPwm = -leftPwm;
    if (INVERT_RIGHT_MOTOR) rightPwm = -rightPwm;

    // Left motor
    if (leftPwm > 0) {
        digitalWrite(PIN_MOTOR_L_IN1, HIGH);
        digitalWrite(PIN_MOTOR_L_IN2, LOW);
        _writeLeftPwm((uint8_t)min(255, (int)leftPwm));
    } else if (leftPwm < 0) {
        digitalWrite(PIN_MOTOR_L_IN1, LOW);
        digitalWrite(PIN_MOTOR_L_IN2, HIGH);
        _writeLeftPwm((uint8_t)min(255, (int)(-leftPwm)));
    } else {
        digitalWrite(PIN_MOTOR_L_IN1, LOW);
        digitalWrite(PIN_MOTOR_L_IN2, LOW);
        _writeLeftPwm(0);
    }

    // Right motor
    if (rightPwm > 0) {
        digitalWrite(PIN_MOTOR_R_IN3, HIGH);
        digitalWrite(PIN_MOTOR_R_IN4, LOW);
        _writeRightPwm((uint8_t)min(255, (int)rightPwm));
    } else if (rightPwm < 0) {
        digitalWrite(PIN_MOTOR_R_IN3, LOW);
        digitalWrite(PIN_MOTOR_R_IN4, HIGH);
        _writeRightPwm((uint8_t)min(255, (int)(-rightPwm)));
    } else {
        digitalWrite(PIN_MOTOR_R_IN3, LOW);
        digitalWrite(PIN_MOTOR_R_IN4, LOW);
        _writeRightPwm(0);
    }
}

void MotorDriver::stop() {
    _currentState = STATE_STOP;
    _currentSpeed = 0;
    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);
    digitalWrite(PIN_MOTOR_R_IN3, LOW);
    digitalWrite(PIN_MOTOR_R_IN4, LOW);
    _writeLeftPwm(0);
    _writeRightPwm(0);
}

void MotorDriver::brake() {
    _currentState = STATE_BRAKE;
    _currentSpeed = 0;
    digitalWrite(PIN_MOTOR_L_IN1, HIGH);
    digitalWrite(PIN_MOTOR_L_IN2, HIGH);
    digitalWrite(PIN_MOTOR_R_IN3, HIGH);
    digitalWrite(PIN_MOTOR_R_IN4, HIGH);
    _writeLeftPwm(255);
    _writeRightPwm(255);
}

const char* MotorDriver::getStateString() const {
    switch (_currentState) {
        case STATE_FWD:       return "FWD";
        case STATE_REV:       return "REV";
        case STATE_LEFT:      return "LEFT";
        case STATE_RIGHT:     return "RIGHT";
        case STATE_FWD_LEFT:  return "FWD_L";
        case STATE_FWD_RIGHT: return "FWD_R";
        case STATE_REV_LEFT:  return "REV_L";
        case STATE_REV_RIGHT: return "REV_R";
        case STATE_BRAKE:     return "BRAKE";
        case STATE_STOP:
        default:              return "STOP";
    }
}
