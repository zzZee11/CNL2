#include <Arduino.h>
#include <string.h>
#include <ctype.h>
#include "config.h"
#include "motor_driver.h"
#include "encoder.h"
#include "pid_controller.h"

// Subsystem instances
MotorDriver motors;
EncoderManager encoders;
RobotController controller(motors);

// Watchdog & Timing
unsigned long lastCommandTime = 0;
unsigned long lastTelemetryTime = 0;
bool watchdogTripped = true;

// Incoming serial buffer
static char rxBuffer[64];
static uint8_t rxIndex = 0;

void processCommand(const char* cmdStr) {
    // Expected format: CMD:<DIR>[,<SPEED>]
    // Examples: "CMD:W,200", "CMD:WA,200", "CMD:WD,200", "CMD:S,180", "CMD:X,0", "CMD:B,0"
    if (strncmp(cmdStr, "CMD:", 4) != 0) {
        return;
    }

    const char* payload = cmdStr + 4;
    uint8_t speed = DEFAULT_DRIVE_SPEED;

    // Parse direction code (e.g. "W", "WA", "WD", "SA", "SD", "A", "D", "X", "B")
    char dirCode[8];
    const char* commaPtr = strchr(payload, ',');
    if (commaPtr != nullptr) {
        size_t len = commaPtr - payload;
        if (len >= sizeof(dirCode)) len = sizeof(dirCode) - 1;
        strncpy(dirCode, payload, len);
        dirCode[len] = '\0';

        int parsedSpeed = atoi(commaPtr + 1);
        speed = (uint8_t)constrain(parsedSpeed, 0, 255);
    } else {
        strncpy(dirCode, payload, sizeof(dirCode) - 1);
        dirCode[sizeof(dirCode) - 1] = '\0';
    }

    // Convert direction code to uppercase
    for (int i = 0; dirCode[i]; i++) {
        dirCode[i] = toupper((unsigned char)dirCode[i]);
    }

    // Reset watchdog timer
    lastCommandTime = millis();
    watchdogTripped = false;

    // Direct differential drive command from state machine: CMD:LR,<leftPwm>,<rightPwm>
    if (strcmp(dirCode, "LR") == 0 && commaPtr != nullptr) {
        int pwmL = atoi(commaPtr + 1);
        const char* secondComma = strchr(commaPtr + 1, ',');
        int pwmR = (secondComma != nullptr) ? atoi(secondComma + 1) : pwmL;
        pwmL = constrain(pwmL, -255, 255);
        pwmR = constrain(pwmR, -255, 255);
        controller.setRawMotors((int16_t)pwmL, (int16_t)pwmR);
        return;
    }

    // Map command codes to controller states
    if (strcmp(dirCode, "WA") == 0) {
        controller.setCommand(STATE_FWD_LEFT, speed);
    } else if (strcmp(dirCode, "WD") == 0) {
        controller.setCommand(STATE_FWD_RIGHT, speed);
    } else if (strcmp(dirCode, "SA") == 0) {
        controller.setCommand(STATE_REV_LEFT, speed);
    } else if (strcmp(dirCode, "SD") == 0) {
        controller.setCommand(STATE_REV_RIGHT, speed);
    } else if (strcmp(dirCode, "W") == 0) {
        controller.setCommand(STATE_FWD, speed);
    } else if (strcmp(dirCode, "S") == 0) {
        controller.setCommand(STATE_REV, speed);
    } else if (strcmp(dirCode, "A") == 0) {
        controller.setCommand(STATE_LEFT, (speed > 0) ? speed : DEFAULT_TURN_SPEED);
    } else if (strcmp(dirCode, "D") == 0) {
        controller.setCommand(STATE_RIGHT, (speed > 0) ? speed : DEFAULT_TURN_SPEED);
    } else if (strcmp(dirCode, "B") == 0) {
        controller.setCommand(STATE_BRAKE, 0);
    } else if (strcmp(dirCode, "X") == 0) {
        controller.setCommand(STATE_STOP, 0);
    } else {
        controller.setCommand(STATE_STOP, 0);
    }
}

void readSerialInput() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        if (c == '\n' || c == '\r') {
            if (rxIndex > 0) {
                rxBuffer[rxIndex] = '\0';
                processCommand(rxBuffer);
                rxIndex = 0;
            }
        } else if (rxIndex < sizeof(rxBuffer) - 1) {
            rxBuffer[rxIndex++] = c;
        } else {
            rxIndex = 0;
        }
    }
}

void checkWatchdog(unsigned long now) {
    if (!watchdogTripped && (now - lastCommandTime > WATCHDOG_TIMEOUT_MS)) {
        controller.setCommand(STATE_STOP, 0);
        watchdogTripped = true;
    }
}

void sendTelemetry(unsigned long now) {
    if (now - lastTelemetryTime < TELEMETRY_INTERVAL_MS) {
        return;
    }
    lastTelemetryTime = now;

    float rpmL = encoders.getRpmLeft();
    float rpmR = encoders.getRpmRight();
    int64_t ticksL = encoders.getTicksLeft();
    int64_t ticksR = encoders.getTicksRight();
    const char* stateStr = motors.getStateString();
    const char* flagsStr = watchdogTripped ? "WDG" : "OK";

    // Format: TELE:<rpmL>,<rpmR>,<ticksL>,<ticksR>,<state>,<flags>
    Serial.printf("TELE:%.1f,%.1f,%lld,%lld,%s,%s\n",
                  rpmL, rpmR,
                  (long long)ticksL, (long long)ticksR,
                  stateStr, flagsStr);
}

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial && millis() < 1000);

    motors.init();
    encoders.init();
    controller.init();

    lastCommandTime = millis();
    lastTelemetryTime = millis();
    watchdogTripped = true;

    Serial.println("# ESP32-S3 Multi-Key Robot Car Controller Ready");
}

void loop() {
    unsigned long now = millis();

    // 1. Process incoming commands
    readSerialInput();

    // 2. Safety watchdog check
    checkWatchdog(now);

    // 3. Encoder calculation & closed-loop PID control
    encoders.update(now);
    controller.update(now,
                      encoders.getRpmLeft(),
                      encoders.getRpmRight(),
                      encoders.getTicksLeft(),
                      encoders.getTicksRight());

    // 4. Emit real-time telemetry stream
    sendTelemetry(now);
}
