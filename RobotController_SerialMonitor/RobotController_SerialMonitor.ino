#include <Arduino.h>

#include "Config.h"
#include "MotorController.h"
#include "EncoderReader.h"
#include "WheelSpeedPID.h"
#include "SerialComms.h"

// Create module objects
MotorController motors;
EncoderReader encoders;
WheelSpeedPID speedPID;
SerialComms comms;

// ============================================================================
// USB SERIAL MONITOR SPEED CONTROL & TELEMETRY
// ============================================================================
static float serialCmdLeftSpeed  = 0.0f;  // Commanded left velocity in m/s
static float serialCmdRightSpeed = 0.0f;  // Commanded right velocity in m/s
static uint32_t lastSerialCmdMs  = 0;

// Set to true if you want the robot to auto-stop after inactivity timeout,
// or false if you want it to maintain the set speed until you send a new command.
constexpr bool ENABLE_SERIAL_WATCHDOG = false;
constexpr uint32_t SERIAL_CMD_TIMEOUT_MS = 4000;

// Buffer for parsing lines from Serial Monitor
static char usbBuffer[64];
static size_t usbBufferIndex = 0;

void printHelpBanner() {
    Serial.println("\n========================================================");
    Serial.println("  ESP32 ROBOT CONTROLLER: SERIAL MONITOR SPEED TEST");
    Serial.println("========================================================");
    Serial.println("Send wheel speeds via Serial Monitor in m/s: <left>,<right>");
    Serial.println("Examples:");
    Serial.println("  0.20,0.20   -> Forward at 0.20 m/s");
    Serial.println("  0.00,0.00   -> Stop motors");
    Serial.println("  stop        -> Quick halt");
    Serial.println("  diag        -> Run hardware encoder pin diagnostic");
    Serial.println("  0.15,-0.15  -> Spin clockwise (right)");
    Serial.println(" -0.15,0.15   -> Spin counter-clockwise (left)");
    Serial.println("  0.30,0.15   -> Curve forward-right");
    Serial.println("Ensure line ending is set to 'Newline' @ 115200 baud.");
    Serial.println("========================================================\n");
}

/**
 * Non-blocking line reader that parses incoming speed commands from USB Serial.
 */
void readSpeedsFromSerialMonitor() {
    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());

        if (c == '\n' || c == '\r') {
            if (usbBufferIndex > 0) {
                usbBuffer[usbBufferIndex] = '\0';

                // Handle "stop" or "s" quick halt
                if (strcasecmp(usbBuffer, "stop") == 0 || strcasecmp(usbBuffer, "s") == 0) {
                    serialCmdLeftSpeed  = 0.0f;
                    serialCmdRightSpeed = 0.0f;
                    lastSerialCmdMs     = millis();
                    Serial.println("\n>>> [STOP] Motors Halted <<<");
                }
                // Handle "diag" or "d" pin diagnostic
                else if (strcasecmp(usbBuffer, "diag") == 0 || strcasecmp(usbBuffer, "d") == 0) {
                    Serial.println("\n--- ENCODER HARDWARE PIN DIAGNOSTIC ---");
                    Serial.printf("Left Encoder  -> Pin 1 (Phase A): %d (edges: %u) | Pin 2 (Phase B): %d (edges: %u)\n",
                                  digitalRead(LEFT_ENCODER_A), EncoderReader::getLeftTransitionsA(),
                                  digitalRead(LEFT_ENCODER_B), EncoderReader::getLeftTransitionsB());
                    Serial.printf("Right Encoder -> Pin 12 (Phase A): %d | Pin 13 (Phase B): %d\n",
                                  digitalRead(RIGHT_ENCODER_A), digitalRead(RIGHT_ENCODER_B));
                    Serial.printf("Left Mode: %s\n",
                                  EncoderReader::isLeftPhaseBHealthy() ? "DUAL-CHANNEL QUADRATURE (Healthy)" : "DIRECTION-ASSIST FALLBACK (Phase B not pulsing)");
                    Serial.println("----------------------------------------\n");
                }
                // Handle "help" or "h"
                else if (strcasecmp(usbBuffer, "help") == 0 || strcasecmp(usbBuffer, "h") == 0) {
                    printHelpBanner();
                }
                // Parse comma-separated speed values: "<left>,<right>"
                else {
                    char* comma = strchr(usbBuffer, ',');
                    if (comma != nullptr) {
                        *comma = '\0';
                        char* leftEnd = nullptr;
                        char* rightEnd = nullptr;

                        float leftVal  = strtof(usbBuffer, &leftEnd);
                        float rightVal = strtof(comma + 1, &rightEnd);

                        // Validate numbers and sanity-check limits (+/- 1.5 m/s)
                        if (leftEnd != usbBuffer && rightEnd != (comma + 1) &&
                            isfinite(leftVal) && isfinite(rightVal) &&
                            fabsf(leftVal) <= 1.5f && fabsf(rightVal) <= 1.5f) {

                            serialCmdLeftSpeed  = leftVal;
                            serialCmdRightSpeed = rightVal;
                            lastSerialCmdMs     = millis();

                            Serial.printf("\n>>> [NEW TARGET] Left: %+.2f m/s | Right: %+.2f m/s <<<\n",
                                          serialCmdLeftSpeed, serialCmdRightSpeed);
                        } else {
                            Serial.println("\n[!] Invalid format. Use: <left_m_s>,<right_m_s> (e.g. 0.20,0.20)");
                        }
                    } else {
                        Serial.println("\n[!] Missing comma. Format: <left_m_s>,<right_m_s> (e.g. 0.20,0.20)");
                    }
                }
                usbBufferIndex = 0;
            }
        } else {
            if (usbBufferIndex < sizeof(usbBuffer) - 1) {
                usbBuffer[usbBufferIndex++] = c;
            } else {
                usbBufferIndex = 0; // Overflow discard
            }
        }
    }

    // Optional inactivity watchdog
    if (ENABLE_SERIAL_WATCHDOG && (millis() - lastSerialCmdMs > SERIAL_CMD_TIMEOUT_MS)) {
        serialCmdLeftSpeed  = 0.0f;
        serialCmdRightSpeed = 0.0f;
    }
}

/**
 * Periodically prints both commanded and measured wheel speeds (in m/s and RPM)
 * plus PWM drive values to the Serial Monitor at 5 Hz (every 200 ms).
 */
void printSpeedTelemetry(float targetLeftRPM, float targetRightRPM,
                         float actualLeftRPM, float actualRightRPM,
                         int leftPWM, int rightPWM) {
    static uint32_t lastPrintMs = 0;

    if (millis() - lastPrintMs >= 200) {
        lastPrintMs = millis();

        // Convert RPM to linear velocity in m/s: v = (RPM * 2 * PI * R) / 60
        const float rpmToMps = (2.0f * PI * WHEEL_RADIUS_M) / 60.0f;
        float actualLeftMps  = actualLeftRPM  * rpmToMps;
        float actualRightMps = actualRightRPM * rpmToMps;
        float targetLeftMps  = targetLeftRPM  * rpmToMps;
        float targetRightMps = targetRightRPM * rpmToMps;
        const char* lMode    = EncoderReader::isLeftPhaseBHealthy() ? "2X-QUAD" : "1X-ASSIST";

        Serial.printf("[TARGET] L:%+.2fm/s (%+5.1f RPM) R:%+.2fm/s (%+5.1f RPM) | "
                      "[ACTUAL] L:%+.2fm/s (%+5.1f RPM) R:%+.2fm/s (%+5.1f RPM) | "
                      "PWM:[%4d,%4d] [%s]\n",
                      targetLeftMps, targetLeftRPM, targetRightMps, targetRightRPM,
                      actualLeftMps, actualLeftRPM, actualRightMps, actualRightRPM,
                      leftPWM, rightPWM, lMode);
    }
}

// ============================================================================
// MAIN SETUP
// ============================================================================
void setup() {
    // USB Serial Monitor initialization
    Serial.begin(115200);
    delay(500);

    printHelpBanner();

    // Initialize motor controller
    if (!motors.begin()) {
        Serial.println("ERROR: Motor initialization failed!");
        while (true) {
            delay(1000);
        }
    }

    // Initialize wheel encoders
    encoders.begin();

    // Initialize PID controller
    speedPID.begin(
        TICKS_PER_REVOLUTION,
        PID_SAMPLE_TIME_MS
    );

    speedPID.setGains(
        PID_KP,
        PID_KI,
        PID_KD
    );

    // Synchronize PID encoder baselines
    speedPID.reset(
        encoders.getLeftTicks(),
        encoders.getRightTicks()
    );

    // Also initialize hardware UART pins for fallback / Pi link
    comms.begin();

    // Start with both motors safely stopped
    motors.stop();

    Serial.println("Robot controller initialized and ready for Serial commands.");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
    // 1. Check for incoming speed commands from USB Serial Monitor
    readSpeedsFromSerialMonitor();

    // 2. Also check hardware UART (Serial1) if Pi is communicating
    comms.update();

    // 3. Determine target velocities:
    // If USB Serial commands have been sent, use them; otherwise fallback to Pi UART if fresh
    float targetLeftMps  = 0.0f;
    float targetRightMps = 0.0f;

    if (serialCmdLeftSpeed != 0.0f || serialCmdRightSpeed != 0.0f || (millis() - lastSerialCmdMs < 1000)) {
        // Active USB Serial Monitor command
        targetLeftMps  = serialCmdLeftSpeed;
        targetRightMps = serialCmdRightSpeed;
    } else if (comms.isCommandFresh()) {
        // Pi UART hardware command
        targetLeftMps  = comms.getLeftVelocity();
        targetRightMps = comms.getRightVelocity();
    }

    // 4. Feed commanded motor direction to encoder reader for direction-assist fallback
    EncoderReader::setMotorDirections(targetLeftMps >= 0.0f, targetRightMps >= 0.0f);

    // 5. Convert linear speeds (m/s) to target wheel RPM
    const float mpsToRpm = 60.0f / (2.0f * PI * WHEEL_RADIUS_M);
    float targetLeftRPM  = targetLeftMps  * mpsToRpm;
    float targetRightRPM = targetRightMps * mpsToRpm;

    // 6. Read current encoder counts
    long leftTicks  = encoders.getLeftTicks();
    long rightTicks = encoders.getRightTicks();

    // 7. Update PID loop at configured sample rate (50 ms / 20 Hz)
    if (speedPID.update(
            targetLeftRPM,
            targetRightRPM,
            leftTicks,
            rightTicks
        )) {

        // 8. Command motor PWM outputs
        int leftPWM  = static_cast<int>(speedPID.getLeftOutput());
        int rightPWM = static_cast<int>(speedPID.getRightOutput());
        motors.setSpeeds(leftPWM, rightPWM);

        // 9. Print clean speed telemetry to USB Serial Monitor
        printSpeedTelemetry(targetLeftRPM, targetRightRPM,
                            speedPID.getLeftRPM(), speedPID.getRightRPM(),
                            leftPWM, rightPWM);

        // 10. Send encoder telemetry packet to Pi UART (Serial1)
        Serial1.print("ENC,");
        Serial1.print(encoders.getLeftTicks());
        Serial1.print(",");
        Serial1.print(encoders.getRightTicks());
        Serial1.print(",");
        Serial1.print(speedPID.getLeftRPM(), 1);
        Serial1.print(",");
        Serial1.println(speedPID.getRightRPM(), 1);
    }
}
