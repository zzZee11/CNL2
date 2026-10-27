
#include <Arduino.h>

#include "Config.h"
#include "MotorController.h"
#include "EncoderReader.h"
#include "WheelSpeedPID.h"
#include "SerialComms.h"

// Create module objects.
MotorController motors;
EncoderReader encoders;
WheelSpeedPID speedPID;
SerialComms comms;

void setup() {
    // USB serial for debugging.
    Serial.begin(115200);

    // Initialize motor controller.
    if (!motors.begin()) {
        Serial.println("ERROR: Motor initialization failed!");

        while (true) {
            delay(1000);
        }
    }

    // Initialize wheel encoders.
    encoders.begin();

    // Initialize PID controller.
    speedPID.begin(
        TICKS_PER_REVOLUTION,
        PID_SAMPLE_TIME_MS
    );

    speedPID.setGains(
        PID_KP,
        PID_KI,
        PID_KD
    );

    // Synchronize PID encoder baselines.
    speedPID.reset(
        encoders.getLeftTicks(),
        encoders.getRightTicks()
    );

    // Initialize UART communication with Raspberry Pi.
    comms.begin();

    // Start with both motors stopped.
    motors.stop();

    Serial.println("Robot controller initialized.");
}

void loop() {
    // 1. Receive commands from Raspberry Pi.
    comms.update();

    // 2. Default targets to zero.
    float targetLeftRPM = 0.0f;
    float targetRightRPM = 0.0f;

    // 3. Convert commanded wheel velocities from m/s to RPM.
    // If communication times out, keep both targets at zero.
    if (comms.isCommandFresh()) {
        targetLeftRPM =
            comms.getLeftVelocity() * 60.0f
            / (2.0f * PI * WHEEL_RADIUS_M);

        targetRightRPM =
            comms.getRightVelocity() * 60.0f
            / (2.0f * PI * WHEEL_RADIUS_M);
    }

    // 4. Read encoder counts.
    long leftTicks = encoders.getLeftTicks();
    long rightTicks = encoders.getRightTicks();

    // 5. Update PID at the configured interval.
    if (speedPID.update(
            targetLeftRPM,
            targetRightRPM,
            leftTicks,
            rightTicks
        )) {

        // 6. Apply the calculated PWM commands.
        motors.setSpeeds(
            static_cast<int>(speedPID.getLeftOutput()),
            static_cast<int>(speedPID.getRightOutput())
        );
    // Send encoder counts and RPM to Raspberry Pi at regular intervals (20 Hz).
    static uint32_t lastTelemetryMs = 0;

    if (millis() - lastTelemetryMs >= 50) {
        lastTelemetryMs = millis();

        Serial1.print("ENC,");
        Serial1.print(encoders.getLeftTicks());
        Serial1.print(",");
        Serial1.print(encoders.getRightTicks());
        Serial1.print(",");
        Serial1.print(speedPID.getLeftRPM(), 1);
        Serial1.print(",");
        Serial1.println(speedPID.getRightRPM(), 1);
    }

        // Optional USB debugging.
        Serial.print("Target RPM: ");
        Serial.print(targetLeftRPM);
        Serial.print(", ");
        Serial.print(targetRightRPM);

        Serial.print(" | Actual RPM: ");
        Serial.print(speedPID.getLeftRPM());
        Serial.print(", ");
        Serial.println(speedPID.getRightRPM());
    }
}
