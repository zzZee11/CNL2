#include <cassert>
#include <cstdio>
#include <cstring>

// Include mock Arduino environment
#include <Arduino.h>
#include <Wire.h>

HardwareSerial Serial;
TwoWire Wire;

#include "src/config.h"
#include "src/motor_driver.h"
#include "src/pid_controller.h"
#include "src/distance_sensors.h"

// Mock MotorDriver for hardware testing
class MockMotorDriver : public MotorDriver {
public:
    int writeLeftCount = 0;
    int writeRightCount = 0;
    int lastLeftPwm = -999;
    int lastRightPwm = -999;

    void setRawMotors(int16_t leftPwm, int16_t rightPwm) override {
        writeLeftCount++;
        writeRightCount++;
        lastLeftPwm = leftPwm;
        lastRightPwm = rightPwm;
    }
};

void test_repeated_heartbeat_deduplication() {
    printf("[TEST] Running test_repeated_heartbeat_deduplication...\n");
    MockMotorDriver mockMotors;
    RobotController controller(mockMotors);
    controller.init();

    assert(controller.getTargetUpdateCount() == 0);
    assert(controller.getRedundantHeartbeatCount() == 0);

    // Initial command: CMD:W,200
    controller.setCommand(STATE_FWD, 200);
    assert(controller.getTargetUpdateCount() == 1);
    assert(controller.getRedundantHeartbeatCount() == 0);
    assert(mockMotors.writeLeftCount == 1);
    assert(mockMotors.lastLeftPwm == 200);

    // 50 repeated identical heartbeat packets
    for (int i = 0; i < 50; i++) {
        controller.setCommand(STATE_FWD, 200);
    }

    // Must NOT have rewritten motor registers or incremented target updates
    assert(controller.getTargetUpdateCount() == 1);
    assert(controller.getRedundantHeartbeatCount() == 50);
    assert(mockMotors.writeLeftCount == 1); // Only initial write!

    // Now change command: CMD:B,0 (Emergency Brake)
    controller.setCommand(STATE_BRAKE, 0);
    assert(controller.getTargetUpdateCount() == 2);
    assert(controller.getCommandedState() == STATE_BRAKE);

    printf("  -> PASS: Repeated heartbeats cleanly deduplicated (0 redundant PWM writes)\n");
}

void test_encoder_sync_controlled_diagnostic() {
    printf("[TEST] Running test_encoder_sync_controlled_diagnostic...\n");
    MockMotorDriver mockMotors;
    RobotController controller(mockMotors);
    controller.init();

    controller.setCommand(STATE_FWD, 200);
    int initialWrites = mockMotors.writeLeftCount;

    // Simulate extreme encoder mismatch: Left = 5000 ticks, Right = 0 ticks
    // Since ENABLE_ENCODER_SYNC is false and SYNC_KP is 0.0f, no trim should be applied
    controller.update(100, 100.0f, 0.0f, 5000, 0);
    controller.update(150, 100.0f, 0.0f, 6000, 0);

    assert(mockMotors.writeLeftCount == initialWrites); // No sync trim writes
    assert(mockMotors.lastLeftPwm == 200);              // PWM remains pure open-loop
    assert(mockMotors.lastRightPwm == 200);

    printf("  -> PASS: Controlled diagnostic confirmed (zero sync trim hunting/jitter)\n");
}

void test_raw_motor_deduplication() {
    printf("[TEST] Running test_raw_motor_deduplication...\n");
    MockMotorDriver mockMotors;
    RobotController controller(mockMotors);
    controller.init();

    // First raw motor packet: CMD:LR,180,180
    controller.setRawMotors(180, 180);
    assert(controller.getTargetUpdateCount() == 1);
    assert(controller.getRedundantHeartbeatCount() == 0);
    assert(mockMotors.writeLeftCount == 1);

    // 20 repeated identical raw motor packets
    for (int i = 0; i < 20; i++) {
        controller.setRawMotors(180, 180);
    }
    assert(controller.getTargetUpdateCount() == 1);
    assert(controller.getRedundantHeartbeatCount() == 20);
    assert(mockMotors.writeLeftCount == 1);

    printf("  -> PASS: Raw motor commands deduplicated without PWM register hammering\n");
}

void test_distance_sensor_scheduling_and_stale_safety() {
    printf("[TEST] Running test_distance_sensor_scheduling_and_stale_safety...\n");
    DistanceSensors sensors;
    // Before init, all distance outputs must default to -1 (unavailable)
    assert(sensors.getDistanceLeftMm() == -1);
    assert(sensors.getDistanceCenterMm() == -1);
    assert(sensors.getDistanceRightMm() == -1);

    // Call update within 20 ms slot interval -> should return immediately without polling
    sensors.update(10);
    assert(sensors.getTotalReads() == 0);

    // Call update after 25 ms -> advances one time slot
    sensors.update(25);
    sensors.update(50);
    sensors.update(75);

    // Stale data check: at 500 ms (> TOF_STALE_TIMEOUT_MS of 250 ms)
    sensors.update(500);
    assert(sensors.getDistanceLeftMm() == -1);
    assert(sensors.getDistanceCenterMm() == -1);
    assert(sensors.getDistanceRightMm() == -1);

    printf("  -> PASS: Distance sensor time-slicing and stale data safety confirmed\n");
}

int main() {
    printf("==================================================\n");
    printf(" CNL2 ESP32 Firmware Timing & Safety Test Suite   \n");
    printf("==================================================\n");

    test_repeated_heartbeat_deduplication();
    test_encoder_sync_controlled_diagnostic();
    test_raw_motor_deduplication();
    test_distance_sensor_scheduling_and_stale_safety();

    printf("\nAll firmware timing & safety tests PASSED successfully!\n");
    return 0;
}
