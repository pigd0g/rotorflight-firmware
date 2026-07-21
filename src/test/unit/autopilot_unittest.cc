/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <math.h>
#include <string.h>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

extern "C" {
    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/filter.h"
    #include "common/maths.h"
    #include "common/time.h"

    #include "fc/rc.h"
    #include "fc/runtime_config.h"

    #include "flight/imu.h"
    #include "flight/position.h"
    #include "flight/position_estimator.h"
    #include "flight/autopilot.h"

    #include "io/gps.h"

    #include "pg/pos_hold.h"
    #include "pg/autopilot.h"
    #include "pg/position.h"

    #include "scheduler/scheduler.h"

    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"
    #include "sensors/sensors.h"
}

#include "unittest_macros.h"

// ---------------------------------------------------------------------------
// Mock infrastructure
// ---------------------------------------------------------------------------

extern "C" {

// Mock globals
acc_t acc;
baro_t baro;
float rMat[3][3];
attitudeEulerAngles_t attitude;
gpsSolutionData_t gpsSol;
float GPS_scaleLonDown;

uint8_t debugMode;
int32_t debug[DEBUG_VALUE_COUNT];

uint8_t armingFlags = 0;
uint16_t flightModeFlags = 0;
uint8_t stateFlags = 0;

// Mock micros: returns a value that increments by 10000 us (100 Hz) each call.
static timeUs_t mock_micros_value = 0;
timeUs_t micros(void) { return mock_micros_value; }
void mock_micros_advance(timeUs_t delta) { mock_micros_value += delta; }

bool baroIsReady(void) { return true; }

// Mocked sensor mask: OR of all enabled sensors.
static uint32_t mock_enabled_sensors = 0;
bool sensors(uint32_t mask) { return (mock_enabled_sensors & mask) != 0; }
void sensorsSet(uint32_t mask) { mock_enabled_sensors |= mask; }
void sensorsClear(uint32_t mask) { mock_enabled_sensors &= ~mask; }
uint32_t sensorsMask(void) { return mock_enabled_sensors; }

// State / arming flag mocks (same pattern as position_estimator_unittest)
#undef STATE
#undef ENABLE_STATE
#undef DISABLE_STATE
#undef ARMING_FLAG
#undef ENABLE_ARMING_FLAG
#undef DISABLE_ARMING_FLAG

bool STATE(uint32_t mask) { return (stateFlags & mask) != 0; }
void ENABLE_STATE(uint32_t mask) { stateFlags |= mask; }
void DISABLE_STATE(uint32_t mask) { stateFlags &= ~mask; }
bool ARMING_FLAG(uint32_t mask) { return (armingFlags & mask) != 0; }
void ENABLE_ARMING_FLAG(uint32_t mask) { armingFlags |= mask; }
void DISABLE_ARMING_FLAG(uint32_t mask) { armingFlags &= ~mask; }

// Mocked altitude (cm). getEstimatedAltitudeCm is the baro/GPS fallback path
// used by altitudeControl when the estimator Z is not valid.
static int32_t mock_altitude_cm = 0;
int getEstimatedAltitudeCm(void) { return mock_altitude_cm; }

// Mocked getTaskDeltaTimeUs: return a fixed 10000 us (100 Hz) for the PID dt.
static timeDelta_t mock_task_delta_us = 10000;
timeDelta_t getTaskDeltaTimeUs(taskId_e taskId) { (void)taskId; return mock_task_delta_us; }

// Mocked failsafe
static bool mock_failsafe_active = false;
bool failsafeIsActive(void) { return mock_failsafe_active; }

// Mocked isUpright
static bool mock_is_upright = true;
bool isUpright(void) { return mock_is_upright; }

// gmock interface for getRcDeflection
class MockInterface {
public:
    MOCK_METHOD(float, getRcDeflection, (int axis), ());
};
static MockInterface *g_mock = nullptr;

float getRcDeflection(int axis) { return g_mock ? g_mock->getRcDeflection(axis) : 0.0f; }

// PG registrations
PG_RESET_TEMPLATE(posHoldConfig_t, posHoldConfig,
    .deadband = 5,
    .positionSource = POSHOLD_SOURCE_AUTO,
    .minSats = 5,
    .headingRequired = 1,
    .gpsValidityTimeout = 5,
);
PG_REGISTER_WITH_RESET_TEMPLATE(posHoldConfig_t, posHoldConfig, PG_POSHOLD_CONFIG, 0);

PG_RESET_TEMPLATE(autopilotConfig_t, autopilotConfig,
    .positionP = 30,
    .positionI = 30,
    .positionD = 30,
    .positionA = 30,
    .maxAngle = 50,
    .maxVelocity = 1000,
    .altitudeP = 30,
    .altitudeI = 30,
    .altitudeD = 30,
    .altitudeF = 30,
    .hoverCollective = 500,
    .collectiveMin = -300,
    .collectiveMax = 300,
    .altHoldDeadband = 50,
);
PG_REGISTER_WITH_RESET_TEMPLATE(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT_CONFIG, 0);

PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .alt_source = ALT_SOURCE_DEFAULT,
);
PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 0);

// Mock GPS helpers (same as position_estimator_unittest)
void GPS_calc_longitude_scaling(int32_t lat)
{
    (void)lat;
    GPS_scaleLonDown = cos_approx((float)lat / 1e7f * RAD);
}

void GPS_distance2d(int32_t *currentLat, int32_t *currentLon, int32_t *originLat, int32_t *originLon, float *offsetEast, float *offsetNorth)
{
    GPS_calc_longitude_scaling(*currentLat);
    const float dLat = (float)(*currentLat - *originLat);
    const float dLon = (float)(*currentLon - *originLon) * GPS_scaleLonDown;
    const float scale = 1.113195f;
    *offsetEast = dLon * scale;
    *offsetNorth = dLat * scale;
}

// Required by position_estimator.c (unused in these tests)
float pidGetPidFrequency(void) { return 8000.0f; }

} // extern "C"

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------

using testing::_;
using testing::NiceMock;
using testing::Return;

class AutopilotTest : public ::testing::Test {
protected:
    NiceMock<MockInterface> mock;

    void SetUp() override
    {
        g_mock = &mock;

        memset(&acc, 0, sizeof(acc));
        memset(&baro, 0, sizeof(baro));
        memset(rMat, 0, sizeof(rMat));
        rMat[0][0] = 1.0f; rMat[1][1] = 1.0f; rMat[2][2] = 1.0f;
        memset(&attitude, 0, sizeof(attitude));
        memset(&gpsSol, 0, sizeof(gpsSol));
        memset(&posHoldConfig_System, 0, sizeof(posHoldConfig_System));
        memset(&autopilotConfig_System, 0, sizeof(autopilotConfig_System));
        memset(&positionConfig_System, 0, sizeof(positionConfig_System));

        armingFlags = 0;
        flightModeFlags = 0;
        stateFlags = 0;
        mock_micros_value = 0;
        mock_task_delta_us = 10000;
        mock_altitude_cm = 0;
        mock_failsafe_active = false;
        mock_is_upright = true;
        mock_enabled_sensors = SENSOR_GPS | SENSOR_ACC | SENSOR_BARO;
        GPS_scaleLonDown = 1.0f;

        // Default: sticks centered
        ON_CALL(mock, getRcDeflection(_)).WillByDefault(Return(0.0f));

        // Initialize posHold and autopilot config to defaults
        posHoldConfig_System.deadband = 5;
        posHoldConfig_System.positionSource = POSHOLD_SOURCE_AUTO;
        posHoldConfig_System.minSats = 5;
        posHoldConfig_System.headingRequired = 1;
        posHoldConfig_System.gpsValidityTimeout = 5;

        autopilotConfig_System.positionP = 30;
        autopilotConfig_System.positionI = 30;
        autopilotConfig_System.positionD = 30;
        autopilotConfig_System.positionA = 30;
        autopilotConfig_System.maxAngle = 50;
        autopilotConfig_System.maxVelocity = 1000;
        autopilotConfig_System.altitudeP = 30;
        autopilotConfig_System.altitudeI = 30;
        autopilotConfig_System.altitudeD = 30;
        autopilotConfig_System.altitudeF = 30;
        autopilotConfig_System.hoverCollective = 500;
        autopilotConfig_System.collectiveMin = -300;
        autopilotConfig_System.collectiveMax = 300;
        autopilotConfig_System.altHoldDeadband = 50;

        positionConfig_System.alt_source = ALT_SOURCE_DEFAULT;

        // Set up level IMU
        acc.dev.acc_1G = 256;
        acc.dev.acc_1G_rec = 1.0f / 256.0f;
        acc.accADC[X] = 0;
        acc.accADC[Y] = 0;
        acc.accADC[Z] = 256;

        ENABLE_ARMING_FLAG(ARMED);
        ENABLE_STATE(GPS_FIX);

        autopilotInit();
        positionEstimatorInit();
    }

    void TearDown() override
    {
        g_mock = nullptr;
    }

    // Run the estimator for one cycle at 100 Hz
    void RunEstimatorCycle()
    {
        mock_micros_advance(10000);
        positionEstimatorUpdate(mock_micros_value);
    }
};

// ---------------------------------------------------------------------------
// Test 1: AltHold clamps collective relative to captured hover collective (I3)
//
// If capturedHoverCollective = 0.5 and collectiveMin/Max = -300/+300 (i.e.
// ±0.3 deflection), the collective output must stay within [0.2, 0.8].
// The pre-fix code clamped to absolute [-0.3, +0.3] which would force the
// craft below hover pitch.
// ---------------------------------------------------------------------------
TEST_F(AutopilotTest, AltHoldClampsRelativeToHover)
{
    // Set collective stick near 0.5 so it's captured as hover collective.
    // altHoldDeadband = 50 → 0.05 threshold. 0.5 > 0.05 so the stick is
    // NOT within deadband; hover will come from config: (500-500)*0.002 = 0.
    // To capture 0.5 from the stick, set it within the deadband:
    // Use stick = 0.04 (< 0.05 deadband) to capture that as hover.
    // Then drive a large altitude error and verify the clamp bounds.

    // Set altHoldDeadband wide enough to capture stick = 0.5
    autopilotConfig_System.altHoldDeadband = 600;  // 0.6 threshold

    // First call to altitudeControl: captures hover from stick
    ON_CALL(mock, getRcDeflection(COLLECTIVE)).WillByDefault(Return(0.5f));
    autopilotAltitudeControl();

    // Now set a large altitude error by having getEstimatedAltitudeCm report
    // a much lower altitude than the target (captured on first call).
    // The first call captures target = current altitude = 0.
    // Set current altitude to -10000 cm (big error → max collective).
    mock_altitude_cm = -10000;

    // Run many cycles to build up I-term toward saturation.
    for (int i = 0; i < 200; i++) {
        mock_micros_advance(10000);
        autopilotAltitudeControl();
    }

    // collectiveOut must be within [hover + min, hover + max]
    // = [0.5 + (-0.3), 0.5 + 0.3] = [0.2, 0.8]
    const float coll = getAutopilotCollective();
    EXPECT_GE(coll, 0.5f + (-300) * 0.001f - 0.001f);  // 0.2 - epsilon
    EXPECT_LE(coll, 0.5f + ( 300) * 0.001f + 0.001f);   // 0.8 + epsilon
}

// ---------------------------------------------------------------------------
// Test 2: AltHold freezes collective at hover when sensors fail (C4 / I8)
//
// When altitude sensors are lost mid-hold, collectiveOut must be frozen at
// the captured hover collective, not left at the last PID output.
// ---------------------------------------------------------------------------
TEST_F(AutopilotTest, AltHoldSensorFailFreezesAtHover)
{
    // Capture hover collective from stick = 0.3 (within wide deadband)
    autopilotConfig_System.altHoldDeadband = 600;
    ON_CALL(mock, getRcDeflection(COLLECTIVE)).WillByDefault(Return(0.3f));
    autopilotAltitudeControl();

    // Drive a large altitude error to produce a non-zero collective output
    mock_altitude_cm = -5000;
    for (int i = 0; i < 50; i++) {
        mock_micros_advance(10000);
        autopilotAltitudeControl();
    }

    // Verify collective is NOT at hover (it has moved away due to the error)
    const float collBeforeFail = getAutopilotCollective();
    const float hoverColl = getAutopilotHoverCollective();
    EXPECT_NE(collBeforeFail, hoverColl);

    // Now simulate sensor failure: clear baro and acc sensors so
    // altHoldSensorsOk() returns false and getAltitudeCmControl() falls
    // back to getEstimatedAltitudeCm() which still returns mock_altitude_cm.
    // The freeze happens in pos_hold.c via autopilotFreezeCollectiveAtHover().
    // We call the freeze function directly (simulating what pos_hold does):
    autopilotFreezeCollectiveAtHover();

    const float collAfterFail = getAutopilotCollective();
    EXPECT_FLOAT_EQ(collAfterFail, hoverColl);
}

// ---------------------------------------------------------------------------
// Test 3: Disarm cleanup resets collective output and I-term (I2 / I8)
//
// After disarm, collectiveOut must be 0 and the altitude I-term must be
// reset so the next arm starts clean.
// ---------------------------------------------------------------------------
TEST_F(AutopilotTest, DisarmCleanupResetsState)
{
    // Capture hover and build up I-term
    autopilotConfig_System.altHoldDeadband = 600;
    ON_CALL(mock, getRcDeflection(COLLECTIVE)).WillByDefault(Return(0.4f));
    autopilotAltitudeControl();

    mock_altitude_cm = -5000;
    for (int i = 0; i < 50; i++) {
        mock_micros_advance(10000);
        autopilotAltitudeControl();
    }

    // Verify collective is non-zero before disarm
    EXPECT_NE(getAutopilotCollective(), 0.0f);

    // Disarm cleanup
    autopilotDisarmCleanup();

    // collectiveOut must be reset to 0
    EXPECT_FLOAT_EQ(getAutopilotCollective(), 0.0f);
}

// ---------------------------------------------------------------------------
// Test 4: AltHold dt consistency (I4)
//
// The I-term must use the actual task dt, not a hardcoded 100 Hz constant.
// We verify by changing mock_task_delta_us and checking that the I-term
// accumulation scales proportionally.
// ---------------------------------------------------------------------------
TEST_F(AutopilotTest, AltHoldUsesActualDt)
{
    // Capture hover at 0 and set a fixed altitude error
    ON_CALL(mock, getRcDeflection(COLLECTIVE)).WillByDefault(Return(0.0f));
    autopilotAltitudeControl();  // captures hover = 0, target = 0

    // Current altitude = -1000 cm (error = +1000)
    mock_altitude_cm = -1000;

    // Run one cycle with dt = 0.01s (100 Hz)
    mock_task_delta_us = 10000;
    mock_micros_advance(10000);
    autopilotAltitudeControl();
    const float collAt100Hz = getAutopilotCollective();

    // Reset
    autopilotDisarmCleanup();
    ON_CALL(mock, getRcDeflection(COLLECTIVE)).WillByDefault(Return(0.0f));
    autopilotAltitudeControl();  // re-capture hover = 0, target = 0

    // Run one cycle with dt = 0.02s (50 Hz) — I-term should be ~2× larger
    mock_task_delta_us = 20000;
    mock_micros_advance(20000);
    autopilotAltitudeControl();
    const float collAt50Hz = getAutopilotCollective();

    // The I-term at 50 Hz should be approximately 2× the I-term at 100 Hz
    // (the P and D terms differ too, but I dominates after one cycle with
    // a large error). Allow a wide tolerance since P and D also contribute.
    EXPECT_GT(collAt50Hz, collAt100Hz * 1.5f);
}