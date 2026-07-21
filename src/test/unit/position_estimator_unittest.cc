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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <math.h>

extern "C" {
    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/filter.h"
    #include "common/maths.h"
    #include "common/time.h"

    #include "fc/runtime_config.h"

    #include "flight/imu.h"
    #include "flight/position.h"
    #include "flight/position_estimator.h"

    #include "io/gps.h"

    #include "pg/pos_hold.h"
    #include "pg/autopilot.h"
    #include "pg/position.h"

    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"

    // Test-visible internals (kalmanInit/Predict/Update and getLinearAccelENU
    // are declared in position_estimator.h which is included above).
    void positionEstimatorInit(void);
    void positionEstimatorUpdate(timeUs_t currentTimeUs);
    void positionEstimatorEnableXY(bool enable);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {

// Mocks
acc_t acc;
baro_t baro;
float rMat[3][3];
attitudeEulerAngles_t attitude;

uint8_t debugMode;
int32_t debug[DEBUG_VALUE_COUNT];

// Mock time: advances by 10000 us (100 Hz) each call to mock_micros_advance.
static timeUs_t mock_micros_value = 0;
timeUs_t micros(void) { return mock_micros_value; }
void mock_micros_advance(timeUs_t delta) { mock_micros_value += delta; }
bool baroIsReady(void) { return true; }

uint8_t armingFlags;
uint16_t flightModeFlags;
uint8_t stateFlags = 0;

gpsSolutionData_t gpsSol;
float GPS_scaleLonDown;

#undef STATE
#undef ENABLE_STATE
#undef DISABLE_STATE
#undef ARMING_FLAG
#undef ENABLE_ARMING_FLAG
#undef DISABLE_ARMING_FLAG

PG_RESET_TEMPLATE(posHoldConfig_t, posHoldConfig,
    .deadband = 5,
    .positionSource = POSHOLD_SOURCE_AUTO,
    .minSats = 5,
    .gpsValidityTimeout = 5,
);
PG_REGISTER_WITH_RESET_TEMPLATE(posHoldConfig_t, posHoldConfig, PG_POSHOLD_CONFIG, 0);

PG_RESET_TEMPLATE(autopilotConfig_t, autopilotConfig,
    .positionA = 30,
);
PG_REGISTER_WITH_RESET_TEMPLATE(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT_CONFIG, 0);

PG_RESET_TEMPLATE(positionConfig_t, positionConfig,
    .alt_source = ALT_SOURCE_DEFAULT,
);
PG_REGISTER_WITH_RESET_TEMPLATE(positionConfig_t, positionConfig, PG_POSITION, 0);

bool sensors(uint32_t mask) { return (mask & SENSOR_GPS) != 0; }
void sensorsSet(uint32_t mask) { (void)mask; }
void sensorsClear(uint32_t mask) { (void)mask; }
uint32_t sensorsMask(void) { return 0; }

bool STATE(uint32_t mask) { return (stateFlags & (mask)) != 0; }
void ENABLE_STATE(uint32_t mask) { stateFlags |= (mask); }
void DISABLE_STATE(uint32_t mask) { stateFlags &= ~(mask); }
bool ARMING_FLAG(uint32_t mask) { return (armingFlags & (mask)) != 0; }
void ENABLE_ARMING_FLAG(uint32_t mask) { armingFlags |= (mask); }
void DISABLE_ARMING_FLAG(uint32_t mask) { armingFlags &= ~(mask); }

float pidGetPidFrequency(void) { return 8000.0f; }

void GPS_calc_longitude_scaling(int32_t lat) {
    (void)lat;
    GPS_scaleLonDown = cos_approx((float)lat / 1e7f * RAD);
}

void GPS_distance2d(int32_t *currentLat, int32_t *currentLon, int32_t *originLat, int32_t *originLon, float *offsetEast, float *offsetNorth)
{
    GPS_calc_longitude_scaling(*currentLat);
    const float dLat = (float)(*currentLat - *originLat);
    const float dLon = (float)(*currentLon - *originLon) * GPS_scaleLonDown;
    // 1.113195f is cm per 1e-7 degree. Matches the real GPS_distance2d and GPS_distance_cm_bearing.
    const float scale = 1.113195f;
    *offsetEast = dLon * scale;
    *offsetNorth = dLat * scale;
}

} // extern "C"

class PositionEstimatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        memset(&acc, 0, sizeof(acc));
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
        GPS_scaleLonDown = 1.0f;
        mock_micros_value = 0;

        posHoldConfig_System.positionSource = POSHOLD_SOURCE_AUTO;
        posHoldConfig_System.minSats = 5;
        posHoldConfig_System.gpsValidityTimeout = 5;
        autopilotConfig_System.positionA = 30;
        positionConfig_System.alt_source = ALT_SOURCE_DEFAULT;

        positionEstimatorInit();
    }
};

TEST_F(PositionEstimatorTest, GPSCoordinateToENU)
{
    int32_t originLat = 515000000; // ~51.5N
    int32_t originLon = -000100000; // ~0.1W
    int32_t currentLat = 515000100; // move 100 units of 1e-7 deg lat = ~1.11m north = ~111.3cm
    int32_t currentLon = -000100000;

    float east, north;
    GPS_distance2d(&currentLat, &currentLon, &originLat, &originLon, &east, &north);

    EXPECT_NEAR(north, 111.3f, 1.0f);
    EXPECT_NEAR(east, 0.0f, 1.0f);
}

TEST_F(PositionEstimatorTest, HeadingRequiredWithGPS)
{
    ENABLE_STATE(GPS_FIX);
    EXPECT_TRUE(positionEstimatorIsHeadingRequired());
}

TEST_F(PositionEstimatorTest, HeadingNotRequiredWithoutGPS)
{
    DISABLE_STATE(GPS_FIX);
    EXPECT_FALSE(positionEstimatorIsHeadingRequired());
}

TEST_F(PositionEstimatorTest, IMUAccelENU_Level)
{
    ENABLE_ARMING_FLAG(ARMED);
    acc.dev.acc_1G = 256;
    acc.dev.acc_1G_rec = 1.0f / 256.0f;
    acc.accADC[X] = 0;
    acc.accADC[Y] = 0;
    acc.accADC[Z] = 256; // 1G up

    rMat[0][0] = 1; rMat[0][1] = 0; rMat[0][2] = 0;
    rMat[1][0] = 0; rMat[1][1] = 1; rMat[1][2] = 0;
    rMat[2][0] = 0; rMat[2][1] = 0; rMat[2][2] = 1;

    float ae, an, au;
    getLinearAccelENU(&ae, &an, &au);

    EXPECT_NEAR(ae, 0.0f, 1.0f);
    EXPECT_NEAR(an, 0.0f, 1.0f);
    EXPECT_NEAR(au, 0.0f, 1.0f);
}

TEST_F(PositionEstimatorTest, GPSMeasurementFusesIntoEstimate)
{
    ENABLE_ARMING_FLAG(ARMED);
    ENABLE_STATE(GPS_FIX);

    acc.dev.acc_1G = 256;
    acc.dev.acc_1G_rec = 1.0f / 256.0f;
    acc.accADC[X] = 0;
    acc.accADC[Y] = 0;
    acc.accADC[Z] = 256;

    positionEstimatorEnableXY(true);

    gpsSol.numSat = 12;
    gpsSol.hdop = 150; // 1.5
    gpsSol.llh.lat = 515000000;
    gpsSol.llh.lon = -000100000;
    gpsSol.velE = 0;
    gpsSol.velN = 0;

    mock_micros_advance(10000);
    positionEstimatorUpdate(mock_micros_value);

    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    EXPECT_TRUE(est->isValidXY);
    EXPECT_NEAR(est->position.v[ENU_EAST], 0.0f, 10.0f);
    EXPECT_NEAR(est->position.v[ENU_NORTH], 0.0f, 10.0f);
}

TEST_F(PositionEstimatorTest, EstimateDriftsWithoutGPS)
{
    ENABLE_ARMING_FLAG(ARMED);
    DISABLE_STATE(GPS_FIX);
    positionEstimatorEnableXY(true);

    acc.dev.acc_1G = 256;
    acc.dev.acc_1G_rec = 1.0f / 256.0f;
    acc.accADC[X] = 10;
    acc.accADC[Y] = 0;
    acc.accADC[Z] = 256;

    mock_micros_advance(10000);
    positionEstimatorUpdate(mock_micros_value);

    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    EXPECT_FALSE(est->isValidXY);
}

// C1 regression test: kalmanUpdate covariance update must be the standard
// (I - K H) P form. Observable consequence: after repeated updates with a
// fixed measurement, the estimate converges to that measurement and the
// per-update delta goes to zero (innovation -> 0). With the original buggy
// "Joseph form", the covariance update was wrong and the gain either blew
// up or failed to reduce the error, so the estimate did not converge.
TEST_F(PositionEstimatorTest, KalmanConvergesToFixedMeasurement)
{
    ENABLE_ARMING_FLAG(ARMED);
    ENABLE_STATE(GPS_FIX);

    acc.dev.acc_1G = 256;
    acc.dev.acc_1G_rec = 1.0f / 256.0f;
    acc.accADC[X] = 0;
    acc.accADC[Y] = 0;
    acc.accADC[Z] = 256; // level, no linear accel

    positionEstimatorEnableXY(true);

    // Origin at ~51.5N. Step north by 1000 cm = 10 m.
    // 1e-7 deg lat = 1.113195 cm, so 1000 cm = 898.3 units of 1e-7 deg.
    const int32_t originLat = 515000000;
    const int32_t originLon = -1000000;
    const int32_t stepLatUnits = (int32_t)(1000.0f / 1.113195f);   // ~898
    gpsSol.llh.lat = originLat;
    gpsSol.llh.lon = originLon;
    gpsSol.velN = 0;
    gpsSol.velE = 0;
    gpsSol.numSat = 12;
    gpsSol.hdop = 150;

    // First call captures the origin from the current GPS fix.
    mock_micros_advance(10000);
    positionEstimatorUpdate(mock_micros_value);

    // Step north by 1000 cm for all subsequent cycles.
    gpsSol.llh.lat = originLat + stepLatUnits;

    float prevNorth = 1e9f;
    float maxAbsDelta = 0.0f;
    for (int i = 0; i < 60; ++i) {
        mock_micros_advance(10000);
        positionEstimatorUpdate(mock_micros_value);
        const positionEstimate3d_t *est = positionEstimatorGetEstimate();
        const float north = est->position.v[ENU_NORTH];
        const float delta = fabsf(north - prevNorth);
        if (i > 0 && delta > maxAbsDelta) maxAbsDelta = delta;
        prevNorth = north;
    }

    // After 60 cycles the estimate must be within ~5 cm of the 1000 cm
    // north step. With the original buggy covariance update the estimate
    // diverged or oscillated.
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    EXPECT_NEAR(est->position.v[ENU_NORTH], 1000.0f, 20.0f);

    // The per-update delta must shrink monotonically as the filter converges
    // (innovation -> 0). A non-decreasing delta indicates unstable covariance.
    // We compute the delta of the last 10 cycles and require it to be much
    // smaller than the largest delta observed in the first 50 cycles.
    float lastDelta = 0.0f;
    for (int i = 0; i < 10; ++i) {
        mock_micros_advance(10000);
        positionEstimatorUpdate(mock_micros_value);
        const float north = positionEstimatorGetEstimate()->position.v[ENU_NORTH];
        const float d = fabsf(north - prevNorth);
        if (d > lastDelta) lastDelta = d;
        prevNorth = north;
    }
    EXPECT_LT(lastDelta, maxAbsDelta * 0.5f);
}

// C1 regression test: a GPS velocity measurement must drive the velocity
// estimate toward the measured value. With the buggy velocity-update block
// the covariance (and therefore the gain) was wrong, so the velocity state
// failed to track the measurement.
TEST_F(PositionEstimatorTest, KalmanVelocityTracksMeasurement)
{
    ENABLE_ARMING_FLAG(ARMED);
    ENABLE_STATE(GPS_FIX);

    acc.dev.acc_1G = 256;
    acc.dev.acc_1G_rec = 1.0f / 256.0f;
    acc.accADC[X] = 0;
    acc.accADC[Y] = 0;
    acc.accADC[Z] = 256;

    positionEstimatorEnableXY(true);

    const int32_t originLat = 515000000;
    const int32_t originLon = -1000000;
    gpsSol.llh.lat = originLat;
    gpsSol.llh.lon = originLon;
    // 200 cm/s north
    gpsSol.velN = 200;
    gpsSol.velE = 0;
    gpsSol.numSat = 12;
    gpsSol.hdop = 150;

    for (int i = 0; i < 50; ++i) {
        mock_micros_advance(10000);
        positionEstimatorUpdate(mock_micros_value);
    }

    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    // Velocity estimate must converge toward 200 cm/s north within ~30 cm/s.
    EXPECT_NEAR(est->velocity.v[ENU_NORTH], 200.0f, 30.0f);
}

// S9: Direct Kalman unit tests — exercise the filter primitives in isolation
// to verify the math is correct. These complement the indirect C1 regression
// tests above and would have caught the original covariance-update bug.

TEST_F(PositionEstimatorTest, KalmanInitSetsDefaults)
{
    positionKalman_t kf;
    kalmanInit(&kf, 0.05f, 1.0f, 500.0f, 200.0f);

    EXPECT_FLOAT_EQ(kf.q[0], 0.05f);
    EXPECT_FLOAT_EQ(kf.q[1], 1.0f);
    EXPECT_FLOAT_EQ(kf.rPos, 500.0f);
    EXPECT_FLOAT_EQ(kf.rVel, 200.0f);
    EXPECT_FLOAT_EQ(kf.p[0][0], 10000.0f);
    EXPECT_FLOAT_EQ(kf.p[1][1], 100.0f);
    EXPECT_FLOAT_EQ(kf.p[0][1], 0.0f);
    EXPECT_FLOAT_EQ(kf.p[1][0], 0.0f);
    EXPECT_FLOAT_EQ(kf.x[0], 0.0f);
    EXPECT_FLOAT_EQ(kf.x[1], 0.0f);
}

TEST_F(PositionEstimatorTest, KalmanPredictAdvancesState)
{
    positionKalman_t kf;
    kalmanInit(&kf, 0.05f, 1.0f, 500.0f, 200.0f);

    // Set initial velocity: 100 cm/s
    kf.x[1] = 100.0f;

    // Predict with accel = 50 cm/s^2, dt = 0.1 s
    const float accel = 50.0f;
    const float dt = 0.1f;
    kalmanPredict(&kf, accel, dt);

    // Expected: x[0] += vel*dt + 0.5*accel*dt^2 = 100*0.1 + 0.5*50*0.01 = 10 + 0.25 = 10.25
    //           x[1] += accel*dt = 100 + 50*0.1 = 105
    EXPECT_NEAR(kf.x[0], 10.25f, 0.01f);
    EXPECT_NEAR(kf.x[1], 105.0f, 0.01f);

    // Covariance must grow (predict increases uncertainty)
    EXPECT_GT(kf.p[0][0], 10000.0f);
    EXPECT_GT(kf.p[1][1], 100.0f);
}

TEST_F(PositionEstimatorTest, KalmanUpdateReducesCovariance)
{
    positionKalman_t kf;
    kalmanInit(&kf, 0.05f, 1.0f, 500.0f, 200.0f);

    const float p00Before = kf.p[0][0];
    const float p11Before = kf.p[1][1];

    // Feed a position measurement of 1000 cm, velocity 0
    kalmanUpdate(&kf, 1000.0f, 0.0f, 500.0f, 200.0f);

    // Covariance must decrease (correction reduces uncertainty)
    EXPECT_LT(kf.p[0][0], p00Before);
    EXPECT_LT(kf.p[1][1], p11Before);

    // State must move toward the measurement
    EXPECT_GT(kf.x[0], 0.0f);
    EXPECT_NEAR(kf.x[0], 1000.0f, 200.0f);  // partial convergence in one step
}

TEST_F(PositionEstimatorTest, KalmanUpdateConvergesToMeasurement)
{
    positionKalman_t kf;
    kalmanInit(&kf, 0.05f, 1.0f, 100.0f, 1000.0f);

    // Repeatedly update with the same measurement; state must converge.
    for (int i = 0; i < 100; ++i) {
        kalmanUpdate(&kf, 500.0f, 50.0f, 100.0f, 1000.0f);
    }

    EXPECT_NEAR(kf.x[0], 500.0f, 1.0f);
    EXPECT_NEAR(kf.x[1], 50.0f, 5.0f);
}

// S2: Verify trustXY uses min(East, North), not a leaky average.
TEST_F(PositionEstimatorTest, TrustXYUsesMinOfAxes)
{
    ENABLE_ARMING_FLAG(ARMED);
    ENABLE_STATE(GPS_FIX);

    acc.dev.acc_1G = 256;
    acc.dev.acc_1G_rec = 1.0f / 256.0f;
    acc.accADC[X] = 0;
    acc.accADC[Y] = 0;
    acc.accADC[Z] = 256;

    positionEstimatorEnableXY(true);

    gpsSol.numSat = 12;
    gpsSol.hdop = 150;
    gpsSol.llh.lat = 515000000;
    gpsSol.llh.lon = -1000000;
    gpsSol.velE = 0;
    gpsSol.velN = 0;

    // Run one cycle to get a valid estimate with trust.
    mock_micros_advance(10000);
    positionEstimatorUpdate(mock_micros_value);

    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    EXPECT_TRUE(est->isValidXY);
    // trustXY should be > 0 since we have a valid GPS fix.
    EXPECT_GT(est->trustXY, 0.0f);
    // trustXY should be <= 1.0
    EXPECT_LE(est->trustXY, 1.0f);
}
