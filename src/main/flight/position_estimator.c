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

#include "platform.h"

#if defined(USE_POSITION_HOLD) || defined(USE_ALTITUDE_HOLD)

#include "flight/position_estimator.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/time.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/runtime_config.h"

#include "drivers/time.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "flight/position_estimator.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/position.h"
#include "pg/pos_hold.h"
#include "pg/autopilot.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sensors.h"

// Kalman process noise for position/velocity estimator.
// q[0] = position noise, q[1] = velocity noise.
// Scaled by ap_position_a to allow tuning of IMU accel trust.
#define KALMAN_Q_POSITION   0.05f
#define KALMAN_Q_VELOCITY   1.0f
#define GRAVITY_CMSS        980.665f

// GPS measurement noise base values (cm and cm/s respectively).
// R scales with DOP quadratically in gpsR().
#define GPS_R_POSITION_BASE 500.0f
#define GPS_R_VELOCITY_BASE 200.0f

// Unknown DOP must not be treated as excellent GPS.
#define GPS_DOP_MIN_VALID   100     // hdop < 1.0
#define GPS_DOP_SCALE       0.01f   // hdop is stored *100
#define GPS_DOP_UNKNOWN_R_SCALE 10.0f

// Minimum satellites for a trustworthy GPS fix.
#define GPS_MIN_SATS_DEFAULT 5

typedef struct positionKalman_s {
    float x[2];     // [position, velocity]
    float p[2][2];  // error covariance
    float q[2];     // process noise
    float rPos;     // measurement noise for position
    float rVel;     // measurement noise for velocity
} positionKalman_t;

typedef struct posEstimatorState_s {
    positionEstimate3d_t estimate;

    positionKalman_t kfEast;
    positionKalman_t kfNorth;
    positionKalman_t kfUp;

    bool xyEnabled;
    bool zEnabled;
    bool originValid;
    int32_t originLat;
    int32_t originLon;
    int32_t originAltCm;

    bool sticksActive;
    timeUs_t lastUpdateUs;

    // Latched-validity miss counters: increment each cycle the sensor does
    // not feed, reset to 0 when it does. isValidXY/Z only cleared after
    // gpsValidityTimeout consecutive misses, preventing per-cycle toggle
    // when satellite count dips briefly below the threshold.
    uint8_t gpsMissCount;
    uint8_t baroMissCount;
} posEstimatorState_t;

static FAST_DATA_ZERO_INIT posEstimatorState_t posEstimator;

static inline void kalmanInit(positionKalman_t *kf, float qPos, float qVel, float rPos, float rVel)
{
    memset(kf, 0, sizeof(*kf));
    kf->q[0] = qPos;
    kf->q[1] = qVel;
    kf->rPos = rPos;
    kf->rVel = rVel;
    kf->p[0][0] = 10000.0f;
    kf->p[1][1] = 100.0f;
}

// Predict step: integrate acceleration over dt.
// x = F*x + B*u, P = F*P*F' + Q
STATIC_UNIT_TESTED void kalmanPredict(positionKalman_t *kf, float accel, float dt)
{
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;

    // State transition: position += velocity*dt + 0.5*accel*dt^2, velocity += accel*dt
    kf->x[0] += kf->x[1] * dt + 0.5f * accel * dt2;
    kf->x[1] += accel * dt;

    // Covariance update
    kf->p[0][0] += 2.0f * kf->p[0][1] * dt + kf->p[1][1] * dt2 + kf->q[0] * dt2;
    kf->p[0][1] += kf->p[1][1] * dt + kf->q[1] * dt3;
    kf->p[1][0] = kf->p[0][1];
    kf->p[1][1] += kf->q[1] * dt2;
}

// Correct step with position and velocity measurements.
// Standard one-sided covariance update P' = (I - K H) P, valid for R > 0
// and independent measurements. Symmetry is preserved by writing P[1][0] = P[0][1].
STATIC_UNIT_TESTED void kalmanUpdate(positionKalman_t *kf, float posMeasured, float velMeasured, float rPos, float rVel)
{
    // Position update (H = [1, 0])
    const float yPos = posMeasured - kf->x[0];
    const float sPos = kf->p[0][0] + rPos;
    if (sPos > 0.0f) {
        const float kPos0 = kf->p[0][0] / sPos;
        const float kPos1 = kf->p[1][0] / sPos;
        kf->x[0] += kPos0 * yPos;
        kf->x[1] += kPos1 * yPos;

        const float p00 = kf->p[0][0];
        const float p01 = kf->p[0][1];
        const float p10 = kf->p[1][0];
        const float p11 = kf->p[1][1];

        // (I - K H) P with H = [1, 0], K = [kPos0, kPos1]ᵀ
        // (I - K H) = [[1-kPos0, 0], [-kPos1, 1]]
        const float c00 = 1.0f - kPos0;
        const float c10 = -kPos1;

        kf->p[0][0] = c00 * p00;
        kf->p[0][1] = c00 * p01;
        kf->p[1][0] = c10 * p00 + p10;
        kf->p[1][1] = c10 * p01 + p11;
    }

    // Velocity update (H = [0, 1])
    const float yVel = velMeasured - kf->x[1];
    const float sVel = kf->p[1][1] + rVel;
    if (sVel > 0.0f) {
        const float kVel0 = kf->p[0][1] / sVel;
        const float kVel1 = kf->p[1][1] / sVel;
        kf->x[0] += kVel0 * yVel;
        kf->x[1] += kVel1 * yVel;

        const float p00 = kf->p[0][0];
        const float p01 = kf->p[0][1];
        const float p10 = kf->p[1][0];
        const float p11 = kf->p[1][1];

        // (I - K H) P with H = [0, 1], K = [kVel0, kVel1]ᵀ
        // (I - K H) = [[1, -kVel0], [0, 1-kVel1]]
        const float c01 = -kVel0;
        const float c11 = 1.0f - kVel1;

        kf->p[0][0] = p00 + c01 * p10;
        kf->p[0][1] = p01 + c01 * p11;
        kf->p[1][0] = c11 * p10;
        kf->p[1][1] = c11 * p11;
    }
}

static inline float gpsR(float baseR, uint16_t dop)
{
    if (dop < GPS_DOP_MIN_VALID) {
        return baseR * GPS_DOP_UNKNOWN_R_SCALE;
    }
    const float dopScale = dop * GPS_DOP_SCALE;
    return baseR * dopScale * dopScale;
}

static inline void setEstimateAxis(positionEstimate3d_t *est, enuAxis_e axis,
                                   positionKalman_t *kf, bool valid)
{
    est->position.v[axis] = kf->x[0];
    est->velocity.v[axis] = kf->x[1];

    const float pNorm = kf->p[0][0] + kf->p[1][1];
    float trust = 1.0f / (1.0f + pNorm / 10000.0f);
    if (!valid) {
        trust = 0.0f;
    }

    if (axis == ENU_EAST || axis == ENU_NORTH) {
        est->trustXY = 0.5f * (est->trustXY + trust);
    } else {
        est->trustZ = trust;
    }
}

// Compute earth-frame linear acceleration from IMU (gravity removed), in cm/s^2 ENU.
void getLinearAccelENU(float *accelEast, float *accelNorth, float *accelUp)
{
    const float accScale = acc.dev.acc_1G_rec;
    const float accBF[XYZ_AXIS_COUNT] = {
        acc.accADC[X] * accScale,
        acc.accADC[Y] * accScale,
        acc.accADC[Z] * accScale
    };

    // rMat rotates body -> earth NWU (North-West-Up).
    const float accEF_NWU[XYZ_AXIS_COUNT] = {
        rMat[0][0] * accBF[X] + rMat[0][1] * accBF[Y] + rMat[0][2] * accBF[Z],
        rMat[1][0] * accBF[X] + rMat[1][1] * accBF[Y] + rMat[1][2] * accBF[Z],
        rMat[2][0] * accBF[X] + rMat[2][1] * accBF[Y] + rMat[2][2] * accBF[Z]
    };

    // NWW->ENU: East = -West, North = North, Up = Up - gravity
    *accelEast  = -accEF_NWU[Y] * GRAVITY_CMSS;
    *accelNorth =  accEF_NWU[X] * GRAVITY_CMSS;
    *accelUp    = (accEF_NWU[Z] - 1.0f) * GRAVITY_CMSS;
}

static void feedGPSMeasurements(void)
{
#ifdef USE_GPS
    if (!sensors(SENSOR_GPS) || !STATE(GPS_FIX)) {
        return;
    }

    const uint8_t minSats = posHoldConfig() ? posHoldConfig()->minSats : GPS_MIN_SATS_DEFAULT;
    if (gpsSol.numSat < minSats) {
        return;
    }

    if (posHoldConfig()->positionSource == POSHOLD_SOURCE_OPTICALFLOW_ONLY) {
        return;
    }

    if (!posEstimator.originValid) {
        posEstimator.originLat = gpsSol.llh.lat;
        posEstimator.originLon = gpsSol.llh.lon;
        posEstimator.originAltCm = gpsSol.llh.altCm;
        posEstimator.originValid = true;
        GPS_calc_longitude_scaling(posEstimator.originLat);
    }

    float offsetEast, offsetNorth;
    GPS_distance2d(&gpsSol.llh.lat, &gpsSol.llh.lon,
                  &posEstimator.originLat, &posEstimator.originLon,
                  &offsetEast, &offsetNorth);

    const float rPos = gpsR(GPS_R_POSITION_BASE, gpsSol.hdop);
    const float rVel = gpsR(GPS_R_VELOCITY_BASE, gpsSol.hdop);

    kalmanUpdate(&posEstimator.kfEast,  offsetEast,  (float)gpsSol.velE, rPos, rVel);
    kalmanUpdate(&posEstimator.kfNorth, offsetNorth, (float)gpsSol.velN, rPos, rVel);

    posEstimator.gpsMissCount = 0;
    posEstimator.estimate.isValidXY = true;
#else
    UNUSED(posHoldConfig);
#endif
}

static void feedAltitudeMeasurements(void)
{
#ifdef USE_BARO
    if (!sensors(SENSOR_BARO) || !baroIsReady()) {
        return;
    }

    const float baroAltCm = baro.baroAltitude;
    if (baroAltCm < 15000 && baroAltCm > -2500) {
        // Simple baro altitude fusion relative to origin.
        const float altCm = baroAltCm - posEstimator.originAltCm;
        kalmanUpdate(&posEstimator.kfUp, altCm, 0.0f, 100.0f, 1000.0f);
        posEstimator.baroMissCount = 0;
        posEstimator.estimate.isValidZ = true;
    }
#endif
}

void positionEstimatorInit(void)
{
    memset(&posEstimator, 0, sizeof(posEstimator));

    const float qAccelScale = autopilotConfig() ? autopilotConfig()->positionA * 0.01f : 1.0f;

    kalmanInit(&posEstimator.kfEast,  KALMAN_Q_POSITION, KALMAN_Q_VELOCITY * qAccelScale, GPS_R_POSITION_BASE, GPS_R_VELOCITY_BASE);
    kalmanInit(&posEstimator.kfNorth, KALMAN_Q_POSITION, KALMAN_Q_VELOCITY * qAccelScale, GPS_R_POSITION_BASE, GPS_R_VELOCITY_BASE);
    kalmanInit(&posEstimator.kfUp,   KALMAN_Q_POSITION, KALMAN_Q_VELOCITY * qAccelScale, 100.0f, 1000.0f);
}

void positionEstimatorEnableXY(bool enable)
{
    posEstimator.xyEnabled = enable;
    if (enable) {
        posEstimator.estimate.isValidXY = false;
        posEstimator.originValid = false;
        posEstimator.gpsMissCount = 0;
        kalmanInit(&posEstimator.kfEast,  KALMAN_Q_POSITION, KALMAN_Q_VELOCITY, GPS_R_POSITION_BASE, GPS_R_VELOCITY_BASE);
        kalmanInit(&posEstimator.kfNorth, KALMAN_Q_POSITION, KALMAN_Q_VELOCITY, GPS_R_POSITION_BASE, GPS_R_VELOCITY_BASE);
    }
}

void positionEstimatorUpdate(timeUs_t currentTimeUs)
{
    (void)currentTimeUs;

    const timeUs_t now = micros();
    const float dt = (posEstimator.lastUpdateUs == 0) ? 0.01f : constrainf((now - posEstimator.lastUpdateUs) * 1e-6f, 0.0001f, 0.05f);
    posEstimator.lastUpdateUs = now;

    // Increment miss counters; reset to 0 by feedGPSMeasurements/
    // feedAltitudeMeasurements when the sensor feeds. Validity is only
    // cleared after gpsValidityTimeout consecutive misses (latch), preventing
    // PH/ALTHOLD from toggling off when a satellite count dips for one cycle.
    if (posEstimator.xyEnabled) {
        if (posEstimator.gpsMissCount < UINT8_MAX) posEstimator.gpsMissCount++;
    }
    if (posEstimator.baroMissCount < UINT8_MAX) posEstimator.baroMissCount++;

    float accelEast = 0.0f, accelNorth = 0.0f, accelUp = 0.0f;

    if (sensors(SENSOR_ACC) && ARMING_FLAG(ARMED)) {
        getLinearAccelENU(&accelEast, &accelNorth, &accelUp);
    }

    // Predict all axes
    kalmanPredict(&posEstimator.kfEast,  accelEast,  dt);
    kalmanPredict(&posEstimator.kfNorth, accelNorth, dt);
    kalmanPredict(&posEstimator.kfUp,   accelUp,    dt);

    // Correct with sensors
    if (posEstimator.xyEnabled) {
        feedGPSMeasurements();
    }

    feedAltitudeMeasurements();

    // Latch: only clear validity after N consecutive missed cycles.
    const uint8_t validityLimit = posHoldConfig() ? posHoldConfig()->gpsValidityTimeout : 5;
    if (posEstimator.xyEnabled) {
        if (posEstimator.gpsMissCount >= validityLimit) {
            posEstimator.estimate.isValidXY = false;
        }
    } else {
        posEstimator.estimate.isValidXY = false;
    }
    if (posEstimator.baroMissCount >= validityLimit) {
        posEstimator.estimate.isValidZ = false;
    }

    // Export estimate
    setEstimateAxis(&posEstimator.estimate, ENU_EAST,  &posEstimator.kfEast,  posEstimator.estimate.isValidXY);
    setEstimateAxis(&posEstimator.estimate, ENU_NORTH, &posEstimator.kfNorth, posEstimator.estimate.isValidXY);
    setEstimateAxis(&posEstimator.estimate, ENU_UP,    &posEstimator.kfUp,    posEstimator.estimate.isValidZ);

    DEBUG(AUTOPILOT, 0, posEstimator.estimate.position.v[ENU_EAST]);
    DEBUG(AUTOPILOT, 1, posEstimator.estimate.position.v[ENU_NORTH]);
    DEBUG(AUTOPILOT, 2, posEstimator.estimate.velocity.v[ENU_EAST]);
    DEBUG(AUTOPILOT, 3, posEstimator.estimate.velocity.v[ENU_NORTH]);
    DEBUG(AUTOPILOT, 4, posEstimator.estimate.isValidXY ? 1 : 0);
    DEBUG(AUTOPILOT, 5, posEstimator.estimate.trustXY * 1000);
}

const positionEstimate3d_t *positionEstimatorGetEstimate(void)
{
    return &posEstimator.estimate;
}

bool positionEstimatorIsValidXY(void)
{
    return posEstimator.estimate.isValidXY;
}

bool positionEstimatorIsHeadingRequired(void)
{
#ifdef USE_GPS
    if (posHoldConfig()->positionSource == POSHOLD_SOURCE_OPTICALFLOW_ONLY) {
        return false;
    }
    return sensors(SENSOR_GPS) && STATE(GPS_FIX);
#else
    return false;
#endif
}

void positionEstimatorSetSticksActive(bool active)
{
    posEstimator.sticksActive = active;
}

bool positionEstimatorIsSticksActive(void)
{
    return posEstimator.sticksActive;
}

#endif
