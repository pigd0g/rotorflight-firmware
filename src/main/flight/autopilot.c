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

#include "platform.h"

#if defined(USE_POSITION_HOLD) || defined(USE_ALTITUDE_HOLD)

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/leveling.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/position_estimator.h"
#include "flight/pos_hold.h"

#include "io/gps.h"

#include "pg/autopilot.h"
#include "pg/pos_hold.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sensors.h"

#include "scheduler/scheduler.h"

#include "autopilot.h"

// PID gain scales. Gains are CLI integers; these convert to degrees/cm or degrees/(cm/s).
#define POSITION_P_SCALE      0.0012f   // deg per cm of position error
#define POSITION_I_SCALE      0.00015f  // deg per cm*s integral
#define POSITION_D_SCALE      0.0017f   // deg per cm/s velocity error
#define POSITION_A_SCALE      0.0003f  // deg per cm/s^2 acceleration
#define ALTITUDE_P_SCALE      0.002f    // collective pitch offset per cm of altitude error
#define ALTITUDE_I_SCALE      0.0002f   // collective pitch offset per cm*s
#define ALTITUDE_D_SCALE      0.0005f   // collective pitch offset per cm/s
#define ALTITUDE_F_SCALE      0.0005f   // collective pitch offset per cm/s target velocity
#define YAW_P_SCALE           0.1f      // deg/s per deg bearing error (not used for PH)

// Limits and defaults.
#define ERROR_DISTANCE_LIMIT  2000.0f   // 20 m
#define POSITION_I_LIMIT      2000.0f   // 20 m*s
#define ALTITUDE_I_LIMIT      5000.0f   // cm*s
#define SANITY_CHECK_DISTANCE 2000.0f   // 20 m
#define START_BRAKE_FACTOR    1.6f
#define START_BRAKE_RAMP      0.1f
#define STOP_SPEED_CM_S       20.0f
#define MAX_ANGLE_DEFAULT     50.0f     // deg
#define VELOCITY_I_LIMIT_DEG  15.0f
#define VELOCITY_I_RELAX_CMS  250.0f
#define MIN_COS_TILT          0.5f

// Low-pass filter cutoffs.
#define ACCEL_LPF_CUTOFF      20.0f
#define VEL_LPF_CUTOFF        30.0f

static float autopilotAngle[ANGLE_INDEX_COUNT] = { 0.0f, 0.0f };
static float collectiveOut = 0.0f;    // altitude-hold collective pitch deflection, -1..1

static struct {
    bool navActive;
    bool sticksActive;
    bool wasSticksActive;
    bool isPositionHeld;
    bool isAltitudeHeld;
    float sanityCheckDistance;
} ap;

// Controller state
static vector2_t targetPosition;
static vector2_t posHoldStartPosition;
static vector2_t targetVelocity;
static vector2_t previousVelocity;
static vector2_t velocityError;
static vector2_t distanceError;
static vector2_t distanceErrorIntegral;
static vector2_t velocityIntegral;
static vector2_t pidP;
static vector2_t pidI;
static vector2_t pidD;
static vector2_t pidA;
static vector2_t pidSumVectorEF;

static bool isPosHoldStarting[EF_AXIS_COUNT] = { false, false };
static float dTermRamp[EF_AXIS_COUNT] = { 1.0f, 1.0f };
static pt2Filter_t posAccelLpf[EF_AXIS_COUNT];
static pt2Filter_t posDtermLpf[EF_AXIS_COUNT];
static bool wasAngleSaturated = false;
static vector2_t initialVelocity;

static float altitudeI = 0.0f;
static float previousAlt = 0.0f;
static float targetAltitudeCm = 0.0f;
static float capturedHoverCollective = 0.0f;

static float positionKp, positionKi, positionKd, positionKa;
static float velocityKp, velocityKi, velocityKd, velocityDragKff;
static float altitudeKp, altitudeKi, altitudeKd, altitudeKf;
static float maxAngleDeg;

static inline float getHeadingRad(void)
{
    return DECIDEGREES_TO_RADIANS(attitude.values.yaw);
}

static inline float getTaskIntervalS(void)
{
    const timeDelta_t deltaUs = getTaskDeltaTimeUs(TASK_SELF);
    return (deltaUs > 0) ? deltaUs * 1e-6f : HZ_TO_INTERVAL(POSHOLD_TASK_RATE_HZ);
}

static void rotateBodyToEF(float roll, float pitch, float yawRad, float *east, float *north)
{
    const float cosY = cos_approx(yawRad);
    const float sinY = sin_approx(yawRad);
    *north = pitch * cosY - roll * sinY;
    *east  = pitch * sinY + roll * cosY;
}

static void initPositionHold(void)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    targetPosition.v[EF_EAST]  = est->position.v[ENU_EAST];
    targetPosition.v[EF_NORTH] = est->position.v[ENU_NORTH];
    posHoldStartPosition = targetPosition;
    targetVelocity.v[EF_EAST]  = 0.0f;
    targetVelocity.v[EF_NORTH] = 0.0f;
    isPosHoldStarting[EF_EAST]  = true;
    isPosHoldStarting[EF_NORTH] = true;
    dTermRamp[EF_EAST]  = 1.0f;
    dTermRamp[EF_NORTH] = 1.0f;
}

static void resetDistanceError(void)
{
    distanceError.v[EF_EAST]  = 0.0f;
    distanceError.v[EF_NORTH] = 0.0f;
}

static void resetVelocityIntegral(void)
{
    velocityIntegral.v[EF_EAST]  = 0.0f;
    velocityIntegral.v[EF_NORTH] = 0.0f;
}

static float calculateSanityCheckDistance(void)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const float speedCmS = vector2Norm((const vector2_t *)&est->velocity.v);
    return fmaxf(SANITY_CHECK_DISTANCE, speedCmS * 2.0f);
}

static void sticksMoveTarget(void)
{
    const float yawRad = getHeadingRad();
    const float pitchStick = getRcDeflection(FD_PITCH);
    const float rollStick  = getRcDeflection(FD_ROLL);

    float northVel, eastVel;
    rotateBodyToEF(rollStick, pitchStick, yawRad, &eastVel, &northVel);

    targetVelocity.v[EF_NORTH] = northVel * autopilotConfig()->maxVelocity;
    targetVelocity.v[EF_EAST]  = eastVel  * autopilotConfig()->maxVelocity;

    const float dt = HZ_TO_INTERVAL(POSHOLD_TASK_RATE_HZ);
    targetPosition.v[EF_NORTH] += targetVelocity.v[EF_NORTH] * dt;
    targetPosition.v[EF_EAST]  += targetVelocity.v[EF_EAST]  * dt;
    posHoldStartPosition = targetPosition;
}

static bool positionControl(void)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const float dt = getTaskIntervalS();

    if (!est->isValidXY) {
            autopilotAngle[AI_ROLL] = autopilotAngle[AI_PITCH] = 0.0f;
        return false;
    }

    const vector2_t currentPosition = *(const vector2_t *)&est->position.v;
    const vector2_t velocity        = *(const vector2_t *)&est->velocity.v;

    // Station keeping: capture target on first entry and when sticks first move.
    if (!ap.isPositionHeld) {
        initPositionHold();
        ap.sanityCheckDistance = calculateSanityCheckDistance();
        ap.isPositionHeld = true;
        initialVelocity = velocity;
    } else {
        if (ap.sticksActive) {
            if (!ap.wasSticksActive) {
                initPositionHold();
                initialVelocity = velocity;
            }
            sticksMoveTarget();
        }

        vector2_t deltaPos;
        vector2Sub(&deltaPos, &posHoldStartPosition, &currentPosition);
        if (vector2Norm(&deltaPos) > ap.sanityCheckDistance) {
        autopilotAngle[AI_ROLL] = autopilotAngle[AI_PITCH] = 0.0f;
            return false;
        }
    }

    for (unsigned axis = 0; axis < EF_AXIS_COUNT; axis++) {
        const float velocityFiltered = pt2FilterApply(&posDtermLpf[axis], velocity.v[axis]);
        const float accelerationRaw = (previousVelocity.v[axis] - velocityFiltered) * POSHOLD_TASK_RATE_HZ;
        const float acceleration = pt2FilterApply(&posAccelLpf[axis], accelerationRaw);

        if (ap.isPositionHeld) {
            if (isPosHoldStarting[axis]) {
                dTermRamp[axis] += (START_BRAKE_FACTOR - dTermRamp[axis]) * START_BRAKE_RAMP;
                if (dTermRamp[axis] > START_BRAKE_FACTOR - 0.02f) {
                    dTermRamp[axis] = START_BRAKE_FACTOR;
                }

                float slowingDownFactor = 0.0f;
                if (fabsf(initialVelocity.v[axis]) > 0.01f) {
                    slowingDownFactor = 1.0f - (fabsf(velocity.v[axis]) / fabsf(initialVelocity.v[axis]));
                    slowingDownFactor = constrainf(slowingDownFactor, 0.0f, 1.0f);
                } else {
                    slowingDownFactor = 1.0f;
                }

                const float tempTarget = targetPosition.v[axis] +
                    (currentPosition.v[axis] - targetPosition.v[axis]) * slowingDownFactor;
                distanceError.v[axis] = tempTarget - currentPosition.v[axis];

                if (((initialVelocity.v[axis] * velocity.v[axis]) < 0.0f) ||
                    (fabsf(velocity.v[axis]) < STOP_SPEED_CM_S)) {
                    isPosHoldStarting[axis] = false;
                    dTermRamp[axis] = 1.0f;
                    targetPosition.v[axis] = currentPosition.v[axis];
                }
            } else {
                dTermRamp[axis] += (1.0f - dTermRamp[axis]) * START_BRAKE_RAMP;
                distanceError.v[axis] = targetPosition.v[axis] - currentPosition.v[axis];
            }
        }

        velocityError.v[axis] = targetVelocity.v[axis] - velocityFiltered;

        // Position PID (station keeping / pure PH). Integral frozen during braking ramp.
        velocityError.v[axis] *= dTermRamp[axis];
        distanceError.v[axis] = constrainf(distanceError.v[axis], -ERROR_DISTANCE_LIMIT, ERROR_DISTANCE_LIMIT);
        distanceErrorIntegral.v[axis] += distanceError.v[axis] * dt * (isPosHoldStarting[axis] ? 0.0f : 1.0f);
        distanceErrorIntegral.v[axis] = constrainf(distanceErrorIntegral.v[axis], -POSITION_I_LIMIT, POSITION_I_LIMIT);

        pidP.v[axis] = distanceError.v[axis] * positionKp;
        pidI.v[axis] = distanceErrorIntegral.v[axis] * positionKi;
        pidD.v[axis] = velocityError.v[axis] * positionKd * dTermRamp[axis];
        pidA.v[axis] = acceleration * positionKa;

        pidSumVectorEF.v[axis] = pidP.v[axis] + pidI.v[axis] + pidD.v[axis] + pidA.v[axis];
    }

    previousVelocity = velocity;

    // EF -> body frame rotation by yaw, then clamp to max angle.
    const float headingRad = getHeadingRad();
    vector2_t headingV;
    headingV.v[EF_EAST]  = sin_approx(headingRad);
    headingV.v[EF_NORTH] = cos_approx(headingRad);

    vector2_t angleV;
    angleV.v[AI_PITCH] = vector2Dot(&headingV, &pidSumVectorEF);
    angleV.v[AI_ROLL]  = vector2Cross(&headingV, &pidSumVectorEF);

    const float mag = vector2Norm(&angleV);
    wasAngleSaturated = (mag > maxAngleDeg);
    if (mag > maxAngleDeg && mag > 0.001f) {
        const float scale = maxAngleDeg / mag;
        vector2Scale(&angleV, &angleV, scale);
    }

    // Betaflight convention: roll angle sign is inverted.
    autopilotAngle[AI_ROLL]  = -angleV.v[AI_ROLL];
    autopilotAngle[AI_PITCH] =  angleV.v[AI_PITCH];

    DEBUG(AUTOPILOT_PID, 0, pidSumVectorEF.v[EF_EAST] * 10);
    DEBUG(AUTOPILOT_PID, 1, pidSumVectorEF.v[EF_NORTH] * 10);
    DEBUG(AUTOPILOT_PID, 2, autopilotAngle[AI_ROLL] * 10);
    DEBUG(AUTOPILOT_PID, 3, autopilotAngle[AI_PITCH] * 10);
    DEBUG(AUTOPILOT_PID, 4, targetPosition.v[EF_EAST]);
    DEBUG(AUTOPILOT_PID, 5, targetPosition.v[EF_NORTH]);

    return true;
}

static float getAltitudeCmControl(void)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    if (est->isValidZ) {
        return est->position.v[ENU_UP];
    }
    // Fall back to the baro/GPS-derived altitude from position.c.
    return getEstimatedAltitudeCm();
}

static void altitudeControl(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();

    if (!ap.isAltitudeHeld) {
        targetAltitudeCm = getAltitudeCmControl();
        altitudeI = 0.0f;
        ap.isAltitudeHeld = true;

        // Capture hover collective from current collective stick if near center, else config.
        const float collCmd = getRcDeflection(COLLECTIVE); // -1..+1
        if (fabsf(collCmd) < (cfg->altHoldDeadband * 0.001f)) {
            capturedHoverCollective = collCmd;
        } else {
            capturedHoverCollective = (cfg->hoverCollective - 500.0f) * 0.002f; // 0..1000 -> -1..1
        }
        return;
    }

    const float currentAlt = getAltitudeCmControl();
    const float altitudeError = targetAltitudeCm - currentAlt;
    const float itermRelax = (fabsf(altitudeError) < 200.0f) ? 1.0f : 0.1f;

    const float altitudeP = altitudeError * altitudeKp;
    altitudeI += altitudeError * altitudeKi * itermRelax * HZ_TO_INTERVAL(POSHOLD_TASK_RATE_HZ);
    altitudeI = constrainf(altitudeI, -ALTITUDE_I_LIMIT, ALTITUDE_I_LIMIT);

    const float vario = (currentAlt - previousAlt) * POSHOLD_TASK_RATE_HZ;
    previousAlt = currentAlt;
    const float altitudeD = -vario * altitudeKd;
    const float altitudeF = 0.0f; // no target vertical velocity for basic alt hold

    // Altitude PID produces a collective offset in [-1..1] deflection units.
    float collectiveOffset = altitudeP + altitudeI + altitudeD + altitudeF;

    // Add hover trim and clamp to configured min/max collective relative to hover.
    float collectiveCmd = capturedHoverCollective + collectiveOffset;
    const float minCollective = cfg->collectiveMin * 0.001f;
    const float maxCollective = cfg->collectiveMax * 0.001f;
    collectiveCmd = constrainf(collectiveCmd, minCollective, maxCollective);

    collectiveOut = collectiveCmd;

    DEBUG(AUTOPILOT_ALTITUDE, 0, currentAlt);
    DEBUG(AUTOPILOT_ALTITUDE, 1, targetAltitudeCm);
    DEBUG(AUTOPILOT_ALTITUDE, 2, altitudeError);
    DEBUG(AUTOPILOT_ALTITUDE, 3, collectiveOut * 1000);
    DEBUG(AUTOPILOT_ALTITUDE, 4, altitudeI);
}

static void autopilotLoadGains(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();

    positionKp = cfg->positionP  * POSITION_P_SCALE;
    positionKi = cfg->positionI  * POSITION_I_SCALE;
    positionKd = cfg->positionD  * POSITION_D_SCALE;
    positionKa = cfg->positionA  * POSITION_A_SCALE;

    velocityKp      = cfg->velocityP * 0.0004f;
    velocityKi      = cfg->velocityI * 0.001f;
    velocityKd      = cfg->velocityD * 0.0004f;
    velocityDragKff = cfg->velocityDragCoeff * 0.0001f;

    altitudeKp = cfg->altitudeP * ALTITUDE_P_SCALE;
    altitudeKi = cfg->altitudeI * ALTITUDE_I_SCALE;
    altitudeKd = cfg->altitudeD * ALTITUDE_D_SCALE;
    altitudeKf = cfg->altitudeF * ALTITUDE_F_SCALE;

    maxAngleDeg = cfg->maxAngle;
    if (maxAngleDeg < 5.0f || maxAngleDeg > 80.0f) {
        maxAngleDeg = MAX_ANGLE_DEFAULT;
    }
}

void INIT_CODE autopilotInit(void)
{
    memset(&ap, 0, sizeof(ap));
    memset(&targetPosition, 0, sizeof(targetPosition));
    memset(&targetVelocity, 0, sizeof(targetVelocity));
    memset(&distanceErrorIntegral, 0, sizeof(distanceErrorIntegral));
    memset(&velocityIntegral, 0, sizeof(velocityIntegral));
    memset(&isPosHoldStarting, 0, sizeof(isPosHoldStarting));
    memset(&dTermRamp, 0, sizeof(dTermRamp));
    dTermRamp[EF_EAST] = dTermRamp[EF_NORTH] = 1.0f;

    for (unsigned axis = 0; axis < EF_AXIS_COUNT; axis++) {
        pt2FilterInit(&posAccelLpf[axis], ACCEL_LPF_CUTOFF, POSHOLD_TASK_RATE_HZ);
        pt2FilterInit(&posDtermLpf[axis], VEL_LPF_CUTOFF, POSHOLD_TASK_RATE_HZ);
    }

    autopilotLoadGains();
}

void autopilotResetPositionControl(void)
{
    ap.navActive = false;
    ap.sticksActive = false;
    ap.wasSticksActive = false;
    ap.isPositionHeld = false;
    ap.isAltitudeHeld = false;
    positionEstimatorEnableXY(true);
    initPositionHold();
    resetDistanceError();
    resetVelocityIntegral();
    previousVelocity = *(const vector2_t *)&positionEstimatorGetEstimate()->velocity.v;
    ap.sanityCheckDistance = calculateSanityCheckDistance();
}

bool autopilotPositionControl(void)
{
    return positionControl();
}

void autopilotAltitudeControl(void)
{
    altitudeControl();
}

float getAutopilotAngle(unsigned axis)
{
    if (axis < ANGLE_INDEX_COUNT) {
        return autopilotAngle[axis];
    }
    return 0.0f;
}

float getAutopilotThrottle(void)
{
    return 0.0f; // governor handles throttle; altitude hold uses collective only
}

float getAutopilotCollective(void)
{
    return collectiveOut;
}

void autopilotSetSticksActive(bool active)
{
    ap.wasSticksActive = ap.sticksActive;
    ap.sticksActive = active;
}

void autopilotDisarmCleanup(void)
{
    ap.isPositionHeld = false;
    ap.isAltitudeHeld = false;
    collectiveOut = 0.0f;
    altitudeI = 0.0f;
}

#endif
