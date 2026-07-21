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

#ifdef USE_POSITION_HOLD

#include <stdbool.h>
#include <stdint.h>

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"

#include "config/config.h"

#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/position.h"
#include "flight/position_estimator.h"
#include "flight/pos_hold.h"
#include "flight/autopilot.h"

#include "io/gps.h"

#include "pg/pos_hold.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"

float posHoldAngle[ANGLE_INDEX_COUNT] = { 0.0f, 0.0f };

// Must match imu.c:87 — minimum ground speed (cm/s) for GPS course-over-ground
// to be considered a usable heading source. Kept here as a local constant so
// pos_hold.c is self-contained; if imu.c ever changes its threshold, update
// this to match.
#define POSHOLD_GPS_COG_MIN_GROUNDSPEED 500
#define POSHOLD_GPS_MIN_SATS_FOR_COG    5

typedef struct posHoldState_s {
    bool isEnabled;
    bool isControlOk;
    bool areSensorsOk;
    // Latched: becomes true the first time GPS COG becomes usable (mag healthy
    // OR groundSpeed >= POSHOLD_GPS_COG_MIN_GROUNDSPEED with a fix and enough
    // sats). Stays true until both mag and GPS fix are lost entirely. Prevents
    // PH from disengaging moment-to-moment when ground speed dips below the
    // COG threshold (e.g. wind gusts), which would otherwise toggle PH off
    // and on in flight.
    bool gpsHeadingEverValid;
} posHoldState_t;

static posHoldState_t posHold;

void INIT_CODE posHoldInit(void)
{
    posHold.gpsHeadingEverValid = false;
    autopilotInit();
}

static void posHoldCheckSticks(void)
{
    if (failsafeIsActive()) {
        positionEstimatorSetSticksActive(false);
        autopilotSetSticksActive(false);
        return;
    }

    // Read deadband from config each cycle so CLI changes take effect
    // without requiring a reboot.
    const float deadband = posHoldConfig()->deadband * 0.01f;
    const bool sticksDeflected =
        (fabsf(getRcDeflection(FD_ROLL)) > deadband) ||
        (fabsf(getRcDeflection(FD_PITCH)) > deadband);

    positionEstimatorSetSticksActive(sticksDeflected);
    autopilotSetSticksActive(sticksDeflected);
}

static bool posHoldSensorsOk(void)
{
    if (!positionEstimatorIsValidXY()) {
        return false;
    }

    if (!isUpright()) {
        return false;
    }

    if (posHoldConfig()->headingRequired && positionEstimatorIsHeadingRequired()) {
        // GPS gives absolute ENU measurements; a bad yaw in the body->EF
        // rotation will mis-rotate the correction and cause a flyaway. We
        // therefore require either a healthy mag or a GPS course-over-ground
        // that imu.c considers usable (STATE(GPS_FIX), >=5 sats, and
        // groundSpeed >= GPS_COG_MIN_GROUNDSPEED).
        const bool magOk = sensors(SENSOR_MAG);
        const bool gpsCogNowOk =
            sensors(SENSOR_GPS) && STATE(GPS_FIX) &&
            gpsSol.numSat >= POSHOLD_GPS_MIN_SATS_FOR_COG &&
            gpsSol.groundSpeed >= POSHOLD_GPS_COG_MIN_GROUNDSPEED;

        // Latch: once COG (or mag) has ever been usable, stay valid until
        // both mag and GPS fix are lost entirely. Prevents PH from toggling
        // off when groundSpeed dips below the COG threshold momentarily.
        if (magOk || gpsCogNowOk) {
            posHold.gpsHeadingEverValid = true;
        } else if (!sensors(SENSOR_GPS) || !STATE(GPS_FIX)) {
            // Total GPS loss: drop the latch so we re-validate when GPS returns.
            posHold.gpsHeadingEverValid = false;
        }
        return posHold.gpsHeadingEverValid;
    }

    return true;
}

static bool altHoldSensorsOk(void)
{
#ifdef USE_ALTITUDE_HOLD
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    if (est->isValidZ) {
        return true;
    }
    return sensors(SENSOR_BARO) && baroIsReady();
#else
    return true;
#endif
}

void updatePosHold(timeUs_t currentTimeUs)
{
    // Run the position estimator at the start of this task so the controller
    // and the estimator share a single task slot and cannot race each other
    // (previously the estimator ran in TASK_ALTITUDE and the controller in
    // TASK_POSHOLD, with no synchronization between them — see C5 in
    // code-review.md). Only run when at least one of PH/ALTHOLD is active to
    // save CPU when neither is in use.
    const bool anyAutopilotMode = FLIGHT_MODE(POS_HOLD_MODE) || FLIGHT_MODE(ALTHOLD_MODE);
    if (anyAutopilotMode) {
        positionEstimatorUpdate(currentTimeUs);
    }

    if (FLIGHT_MODE(POS_HOLD_MODE)) {
        if (!posHold.isEnabled) {
            autopilotResetPositionControl();
            // Reset the heading-valid latch on (re)engagement so we require
            // fresh confirmation that the current yaw is trustworthy before
            // letting PH command angles. The craft may have been moved or the
            // GPS may have dropped between flights.
            posHold.gpsHeadingEverValid = false;
            posHold.isControlOk = true;
            posHold.isEnabled = true;
        }
    } else {
        if (posHold.isEnabled) {
            positionEstimatorSetSticksActive(false);
            autopilotSetSticksActive(false);
            positionEstimatorEnableXY(false);
        }
        posHold.isEnabled = false;
    }

    if (posHold.isEnabled) {
        posHoldCheckSticks();
        posHold.areSensorsOk = posHoldSensorsOk();

        if (posHold.areSensorsOk) {
            posHold.isControlOk = autopilotPositionControl();
        } else {
            posHold.isControlOk = false;
        }

        for (unsigned i = 0; i < ANGLE_INDEX_COUNT; i++) {
            posHoldAngle[i] = getAutopilotAngle(i);
        }
    } else {
        for (unsigned i = 0; i < ANGLE_INDEX_COUNT; i++) {
            posHoldAngle[i] = 0.0f;
        }
    }

#ifdef USE_ALTITUDE_HOLD
    if (FLIGHT_MODE(ALTHOLD_MODE)) {
        if (altHoldSensorsOk()) {
            autopilotAltitudeControl();
        } else {
            // Sensors lost mid-hold: freeze collective at the captured hover
            // trim instead of leaving the last (possibly large) PID output
            // applied as the collective setpoint. Prevents uncommanded
            // climb/descent while the OSD "ALTHOLD FAIL" warning is shown.
            autopilotFreezeCollectiveAtHover();
        }
    }
#endif
}

bool isPosHoldInControl(void)
{
    return posHold.isEnabled && posHold.isControlOk && posHold.areSensorsOk;
}

bool posHoldFailure(void)
{
    return FLIGHT_MODE(POS_HOLD_MODE) && (!posHold.isControlOk || !posHold.areSensorsOk);
}

bool altHoldFailure(void)
{
    return FLIGHT_MODE(ALTHOLD_MODE) && !altHoldSensorsOk();
}

#endif
