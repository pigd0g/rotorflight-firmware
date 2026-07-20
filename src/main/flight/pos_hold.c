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

#include "pg/pos_hold.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"

float posHoldAngle[ANGLE_INDEX_COUNT] = { 0.0f, 0.0f };

typedef struct posHoldState_s {
    bool isEnabled;
    bool isControlOk;
    bool areSensorsOk;
    float deadband;
} posHoldState_t;

static posHoldState_t posHold;

void INIT_CODE posHoldInit(void)
{
    posHold.deadband = posHoldConfig()->deadband * 0.01f;
    autopilotInit();
}

static void posHoldCheckSticks(void)
{
    if (failsafeIsActive()) {
        positionEstimatorSetSticksActive(false);
        autopilotSetSticksActive(false);
        return;
    }

    const bool sticksDeflected =
        (fabsf(getRcDeflection(FD_ROLL)) > posHold.deadband) ||
        (fabsf(getRcDeflection(FD_PITCH)) > posHold.deadband);

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
        return sensors(SENSOR_MAG) || canUseGPSHeading;
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
    UNUSED(currentTimeUs);

    if (FLIGHT_MODE(POS_HOLD_MODE)) {
        if (!posHold.isEnabled) {
            autopilotResetPositionControl();
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
