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

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/autopilot.h"

PG_REGISTER_WITH_RESET_TEMPLATE(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT_CONFIG, 0);

PG_RESET_TEMPLATE(autopilotConfig_t, autopilotConfig,
    .positionP  = 30,
    .positionI  = 30,
    .positionD  = 30,
    .positionA  = 30,
    .positionCutoff = 30,
    .stopThreshold  = 10,
    .maxAngle   = 50,

    .velocityControlEnable = 1,
    .velocityP  = 50,
    .velocityI  = 10,
    .velocityD  = 5,
    .velocityDragCoeff = 50,
    .maxVelocity = 1000,
    .velocityBuildupMaxPitch = 8,

    .altitudeP  = 30,
    .altitudeI  = 30,
    .altitudeD  = 30,
    .altitudeF  = 30,
    .hoverCollective = 500,
    .collectiveMin = -300,
    .collectiveMax =  300,
    .altHoldMinThrottle = 1050,
    .altHoldMaxThrottle = 1400,
    .landingAltitudeM = 4,

    .waypointArrivalRadius = 500,
    .waypointHoldRadius = 200,
    .stickDeadband = 50,
    .altHoldDeadband = 50,
    .yawMode = YAW_MODE_VELOCITY,
    .yawP = 50,
    .yawD = 10,
    .maxYawRate = 30,
    .minForwardVelocity = 100,
);

#endif
