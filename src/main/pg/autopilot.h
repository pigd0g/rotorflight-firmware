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

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "pg/pg.h"

typedef struct autopilotConfig_s {
    uint8_t  positionP;
    uint8_t  positionI;
    uint8_t  positionD;
    uint8_t  positionA;
    uint8_t  maxAngle;

    uint16_t maxVelocity;

    uint8_t  altitudeP;
    uint8_t  altitudeI;
    uint8_t  altitudeD;
    uint8_t  altitudeF;
    uint16_t hoverCollective;      // 0..1000 collective value used as hover baseline
    int16_t  collectiveMin;        // min allowed collective delta from hover (unitless deflection × 1000)
    int16_t  collectiveMax;        // max allowed collective delta from hover (unitless deflection × 1000)
    uint16_t altHoldDeadband;        // collective stick deadband for altitude-hold hover capture (0..1000)
} autopilotConfig_t;

PG_DECLARE(autopilotConfig_t, autopilotConfig);
