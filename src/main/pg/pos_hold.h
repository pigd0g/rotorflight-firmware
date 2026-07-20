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

typedef enum {
    POSHOLD_SOURCE_AUTO = 0,
    POSHOLD_SOURCE_GPS_ONLY,
    POSHOLD_SOURCE_OPTICALFLOW_ONLY,
} posHoldSource_e;

typedef struct posHoldConfig_s {
    uint8_t  deadband;
    uint8_t  positionSource;
    uint8_t  minSats;
    uint8_t  headingRequired;       // require mag or GPS heading for position hold
    uint16_t opticalflowQualityMin;
    uint16_t opticalflowMaxRange;
} posHoldConfig_t;

PG_DECLARE(posHoldConfig_t, posHoldConfig);
