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

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/pos_hold.h"

PG_REGISTER_WITH_RESET_TEMPLATE(posHoldConfig_t, posHoldConfig, PG_POSHOLD_CONFIG, 0);

PG_RESET_TEMPLATE(posHoldConfig_t, posHoldConfig,
    .deadband = 5,
    .positionSource = POSHOLD_SOURCE_AUTO,
    .minSats = 12,
    .headingRequired = 1,
    .opticalflowQualityMin = 30,
    .opticalflowMaxRange = 400,
    .gpsValidityTimeout = 5,
);

#endif
