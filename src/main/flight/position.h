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

#include "flight/position_estimator.h"

void positionInit(void);
void positionUpdate(void);

#ifdef USE_POSITION_HOLD
void positionEstimatorInit(void);
void positionEstimatorEnableXY(bool enable);
const positionEstimate3d_t *positionEstimatorGetEstimate(void);
bool positionEstimatorIsValidXY(void);
bool positionEstimatorIsHeadingRequired(void);
void positionEstimatorSetSticksActive(bool active);
bool positionEstimatorIsSticksActive(void);
#endif

float getAltitude(void);
float getVario(void);

int getEstimatedAltitudeCm(void);
int getEstimatedVarioCms(void);
