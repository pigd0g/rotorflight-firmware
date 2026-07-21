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

#include <stdbool.h>
#include <stdint.h>

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"

#if defined(USE_POSITION_HOLD) || defined(USE_ALTITUDE_HOLD)

// Earth-frame axes for local ENU position/velocity estimate.
// These are distinct from body-frame FD_* indices.
typedef enum {
    ENU_EAST = 0,
    ENU_NORTH,
    ENU_UP,
    ENU_AXIS_COUNT
} enuAxis_e;

typedef enum {
    EF_EAST = 0,
    EF_NORTH,
    EF_AXIS_COUNT
} efAxis_e;

typedef struct vector2_s {
    float v[EF_AXIS_COUNT];
} vector2_t;

static inline void vector2Zero(vector2_t *v)
{
    v->v[0] = 0.0f; v->v[1] = 0.0f;
}

static inline float vector2Norm(const vector2_t *v)
{
    return sqrtf(sq(v->v[0]) + sq(v->v[1]));
}

static inline void vector2Scale(vector2_t *out, const vector2_t *in, float scale)
{
    out->v[0] = in->v[0] * scale;
    out->v[1] = in->v[1] * scale;
}

static inline void vector2Add(vector2_t *out, const vector2_t *a, const vector2_t *b)
{
    out->v[0] = a->v[0] + b->v[0];
    out->v[1] = a->v[1] + b->v[1];
}

static inline void vector2Sub(vector2_t *out, const vector2_t *a, const vector2_t *b)
{
    out->v[0] = a->v[0] - b->v[0];
    out->v[1] = a->v[1] - b->v[1];
}

static inline float vector2Dot(const vector2_t *a, const vector2_t *b)
{
    return a->v[0] * b->v[0] + a->v[1] * b->v[1];
}

static inline float vector2Cross(const vector2_t *a, const vector2_t *b)
{
    return a->v[0] * b->v[1] - a->v[1] * b->v[0];
}

// Simple 3-D vector used by the position estimator.
typedef struct vector3_s {
    float v[ENU_AXIS_COUNT];
} vector3_t;

static inline void vector3Zero(vector3_t *v)
{
    v->v[0] = 0.0f; v->v[1] = 0.0f; v->v[2] = 0.0f;
}

static inline float vector3Norm(const vector3_t *v)
{
    return sqrtf(sq(v->v[0]) + sq(v->v[1]) + sq(v->v[2]));
}

static inline void vector3Scale(vector3_t *out, const vector3_t *in, float scale)
{
    out->v[0] = in->v[0] * scale;
    out->v[1] = in->v[1] * scale;
    out->v[2] = in->v[2] * scale;
}

static inline void vector3Add(vector3_t *out, const vector3_t *a, const vector3_t *b)
{
    out->v[0] = a->v[0] + b->v[0];
    out->v[1] = a->v[1] + b->v[1];
    out->v[2] = a->v[2] + b->v[2];
}

static inline void vector3Sub(vector3_t *out, const vector3_t *a, const vector3_t *b)
{
    out->v[0] = a->v[0] - b->v[0];
    out->v[1] = a->v[1] - b->v[1];
    out->v[2] = a->v[2] - b->v[2];
}

// Extract the horizontal (East-North) components of a 3-D vector.
// Replaces the strict-aliasing-UB pattern *(const vector2_t *)&est->position.v.
static inline vector2_t vector3ToVector2XY(const vector3_t *v)
{
    vector2_t out;
    out.v[EF_EAST]  = v->v[ENU_EAST];
    out.v[EF_NORTH] = v->v[ENU_NORTH];
    return out;
}

// Unified position estimate from Kalman-filter sensor fusion.
// All values in local ENU (East-North-Up) centimeters, zeroed at arm point.
typedef struct positionEstimate3d_s {
    vector3_t position;        // cm, ENU
    vector3_t velocity;        // cm/s, ENU
    float trustXY;             // 0-1, derived from KF XY covariance
    float trustZ;              // 0-1, derived from KF Z covariance
    bool isValidXY;            // true if at least one XY measurement source active
    bool isValidZ;             // true if at least one Z measurement source active
} positionEstimate3d_t;

void positionEstimatorInit(void);
void positionEstimatorEnableXY(bool enable);
void positionEstimatorUpdate(timeUs_t currentTimeUs);

const positionEstimate3d_t *positionEstimatorGetEstimate(void);

bool positionEstimatorIsValidXY(void);
bool positionEstimatorIsHeadingRequired(void);

void positionEstimatorSetSticksActive(bool active);
bool positionEstimatorIsSticksActive(void);

void getLinearAccelENU(float *accelEast, float *accelNorth, float *accelUp);

#endif
