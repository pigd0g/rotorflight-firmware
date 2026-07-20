/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "fc/runtime_config.h"
#include "io/beeper.h"

uint8_t armingFlags = 0;
uint8_t stateFlags = 0;
uint16_t flightModeFlags = 0;

static uint32_t enabledSensors = 0;

// Must be no longer than OSD_WARNINGS_MAX_SIZE (11) to be displayed fully in OSD
// Order MUST match the bit positions in armingDisableFlags_e (runtime_config.h).
// Index i corresponds to bit i; consumer code (cli.c, osd_warnings.c) looks
// up names by bit position.
const char *armingDisableFlagNames[]= {
    "NOGYRO",        // bit 0
    "FAILSAFE",      // bit 1
    "RXLOSS",        // bit 2
    "BADRX",         // bit 3
    "BOXFAILSAFE",   // bit 4
    "RUNAWAY",       // bit 5
    "CRASH",         // bit 6
    "THROTTLE",      // bit 7
    "ANGLE",         // bit 8
    "BOOTGRACE",     // bit 9
    "NOPREARM",      // bit 10
    "LOAD",          // bit 11
    "CALIB",         // bit 12
    "CLI",           // bit 13
    "CMS",           // bit 14
    "BST",           // bit 15
    "MSP",           // bit 16
    "PARALYZE",      // bit 17
    "GPS",           // bit 18
    "RESCUE_SW",     // bit 19
    "RPMFILTER",     // bit 20
    "REBOOT_REQD",   // bit 21
    "DSHOT_BBANG",   // bit 22
    "NO_ACC_CAL",    // bit 23
    "MOTOR_PROTO",   // bit 24
    "OVERRIDE",      // bit 25
    "ALTHOLD",       // bit 26
    "POSHOLD",       // bit 27
    "ARMSWITCH",     // bit 28
};

static armingDisableFlags_e armingDisableFlags = 0;

void setArmingDisabled(armingDisableFlags_e flag)
{
    armingDisableFlags = armingDisableFlags | flag;
}

void unsetArmingDisabled(armingDisableFlags_e flag)
{
    armingDisableFlags = armingDisableFlags & ~flag;
}

bool isArmingDisabled(void)
{
    return armingDisableFlags;
}

armingDisableFlags_e getArmingDisableFlags(void)
{
    return armingDisableFlags;
}

/**
 * Enables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
uint16_t enableFlightMode(flightModeFlags_e mask)
{
    uint16_t oldVal = flightModeFlags;

    flightModeFlags |= (mask);
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

/**
 * Disables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
uint16_t disableFlightMode(flightModeFlags_e mask)
{
    uint16_t oldVal = flightModeFlags;

    flightModeFlags &= ~(mask);
    if (flightModeFlags != oldVal)
        beeperConfirmationBeeps(1);
    return flightModeFlags;
}

bool sensors(uint32_t mask)
{
    return enabledSensors & mask;
}

void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}

void sensorsClear(uint32_t mask)
{
    enabledSensors &= ~(mask);
}

uint32_t sensorsMask(void)
{
    return enabledSensors;
}
