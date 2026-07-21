# GPS Position Hold

GPS Position Hold is a station-keeping autopilot. When activated it captures
the current horizontal position and uses a Kalman estimate of position and
velocity (fused from GPS and IMU) to drive the angle-mode PID so the craft
holds its position. Pilot stick inputs nudge the hold target; on stick return
the craft holds the new position.

This page documents the CLI parameters exposed for the initial configuration.

## Modes that use the position hold stack

- **`POS_HOLD`** (BOXPOSHOLD) — station keeping with stick nudging.

`ANGLE` is force-enabled whenever POS_HOLD, ALT_HOLD, or failsafe is active, so
the craft self-levels under any of these modes.

`GPS RESCUE` (BOXGPSRESCUE) is a separate failsafe return-to-home mode that
runs its own controller and does **not** share the position-hold angle targets
or estimator.

## Enabling

Position hold requires a GPS position source and a valid heading source. Set
up GPS first (see [docs/Gps.md](Gps.md)) and verify it acquires a fix before
tuning the parameters below.

Assign `POS_HOLD` to an aux channel in the Modes tab. Throttle must be raised
once after arming before position hold will engage.

## CLI parameter reference

Parameters are read/written with `get` and `set` in the CLI, for example
`set pos_hold_deadband = 5`. Changes are saved with `save`.

### Sensor and source selection

| Parameter | Type | Range | Default | Description |
| --- | --- | --- | --- | --- |
| `pos_hold_position_source` | enum | `AUTO`, `GPS_ONLY`, `OPTICALFLOW_ONLY` | `AUTO` | Which position sensor(s) feed the position-hold estimator. `AUTO` and `GPS_ONLY` both use GPS. `OPTICALFLOW_ONLY` is **reserved** — no optical-flow sensor feeding is implemented yet; selecting it disables GPS XY correction and position hold will not engage. |
| `pos_hold_min_sats` | uint8 | 3..50 | `12` | Minimum number of GPS satellites required for a fix to be trusted by the position estimator. |
| `pos_hold_heading_required` | enum | `OFF`, `ON` | `ON` | When `ON`, position hold is gated on a valid heading (mag or GPS course-over-ground). Leave `ON` whenever GPS is in the loop to prevent flyaways from a bad yaw. |
| `pos_hold_opticalflow_quality_min` | uint16 | 0..1000 | `30` | **Reserved.** Minimum optical flow quality for flow acceptance. No optical-flow code is implemented. |
| `pos_hold_opticalflow_max_range` | uint16 | 0..1000 | `400` | **Reserved.** Maximum usable range for optical flow scaling. No optical-flow code is implemented. |
| `pos_hold_gps_validity_timeout` | uint8 | 1..50 | `5` | Consecutive missed GPS cycles (in tenths of a second) before the estimator clears `isValidXY` and position hold disengages. A small value drops position hold quickly on GPS loss; a large value rides out short dropouts at the cost of a delayed abort. |

### Pilot input

| Parameter | Type | Range | Default | Description |
| --- | --- | --- | --- | --- |
| `pos_hold_deadband` | uint8 | 0..50 | `5` | Stick deadband (percent of full deflection) below which the controller treats the sticks as centered and freezes the hold target. Higher values are more forgiving of small stick trim. Read each cycle, so CLI changes take effect without a reboot. |

### Position PID

The position PID runs the station-keeping loop.

| Parameter | Type | Range | Default | Description |
| --- | --- | --- | --- | --- |
| `ap_position_p` | uint8 | 0..255 | `30` | P gain on (pseudo) distance error. Higher pulls harder toward the hold target. |
| `ap_position_i` | uint8 | 0..255 | `30` | I gain on the integrated distance error. Provides the steady-state trim against wind and small tilt biases. |
| `ap_position_d` | uint8 | 0..255 | `30` | D gain on velocity error. Provides damping and the braking force on entry. |
| `ap_position_a` | uint8 | 0..255 | `30` | Acceleration feedforward. Improves response to fast position changes; reduce if you see overshoot on quick stick inputs. |

### Output limits

| Parameter | Type | Range | Default | Description |
| --- | --- | --- | --- | --- |
| `ap_max_angle` | uint8 | 5..80 | `50` | Maximum lean angle (degrees) the controller is allowed to command in any direction. The hard clamp on autopilot output. Lower for beginner or indoor craft. |
| `ap_max_velocity` | uint16 | 50..1000 | `1000` | Maximum horizontal speed (cm/s) the controller may command when nudging with the sticks. Caps how fast the hold target can be moved by stick input. |

## Initial configuration

A safe starting point for a typical multirotor with a UBLOX GPS and mag
enabled:

```
feature GPS
set gps_provider = UBLOX
set gps_auto_config = ON
set mag_hardware = AUTO            # or your specific mag type
set align_board_yaw = 0            # or your board orientation
set pos_hold_position_source = AUTO
set pos_hold_min_sats = 8
set pos_hold_heading_required = ON
set pos_hold_deadband = 5
set pos_hold_gps_validity_timeout = 5
set ap_position_p = 30
set ap_position_i = 30
set ap_position_d = 30
set ap_position_a = 30
set ap_max_angle = 50
set ap_max_velocity = 1000
save
```

After saving, assign `POS HOLD` to an aux channel, arm, take off, raise the
collective, and flip the switch. The craft should hold its current position
within a couple of meters. Use the OSD "pos hold fail" warning to confirm the
mode is healthy.

## Tuning notes

- **First flights**: keep the defaults. Verify that POS_HOLD engages cleanly
  in still air. Watch for any oscillation around the hold point — small fast
  oscillation means P is too high or D too low; slow drift means I is too low.
- **Position oscillation**: reduce `ap_position_p` and `ap_position_a` together.
  If the craft overshoots on stick inputs, reduce `ap_position_d`.
- **Wind drift**: increase `ap_position_i` until the drift stops. If the craft
  starts to pendulum in wind, you have gone too far — back off slightly.
- **Aggressive stick tracking**: increase `ap_max_velocity` (faster hold-point
  walk) and `ap_position_d` (more braking at the new point). Keep
  `ap_max_angle` as the final authority on what the craft is allowed to do.
- **GPS dropouts**: if the OSD shows `pos hold fail` during brief GPS losses
  (tall buildings, RF interference), raise `pos_hold_gps_validity_timeout`
  (e.g. 10..20) to ride out the dropout. Lower it (e.g. 2..3) for fail-fast
  behavior on a hard GPS failure.

## Removed parameters

Several parameters and features from earlier revisions of the position hold
stack are no longer present in the CLI or the codebase:

- Velocity PID and drag feedforward: `ap_velocity_control_enable`,
  `ap_velocity_p`, `ap_velocity_i`, `ap_velocity_d`, `ap_velocity_drag_coeff`,
  `ap_velocity_buildup_max_pitch`.
- Startup shaping: `ap_position_cutoff`, `ap_stop_threshold`.
- Throttle-coupled altitude hold: `ap_alt_hold_min_throttle`,
  `ap_alt_hold_max_throttle`, `ap_landing_altitude_m`.
- Waypoint navigation: `ap_waypoint_arrival_radius`, `ap_waypoint_hold_radius`.
- Pilot stick shaping: `ap_stick_deadband` (replaced by `pos_hold_deadband`).
- Mission yaw: `ap_yaw_mode`, `ap_yaw_p`, `ap_yaw_d`, `ap_max_yaw_rate`,
  `ap_min_forward_velocity`.
- `getAutopilotThrottle()` API (always returned 0; governor handles throttle).

If you are upgrading from a configuration that used any of these, the values
are not migrated. Re-tune from the defaults above.

## See also

- [docs/Gps.md](Gps.md) — GPS hardware setup, providers, and SBAS.
- [docs/AltHold.md](AltHold.md) — altitude hold (vertical axis) parameters.
- [docs/Modes.md](Modes.md) — flight mode aux-channel assignment.
