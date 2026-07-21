# Altitude Hold

Altitude hold is the vertical-axis companion to position hold. When activated it
captures the current altitude as a target and uses a PID to drive a collective
offset that holds the rotorcraft at that altitude. The pilot can climb or
descend with the collective stick; on stick return the craft holds the new
altitude.

This page documents the CLI parameters for initial configuration. For the
internal architecture, see `betaflight-gps-hold.md` at the repository root.

## Relationship to other modes

- **`ALT_HOLD`** (BOXALTHOLD) is co-active with **`POS_HOLD`**. A pilot can
  enable both switches and have the craft station-keep in 3-D; the controller
  will hold position and altitude independently.
- **`GPS RESCUE`** runs its own altitude logic; alt-hold is suppressed while
  rescue is active.
- **`AUTOPILOT`** activates alt-hold underneath it so the mission controls the
  vertical axis via a navigator-supplied target altitude.
- Throttle is **not** driven by the autopilot. The governor (or the pilot) sets
  the engine/motor RPM; the altitude controller trims **collective** to hold
  altitude. There are no `ap_*_throttle` parameters.

## Enabling

Altitude hold requires an altitude source (barometer, GPS, or rangefinder) and
a valid heading source under POS_HOLD. The relevant sensor parameters live in
the `position` parameter group (`altitude_source` and friends) and the GPS
configuration in [docs/Gps.md](Gps.md).

Assign `ALT_HOLD` to an aux channel in the Modes tab. Throttle must be raised
once after arming before alt-hold will engage.

## How alt-hold captures the hover point

On the rising edge of `ALT_HOLD`:

1. The current altitude is captured as the hold target.
2. The current collective stick position is sampled. If the stick is within
   `ap_alt_hold_deadband` of center, that value is used as the hover
   collective. Otherwise the configured `ap_hover_collective` is used.
3. From that point on, the controller adds a small collective offset on top of
   the hover value to track the hold target.

This means the default `ap_hover_collective` is used only when the collective
stick is not centered at the moment alt-hold engages. For a clean engage, leave
the collective near center and let the controller capture the live value.

## CLI parameter reference

Parameters are read/written with `get` and `set` in the CLI, for example
`set ap_altitude_p = 30`. Changes are saved with `save`.

### Altitude PID

| Parameter | Type | Range | Default | Description |
| --- | --- | --- | --- | --- |
| `ap_altitude_p` | uint8 | 0..255 | `30` | P gain on altitude error (cm). Higher pulls harder toward the target altitude. |
| `ap_altitude_i` | uint8 | 0..255 | `30` | I gain on altitude error. Provides the steady-state trim for rotor RPM variation, weight change, and small sensor biases. |
| `ap_altitude_d` | uint8 | 0..255 | `30` | D gain on vertical speed (vario). Provides damping; reduce if the craft bobs at the hold altitude. |
| `ap_altitude_f` | uint8 | 0..255 | — | Feedforward on target vertical speed. Currently unused by the basic alt-hold loop; reserved for mission profiles that supply a target climb/sink rate. |

### Hover collective

| Parameter | Type | Range | Default | Description |
| --- | --- | --- | --- | --- |
| `ap_hover_collective` | uint16 | 0..1000 | `500` | Default hover collective when the collective stick is not centered at alt-hold entry. The range 0..1000 maps to -1..+1 deflection; 500 is the geometric center. |
| `ap_collective_min` | int16 | -1000..1000 | `-300` | Minimum collective offset below hover that the controller is allowed to command (deflection × 1000). Prevents the controller from driving the collective all the way to zero throttle. |
| `ap_collective_max` | int16 | -1000..1000 | `300` | Maximum collective offset above hover that the controller is allowed to command. Caps the climb authority. |
| `ap_alt_hold_deadband` | uint16 | 0..1000 | `50` | Collective stick deadband (deflection × 1000) used at alt-hold entry to decide whether to capture the live stick as the hover collective. Larger values make the controller more willing to use the live value; smaller values force it back to `ap_hover_collective`. |

## Initial configuration

A safe starting point for a typical collective-driven helicopter (governor
holding RPM, barometer for altitude):

```
set altitude_source = DEFAULT       # baro+GPS blend
set ap_altitude_p = 30
set ap_altitude_i = 30
set ap_altitude_d = 30
set ap_altitude_f = 0
set ap_hover_collective = 500
set ap_collective_min = -300
set ap_collective_max = 300
set ap_alt_hold_deadband = 50
save
```

After saving, hover the craft by hand with the collective near center, then
flip the `ALT HOLD` switch. The controller will capture the live collective
and the current altitude, and the craft should hold position. Tap the
collective stick to nudge altitude; it should return to the new altitude on
release.

## Tuning notes

- **Vertical bobbing (fast)**: reduce `ap_altitude_d` and (slightly)
  `ap_altitude_p`.
- **Vertical bobbing (slow)**: reduce `ap_altitude_i`. A small amount of slow
  drift is acceptable; an oscillating integral is too much I.
- **Sag on engage (altitude drops before recovering)**: the captured hover
  collective is too low. Either re-engage with the collective stick slightly
  higher, or raise `ap_hover_collective`.
- **Climb/overshoot on engage (altitude rises before recovering)**: the
  captured hover collective is too high. Re-engage with the stick slightly
  lower, or lower `ap_hover_collective`.
- **The craft will not climb far enough**: the controller is hitting the
  `ap_collective_max` clamp. Raise the clamp; the value is a collective offset
  from hover, so a larger magnitude allows a larger authority window.
- **Collective feels sluggish near the limits**: the integral is wound to the
  clamp. The I-relax term reduces the I gain when the altitude error is large
  (>200 cm) so the integral does not wind up during aggressive climbs.
- **Diving on stick inputs**: the alt-hold PID runs every cycle and immediately
  resumes targeting the old altitude. To fly down deliberately, the pilot
  must move the stick and **leave it deflected**; the controller does not
  reinterpret stick input as a new altitude command — it just lets the craft
  descend until the stick is released, then re-engages hold at the new
  altitude. If the craft dives further than expected, the integrator may be
  saturated; back off `ap_collective_min` to allow more down authority.

## Removed parameters

Several legacy parameters from the previous throttle-coupled altitude hold
are no longer present in the CLI:

- `ap_alt_hold_min_throttle`, `ap_alt_hold_max_throttle` — replaced by
  `ap_collective_min` / `ap_collective_max`.
- `ap_landing_altitude_m` — landing is no longer part of the alt-hold path.

If you are upgrading from a configuration that used any of these, the values
are not migrated. Re-tune from the defaults above and verify both that the
craft hovers in mid-stick and that it can climb and descend to the limits of
the collective travel.

## See also

- [docs/PosHold.md](PosHold.md) — horizontal-axis position hold parameters.
- [docs/Gps.md](Gps.md) — GPS setup (one of the altitude sources).
- [docs/Governor.md](Governor.md) — the governor holds RPM, which is what
  alt-hold trims against.
- [docs/Modes.md](Modes.md) — flight mode aux-channel assignment.
- `betaflight-gps-hold.md` (repository root) — full developer reference for
  the autopilot stack.
