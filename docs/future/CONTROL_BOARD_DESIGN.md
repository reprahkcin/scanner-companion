# MKII Scanner Control Board Design

Status: Future (planning) — supersedes the LinuxCNC-split approach in
`LINUXCNC_INTEGRATION_PLAN.md` for motion control.

## Architecture (decided)

```
Pi 5 "scannerMKII" ──USB serial──► Arduino Mega 2560 ──step/dir/en──► 6× drivers ──► motors
  scanner-companion                  motor controller                  DM542 / TB6600
  kinematics + planner               - 6× step/dir/enable (per-axis)
  Pi HQ camera (ribbon)              - per-axis home inputs
  XMP + focus stacking               - motor-bus relay + E-stop
                                     - LED lighting PWM
                                     - (optional) DSLR trigger
```

Key decisions:
- **Host:** Raspberry Pi 5 — runs scanner-companion, camera, planning, kinematics.
- **Motion controller:** single Arduino Mega 2560 (5 V logic → drives DM542/TB6600
  opto-inputs directly; 54 I/O = ample headroom).
- **No coordinated/interpolated motion.** Each capture is move → settle → shoot, so
  axes can move sequentially or fire-and-wait. Firmware extends the existing
  `arduino/scanner_controller` serial protocol from 3 to 6 axes.
- **Drivers:** classic external step/dir (DM542 preferred for smoothness; TB6600 budget).
- **Connectors:** 4× panel-mount 5-pin DIN (Sherline motors) + 2× screw/JST (B, R).
- **Enable:** per-axis (de-energize idle motors to cut heat/noise/drift).
- LinuxCNC rig drops out of the motion path.

## Axes (from profiles/sherline_6axis.json)

| Axis | Motor | Frame | Driver (TBD by current) | Connector |
|------|-------|-------|-------------------------|-----------|
| X | Sherline | NEMA 23 | DM542 | 5-pin DIN |
| Y | Sherline | NEMA 23 | DM542 | 5-pin DIN |
| Z | Sherline | NEMA 23 | DM542 | 5-pin DIN |
| A | Sherline (4th axis) | NEMA 23 | DM542 | 5-pin DIN |
| B | camera tilt | medium | DM542 / TB6600 | screw/JST |
| R | focus rail | small | small digital (DM320T-class) | screw/JST |

## Proposed Mega 2560 pin map

Motor signals use the contiguous 22–39 block (tidy ribbon layout):

| Axis | STEP | DIR | ENABLE |
|------|------|-----|--------|
| X | 22 | 23 | 24 |
| Y | 25 | 26 | 27 |
| Z | 28 | 29 | 30 |
| R | 31 | 32 | 33 |
| A | 34 | 35 | 36 |
| B | 37 | 38 | 39 |

I/O:

| Function | Pin(s) | Notes |
|----------|--------|-------|
| Home/limit inputs (X,Y,Z,R,A,B) | 40,41,42,43,44,45 | `INPUT_PULLUP`, polled during homing |
| E-stop input | 2 | interrupt-capable; ISR halts all motion |
| Motor-bus relay/contactor | A0 (54) | active-LOW (reuse existing relay convention) |
| LED lighting (PWM) | 9 (+10 for 2nd ch) | MOSFET-driven |
| DSLR trigger (optional) | 11 (focus), 12 (shutter) | only if not using Pi HQ camera |
| USB serial to Pi | 0/1 (Serial0) | reserved |
| I2C / SPI | 20/21, 50–52 | reserved for future (display, TMC, etc.) |

Pin budget: 18 (motors) + 6 (home) + ~5 (I/O) ≈ 29 of 54 — lots of room.

## Driver notes

- **DM542 enable polarity:** confirm against the TB6600 quirk documented in
  `arduino/TB6600_ENABLE_LOGIC.md` (ENA+ to 5 V, active-HIGH). DM542 opto inputs
  behave similarly; verify per datasheet before wiring.
- Microstepping (DIP) per axis sets the profile's `steps_per_unit`; pick per the
  resolution each axis needs and update the profile to match.
- Mega 5 V step/dir into DM542/TB6600 opto inputs: use the recommended series
  resistor (often ~1 kΩ) if the driver expects current-limited 5 V inputs.

## Power & safety

- Single motor PSU (voltage/current TBD — see open items). Higher voltage = more
  NEMA-23 torque at speed; 36–48 V common for DM542.
- **E-stop cuts motor power** via the relay/contactor on the driver V+ bus, *and*
  signals the Mega (pin 2) to halt stepping. Logic (Mega/Pi) stays powered.
- Fuse the motor bus; common ground between logic and driver signal grounds.

## Software impact

- Firmware: extend `arduino/scanner_controller` to 6 axes (AccelStepper per axis,
  per-axis enable, homing for axes with switches). Reuse the existing serial
  command set (`MOVE`/`ROTATE`/`GET_POS`/`SET_SPEED`/`ZERO`/`RELEASE`/`STATUS`).
- Profile: update `profiles/sherline_6axis.json` `controllers` /
  `axis_controller_map` so all 6 axes point at one `arduino_serial` (Mega), drop
  the `linuxcnc` controller.

## Open items (need from hardware)

1. **Motor currents** — A/phase for the 4 Sherline NEMA-23s, B, and R → sets
   driver DIP current + PSU current budget. Confirm 200 steps/rev, 4-wire bipolar.
2. **5-pin DIN pinout** — which pins are coil A+/A−/B+/B− (+ shield). See
   multimeter procedure (below / to be added).
3. **Power supply** — on hand? voltage and current.
4. **Homing** — which axes get switches (R confirmed; recommend X/Y/Z/B too;
   A may use an index or none).
5. **Microstepping** per axis (resolution vs speed) → driver DIP + profile update.
6. **Camera** — confirm Pi HQ camera (host-triggered) so MCU trigger stays optional.
