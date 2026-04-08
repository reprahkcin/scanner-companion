# Scanner Controller MKII - Adafruit Motor Shield v2.3

Arduino firmware for the MKII scanner using an Adafruit Motor Shield v2.3.

## Hardware Configuration

| Shield Terminal | Function | Motor |
|-----------------|----------|-------|
| M1 + M2 | Rotary table stepper | 4-wire bipolar stepper |
| M3 + M4 | Camera rail stepper | 4-wire bipolar stepper |

### Wiring

Connect your bipolar stepper motors to the shield terminals:

**Stepper 1 (Rotary Table) - M1/M2:**
- Coil A: M1 terminals (A+ and A-)
- Coil B: M2 terminals (B+ and B-)

**Stepper 2 (Camera Rail) - M3/M4:**
- Coil A: M3 terminals (A+ and A-)
- Coil B: M4 terminals (B+ and B-)

### Power

- **Motor Power**: Connect external power supply (12V-24V depending on your steppers) to the shield's power terminals
- **Logic Power**: USB from Raspberry Pi
- **Jumper**: Remove the VIN jumper if using external motor power > 12V

## Required Libraries

Install via Arduino Library Manager:

1. **Adafruit Motor Shield V2 Library**
2. **Adafruit BusIO** (dependency, auto-installed)

Or install via Arduino CLI:
```bash
arduino-cli lib install "Adafruit Motor Shield V2 Library"
```

## Serial Protocol

Baud rate: **115200**

### Commands

| Command | Description | Example |
|---------|-------------|---------|
| `ROTATE <motor> <deg> <CW\|CCW>` | Rotate motor 1 | `ROTATE 1 90 CW` |
| `MOVE <motor> <mm> <FORWARD\|BACKWARD>` | Move motor 2 | `MOVE 2 10.5 FORWARD` |
| `ZERO <motor>` | Reset position counter | `ZERO 1` |
| `GET_POS <motor>` | Get current position | `GET_POS 2` |
| `SET_SPEED <motor> <rpm>` | Set motor speed | `SET_SPEED 1 45` |
| `GET_SPEED <motor>` | Get current speed | `GET_SPEED 1` |
| `RELEASE <motor>` | De-energize motor coils | `RELEASE 1` |
| `RELEASE_ALL` | Release all motors | `RELEASE_ALL` |
| `STATUS` | Show motor status | `STATUS` |

### Compatibility Notes

- `POWER ON/OFF` and `GET_POWER` are accepted for MKI protocol compatibility but have no effect (shield is always powered when Arduino is on)
- `TILT` command returns an error (MKII is 2-axis only)

## Calibration

### Steps Per Degree (Motor 1 - Rotary)

Default assumes 200-step motor with INTERLEAVE (half-step) mode = 400 steps/rev:
```cpp
const float STEPS_PER_DEGREE_M1 = 400.0 / 360.0;  // ~1.111 steps/degree
```

If you have a gear reduction (e.g., 10:1):
```cpp
const float STEPS_PER_DEGREE_M1 = (400.0 * 10.0) / 360.0;  // 11.11 steps/degree
```

### Steps Per MM (Motor 2 - Rail)

Depends on your lead screw pitch:

| Lead Screw | Steps/Rev (INTERLEAVE) | Steps/mm |
|------------|------------------------|----------|
| 8mm lead | 400 | 50 |
| 4mm lead | 400 | 100 |
| 2mm lead | 400 | 200 |

Adjust `STEPS_PER_MM_M2` accordingly.

### Stepping Styles

The `STEP_STYLE` constant controls motor behavior:

| Style | Steps/Rev | Torque | Smoothness | Power |
|-------|-----------|--------|------------|-------|
| `SINGLE` | 200 | Low | Rough | Low |
| `DOUBLE` | 200 | High | Rough | High |
| `INTERLEAVE` | 400 | Medium | Smooth | Medium |
| `MICROSTEP` | 3200 | Medium | Very smooth | Medium |

Default is `INTERLEAVE` for good balance of smoothness and resolution.

## Testing

1. Upload the sketch to your Arduino
2. Open Serial Monitor at 115200 baud
3. You should see: `MKII Ready - Adafruit Motor Shield v2.3`
4. Try: `STATUS` to see current motor states
5. Test rotation: `ROTATE 1 90 CW` then `ROTATE 1 90 CCW`
6. Test rail: `MOVE 2 10 FORWARD` then `MOVE 2 10 BACKWARD`
7. Release motors: `RELEASE_ALL`

## Differences from MKI

| Feature | MKI (TB6600) | MKII (Motor Shield) |
|---------|--------------|---------------------|
| Drivers | External TB6600 | Integrated on shield |
| Power relay | External 24V relay | None (always on) |
| Axes | 3 (rotation, rail, tilt) | 2 (rotation, rail) |
| Microstepping | 32x (configurable) | Up to 16x (MICROSTEP) |
| Max current | 4A per driver | ~1.2A per motor |
| Motor type | High-torque NEMA23 | Smaller NEMA17 |

## Troubleshooting

**"Motor Shield not found" error:**
- Check shield is properly seated on Arduino
- Verify I2C connections (A4/A5 on Uno)
- Check for address conflicts if using multiple shields

**Motors not moving:**
- Check external power supply connected
- Verify motor wiring (swap coil pairs if vibrating but not spinning)
- Try `DOUBLE` step style for more torque

**Motors running backwards:**
- Set `DIR_REVERSE_M1 = true` or `DIR_REVERSE_M2 = true` in sketch
