# Sherline 6-Axis Scanner Configuration

## Overview

Adapting a Sherline CNC mill with 4th axis rotary table for photogrammetry scanning. This configuration provides 6 degrees of freedom for comprehensive specimen coverage.

## Axis Configuration

### Linear Axes (4)

| Axis | Name | Purpose | Hardware |
|------|------|---------|----------|
| X | Mill X | Horizontal positioning (left/right) | Sherline CNC leadscrew |
| Y | Mill Y | Horizontal positioning (front/back) | Sherline CNC leadscrew |
| Z | Mill Z | Vertical positioning (up/down) | Sherline CNC leadscrew |
| R | Camera Rail | Focus stacking (fine linear) | Linear rail on quill rotary |

### Rotational Axes (2)

| Axis | Name | Purpose | Hardware |
|------|------|---------|----------|
| A | Specimen Rotary | Rotate specimen (parallel to X) | 4th axis rotary table with chuck |
| B | Camera Tilt | Point camera angle (0°=horizontal, 90°=down) | Rotary table replacing quill |

## Physical Layout

```
Side View (looking from +Y toward -Y):

                    ┌─────────────┐
                    │  Camera     │ ← Mounted on Camera Rail (R)
                    │  Rail (R)   │
                    └──────┬──────┘
                           │
                    ┌──────┴──────┐
                    │ Camera Tilt │ ← Rotary B (replaces quill)
                    │   (B axis)  │    Rotates in XZ plane
                    └──────┬──────┘
                           │
              Z ↑    ┌─────┴─────┐
                │    │  Z Column │ ← Mill Z axis
                │    └─────┬─────┘
                │          │
    ────────────┼──────────┼────────────────────── X →
                │    ┌─────┴─────┐
                │    │  Specimen │ ← Held in chuck
                │    │   ◯───────│ ← A axis (rotary, parallel to X)
                │    └───────────┘
                │
           ─────┴───── Y (into page)
                │
           ┌────┴────┐
           │ Mill Bed│
           └─────────┘
```

## Coordinate System

- **Origin**: Center of specimen (chuck center)
- **X**: Left/Right (A-axis rotation axis)
- **Y**: Front/Back (toward operator)  
- **Z**: Up/Down (vertical)
- **A**: Specimen rotation (degrees, 0° = top up)
- **B**: Camera tilt (degrees, 0° = horizontal, 90° = looking down)
- **R**: Camera rail position (mm, focus distance)

## Scanning Strategy

### Horizontal Ring Scan (B = 0°)
1. Position camera at horizontal (B = 0°)
2. Set X, Y, Z to frame specimen
3. Rotate specimen through A axis (0° → 360°)
4. At each A position, capture focus stack using R axis

### Spherical Coverage Scan
1. For each elevation angle B (e.g., 0°, 30°, 60°, 90°):
   - Tilt camera to B angle
   - Adjust X, Y, Z to maintain specimen framing
   - Rotate specimen through A axis
   - Capture focus stacks at each A position

### Top-Down Scan (B = 90°)
1. Tilt camera straight down (B = 90°)
2. Position X, Y to center over specimen
3. Use Z + R for focus stacking
4. Rotate specimen through A for coverage

## Motion Controller Options

### Option 1: Existing Arduino + LinuxCNC Hybrid
- Arduino controls camera rail (R) and relays
- LinuxCNC/GRBL controls Sherline axes (X, Y, Z, A, B)
- Python coordinates both via serial

### Option 2: Pure LinuxCNC
- All axes controlled by LinuxCNC
- Python sends G-code commands
- Camera triggering via M-code or parallel port

### Option 3: GRBL-based
- GRBL on Arduino Mega or dedicated board
- 6-axis GRBL fork or multiple controllers
- Python sends G-code via serial

## Software Architecture Requirements

### Modular Axis System
```python
class Axis:
    name: str           # "X", "Y", "Z", "A", "B", "R"
    axis_type: str      # "linear" or "rotary"
    units: str          # "mm" or "degrees"
    min_limit: float
    max_limit: float
    steps_per_unit: float
    current_position: float
    
class MachineProfile:
    name: str
    axes: List[Axis]
    controller_type: str  # "arduino", "linuxcnc", "grbl"
    connection_params: dict
```

### Profile Examples
- `raspberry_pi_3axis.json` - Current 3-motor setup
- `sherline_6axis.json` - New Sherline configuration

## Hardware Checklist

- [ ] Sherline CNC mill with working X, Y, Z
- [ ] 4th axis rotary table (A axis) installed
- [ ] Quill replaced with rotary table (B axis)
- [ ] Camera rail mounted on B axis rotary
- [ ] Stepper drivers for all axes
- [ ] Motion controller (LinuxCNC PC, GRBL board, or Arduino)
- [ ] Camera mount on rail
- [ ] Wiring and limit switches
