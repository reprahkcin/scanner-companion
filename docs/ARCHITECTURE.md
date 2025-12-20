# Scanner Companion Architecture

This document defines the vocabulary, coordinate systems, and machine abstraction model for Scanner Companion. It serves as the foundation for supporting multiple hardware configurations with a unified software interface.

## Design Philosophy

1. **Specimen at Origin**: The object being scanned is always conceptually at (0, 0, 0)
2. **Camera Looks at Origin**: Regardless of how the camera moves, it always points at the specimen
3. **Axes Describe Motion**: Each motor axis is described by its role, geometry, and how it affects the camera-specimen relationship
4. **Profiles Are Complete**: A machine profile contains everything needed to operate a scanner

---

## Coordinate System

All machines use a **right-handed, Z-up coordinate system**:

```
        +Z (up)
         │
         │
         │
         └──────── +X (right)
        /
       /
      +Y (forward/toward viewer)
```

- **Rotations**: Positive rotation is counter-clockwise when looking down the positive axis
- **Origin**: Specimen center is at (0, 0, 0)
- **Units**: Millimeters for linear, degrees for rotary

---

## Vocabulary

### Core Concepts

| Term | Definition |
|------|------------|
| **Specimen** | The object being scanned. Conceptually fixed at origin (0,0,0), though it may physically move during scanning. |
| **Perspective** | A single camera viewpoint. One perspective = one focus stack. |
| **Focus Stack** | Multiple images at different focus distances from the same perspective. |
| **Session** | A complete capture run: all perspectives × all focus slices. |

### Axis Roles

| Role | Description | Motion Type | Example |
|------|-------------|-------------|---------|
| `focus_rail` | Moves camera toward/away from specimen for focus stacking | Linear | Camera on rail |
| `turntable` | Rotates specimen around a vertical axis | Rotary | Spinning platform |
| `tilt` | Changes camera elevation angle relative to specimen | Rotary | Tilting arm or specimen |
| `specimen_rotation` | Alternative turntable with different axis orientation | Rotary | 4th axis rotary table |
| `camera_orbit` | Swings camera around specimen | Rotary | Camera arm rotation |
| `bed_positioning` | Moves specimen to maintain centering (not for scanning) | Linear | Mill X/Y table |

### Motion Relationships

| Scanner Type | What Moves | What's Fixed | Effect |
|--------------|------------|--------------|--------|
| **Specimen-rotating** | Specimen spins, camera slides | Camera arc center | MKI style |
| **Camera-orbiting** | Camera orbits, specimen may rotate | Specimen position | MKII style |
| **Hybrid** | Both move in coordination | Relative geometry | Complex multi-axis |

---

## Machine Profiles

### MKI: Raspberry Pi 3-Axis Scanner

**Configuration**: Specimen rotates and tilts, camera on fixed linear rail

```
Side View (Y=0 plane):                    Top View (Z=-10 plane):

        Camera ←───── Focus Rail              Turntable
           ●━━━━━━━━━━━━━━━●                    ┌───┐
           │                                    │ ● │ Specimen
           │                                    │   │
           ▼                                    └───┘
      ┌─────────┐                                 │
      │Specimen │ ← at origin (0,0,0)             │
      └─────────┘                              Tilt axis
           │                                   (Y rotation)
      ═════●═════ ← Turntable (Z rotation)
           │
       Tilt Axis
      (Y rotation)
```

**Axis Geometry**:

- **M1 (Turntable)**: Z-axis rotation, pivot at (0, 0, -10)
- **M2 (Focus Rail)**: X-axis linear motion, camera at (10→20, 0, 0)
- **M3 (Tilt)**: Y-axis rotation, pivot at (0, -10, 0), swings turntable

**Camera Pose Calculation**:

1. Start with camera at (distance, 0, 0) looking toward origin
2. Apply tilt rotation (rotate around Y through specimen)
3. Apply turntable angle (rotate camera position around Z)
4. Compute look-at matrix toward (0, 0, 0)

---

### MKII: Sherline Mill 6-Axis Scanner

**Configuration**: Camera looks down, orbits via rotary mount, specimen in 4th axis

```
Front View (Y=0 plane):                   Top View (Z=50 plane):

     Camera Rotary (B)                         ┌─────┐
         ╔═══╗                                 │  ●  │ Camera
         ║ ● ║ ← Camera                        │     │
         ╚═╬═╝                                 └──┬──┘
           ║ Focus Rail (Z)                       │
           ║                                   B rotation
           ▼                                  (XZ plane)
      ┌─────────┐
      │Specimen │ ← at origin, in 4th axis
      └────●────┘
           │
    ═══════●═══════ ← 4th Axis Rotary (A)
           │            (X rotation)
     ══════╪══════ ← Mill Bed (X/Y positioning)
```

**Axis Geometry**:

- **A (Turntable)**: X-axis rotation, specimen mounted in rotary table
- **R (Focus Rail)**: Z-axis linear motion, camera at (0, 0, 30→80)
- **B (Camera Orbit)**: Rotation in XZ plane, swings camera around specimen
- **X, Y (Bed)**: Positioning to keep specimen centered under camera

**Camera Pose Calculation**:

1. Start with camera at (0, 0, distance) looking down at origin
2. Apply camera orbit angle (B axis)
3. Compensate with X/Y bed position to keep specimen centered
4. Apply specimen rotation (A axis) for different angles
5. Compute look-at matrix toward (0, 0, 0)

---

## Profile Schema

### Complete Profile Structure

```json
{
  "name": "profile_name",
  "description": "Human-readable description",
  "version": "1.0",
  
  "axes": [
    {
      "name": "M1",
      "label": "Display Name",
      "axis_type": "rotary|linear",
      "units": "deg|mm",
      "min_limit": 0.0,
      "max_limit": 360.0,
      "home_position": 0.0,
      "steps_per_unit": 17.7778,
      "max_velocity": 10.0,
      "acceleration": 50.0,
      "controller_axis": "",
      "motor_index": 1,
      "inverted": false,
      "has_home_switch": false,
      "home_direction": -1,
      "role": "turntable"
    }
  ],
  
  "controllers": [
    {
      "controller_type": "arduino_serial|grbl|linuxcnc|mock",
      "name": "controller_name",
      "port": "/dev/ttyACM0",
      "baudrate": 115200,
      "settings": {}
    }
  ],
  
  "axis_controller_map": {
    "M1": "controller_name"
  },
  
  "scene_geometry": {
    "coordinate_system": "Z-up, right-handed",
    
    "specimen": {
      "position": [0, 0, 0],
      "mounted_on": null,
      "notes": ""
    },
    
    "axes_geometry": {
      "AXIS_NAME": {
        "role": "turntable|focus_rail|tilt|camera_orbit|bed_positioning",
        "motion_type": "rotation|linear",
        "axis_vector": [0, 0, 1],
        "pivot_point": [0, 0, 0],
        "home_position": [0, 0, 0],
        "positive_direction": "description of positive movement"
      }
    },
    
    "camera": {
      "mounted_on": "AXIS_NAME",
      "looks_at": "specimen",
      "home_orientation": [1, 0, 0],
      "notes": ""
    }
  },
  
  "power_relay": {
    "enabled": true,
    "controller": "controller_name",
    "pin": "A0",
    "active_low": true
  },
  
  "scan_defaults": {
    "perspectives": 72,
    "focus_slices": 5,
    "settle_delay": 1.0,
    "elevation_angles": [0, 30, 60]
  }
}
```

### Scene Geometry Fields

| Field | Type | Description |
|-------|------|-------------|
| `coordinate_system` | string | Reference frame description |
| `specimen.position` | [x,y,z] | Specimen center in world coordinates |
| `specimen.mounted_on` | string? | Axis name if specimen moves with an axis |
| `axes_geometry.*.role` | string | Semantic role (see Axis Roles above) |
| `axes_geometry.*.motion_type` | string | "rotation" or "linear" |
| `axes_geometry.*.axis_vector` | [x,y,z] | Unit vector for rotation/motion axis |
| `axes_geometry.*.pivot_point` | [x,y,z] | Center of rotation (rotary) or start position (linear) |
| `axes_geometry.*.positive_direction` | string | Human description of positive movement |
| `camera.mounted_on` | string | Which axis the camera moves with |
| `camera.home_orientation` | [x,y,z] | Direction camera faces at home position |

---

## Image Management

### Session Directory Structure

```
output_dir/
  specimen_name/
    session_YYYYMMDD_HHMMSS/
      metadata.json           # Session parameters, machine profile used
      README.md               # Workflow instructions
      stack_00/               # Perspective 0
        stack_00_shot_000.jpg
        stack_00_shot_001.jpg
        ...
      stack_01/               # Perspective 1
        ...
      xmp_files/              # Consolidated XMP sidecar files
        stack_00.xmp
        stack_01.xmp
        ...
```

### File Server (Planned)

For campus/isolated network use, the Pi will host a simple HTTP file server:

```
┌─────────────────────────────────────────────────────────────┐
│                    Scanner Network                          │
│                                                             │
│  ┌─────────────────┐         ┌─────────────────────────┐   │
│  │  Scanner Pi     │  WiFi   │  Workstation            │   │
│  │  10.0.0.50:8080 │◄───────►│  Browser                │   │
│  │                 │         │                         │   │
│  │  Endpoints:     │         │  - View sessions        │   │
│  │  GET /          │         │  - Preview images       │   │
│  │  GET /sessions  │         │  - Download ZIPs        │   │
│  │  GET /session/X │         │                         │   │
│  │  GET /download  │         │                         │   │
│  └─────────────────┘         └─────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

**Features**:

- Read-only access (no uploads from clients)
- Session browser with thumbnails
- On-demand ZIP generation for bulk download
- Optional: rclone integration for cloud backup when network available

---

## Multi-Machine Future

### Phase 1: Single Pi, Multiple Profiles

- One Raspberry Pi serves as controller
- User selects machine profile at startup
- Profiles stored in `profiles/` directory

### Phase 2: Dedicated Pis

- Each scanner gets its own Pi
- Profile baked into each Pi's configuration
- Common codebase, different active profile

### Phase 3: Networked Fleet

- Multiple Pis on same network
- Central dashboard for monitoring
- Unified session storage

---

## Camera Pose Calculation

### Generic Algorithm

For any machine configuration, camera pose is calculated as:

1. **Start at home**: Camera at `camera.home_position`, facing `camera.home_orientation`
2. **Apply focus offset**: Move along focus rail axis by current focus distance
3. **Apply camera rotation**: If camera orbit axis exists, rotate camera around specimen
4. **Apply specimen rotation**: Transform by turntable angle (inverse, since specimen rotating = camera orbiting opposite)
5. **Apply tilt**: Rotate to current elevation angle
6. **Compute look-at**: Generate rotation matrix so camera Z-axis points at origin

### Pose Output (XMP Format)

```
Position: [x, y, z] in meters (scaled appropriately)
Rotation: 3x3 matrix, row-major, transforms world → camera coordinates
```

---

## Implementation Checklist

### Completed

- [x] `AxisConfig` dataclass
- [x] `MachineProfile` dataclass  
- [x] `MotionController` abstract base class
- [x] `ArduinoSerialController` implementation
- [x] Profile JSON format
- [x] MKI basic operation

### In Progress

- [ ] Scene geometry schema in profiles
- [ ] Generic camera pose calculation from scene geometry
- [ ] Profile selection UI

### Planned

- [ ] GRBL controller implementation
- [ ] LinuxCNC controller implementation
- [ ] Integration of motion abstraction with `scanner_control.py`
- [ ] HTTP file server for image access
- [ ] Multi-machine coordination

---

## References

- [XMP Camera Math](https://dev.epicgames.com/community/learning/knowledge-base/vzwB/realityscan-realitycapture-xmp-camera-math)
- [RealityCapture XMP Alignment](https://rshelp.capturingreality.com/en-US/tools/xmpalign.htm)
- [LinuxCNC Python Interface](http://linuxcnc.org/docs/html/config/python-interface.html)
