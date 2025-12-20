# LinuxCNC Integration Plan

This document outlines the plan to integrate scanner-companion with a LinuxCNC-controlled Sherline 4-axis machine for precision photogrammetry scanning.

## Overview

Instead of using a single Raspberry Pi with Arduino for motor control, this configuration distributes the system across two machines:

- **Raspberry Pi**: Camera capture, GUI, image processing, orchestration
- **LinuxCNC Rig**: 4-axis precision motion control via network

## Architecture

```
┌─────────────────────────────────┐              ┌─────────────────────────────────┐
│     Raspberry Pi (Scanner)      │              │    LinuxCNC Rig (cnc-rig)       │
│         10.0.0.XXX              │              │        10.0.0.78                │
├─────────────────────────────────┤              ├─────────────────────────────────┤
│                                 │              │                                 │
│  ┌───────────────────────────┐  │    TCP/IP    │  ┌───────────────────────────┐  │
│  │   scanner_control.py      │  │◄────────────►│  │   linuxcnc_server.py      │  │
│  │   (Main Application)      │  │   Port 5000  │  │   (Motion Server)         │  │
│  └───────────────────────────┘  │              │  └───────────────────────────┘  │
│            │                    │              │            │                    │
│            ▼                    │              │            ▼                    │
│  ┌───────────────────────────┐  │              │  ┌───────────────────────────┐  │
│  │   Pi HQ Camera            │  │              │  │   LinuxCNC                │  │
│  │   (Ribbon Connection)     │  │              │  │   (Python API)            │  │
│  └───────────────────────────┘  │              │  └───────────────────────────┘  │
│            │                    │              │            │                    │
│            ▼                    │              │            ▼                    │
│  ┌───────────────────────────┐  │              │  ┌───────────────────────────┐  │
│  │   Arduino (USB)           │  │              │  │   Sherline 4-Axis         │  │
│  │   - LED lighting          │  │              │  │   - X: Linear positioning │  │
│  │   - Camera trigger        │  │              │  │   - Y: Linear positioning │  │
│  │   - Limit switches        │  │              │  │   - Z: Focus rail         │  │
│  └───────────────────────────┘  │              │  │   - A: Turntable rotation │  │
│                                 │              │  └───────────────────────────┘  │
└─────────────────────────────────┘              └─────────────────────────────────┘
```

## Hardware Requirements

### LinuxCNC Rig (cnc-rig - 10.0.0.78)

| Component | Status | Notes |
|-----------|--------|-------|
| Debian 12 (Bookworm) | ✅ Installed | |
| LinuxCNC 2.9.2 | ✅ Installed | uspace variant |
| Python 3.11 | ✅ Available | |
| LinuxCNC Python bindings | ✅ Available | `import linuxcnc` works |
| Sherline 4-axis config | ✅ Configured | X, Y, Z linear + A rotary |
| Network connection | ✅ Connected | 10.0.0.78 |

### Raspberry Pi (Scanner Controller)

| Component | Status | Notes |
|-----------|--------|-------|
| Raspberry Pi 4 | ⬜ TBD | Needs assignment |
| Pi HQ Camera | ⬜ TBD | Ribbon connection |
| picamera2 | ⬜ TBD | Camera library |
| Arduino | ⬜ TBD | USB connection for I/O |
| scanner-companion repo | ⬜ TBD | Clone and configure |

## Sherline Axis Mapping

The Sherline 4-axis configuration maps naturally to scanner operations:

| Sherline Axis | Scanner Function | Range | Units |
|---------------|------------------|-------|-------|
| **A** (Joint 3) | Turntable rotation | ±36000° | degrees |
| **Z** (Joint 2) | Focus rail (lens distance) | ±300mm | mm |
| **X** (Joint 0) | Horizontal specimen offset | ±400mm | mm |
| **Y** (Joint 1) | Depth specimen offset | ±300mm | mm |

**Note**: The A axis supports continuous rotation (±36000° = 100 full rotations), perfect for 360° scanning.

## Implementation Plan

### Phase 1: LinuxCNC Motion Server (On cnc-rig)

Create `linuxcnc_server.py` - a TCP server that:
1. Listens on port 5000 for incoming commands
2. Translates commands to LinuxCNC MDI G-code
3. Returns status/completion to client

**Command Protocol** (compatible with existing scanner_control.py):
```
ROTATE A <degrees> <CW|CCW>     → G91 A±<degrees>
MOVE Z <mm> <FORWARD|BACKWARD>  → G91 Z±<mm>
MOVE X <mm> <FORWARD|BACKWARD>  → G91 X±<mm>
MOVE Y <mm> <FORWARD|BACKWARD>  → G91 Y±<mm>
ZERO <axis>                     → G92 <axis>0
GET_POS <axis>                  → Query and return position
HOME <axis>                     → G28 or homing sequence
STATUS                          → Return LinuxCNC state
```

**Response Protocol**:
```
OK                              → Command completed successfully
ERR: <message>                  → Error with description
<number>                        → Position value for GET_POS
```

### Phase 2: Network Motor Driver (On Raspberry Pi)

Create `NetworkMotorDriver` class in scanner_control.py:
```python
class NetworkMotorDriver:
    """Motor driver that communicates with LinuxCNC over TCP"""
    
    def __init__(self, host='10.0.0.78', port=5000):
        self.host = host
        self.port = port
        self.socket = None
    
    def connect(self):
        """Establish connection to LinuxCNC server"""
        pass
    
    def send_command(self, command, timeout=120):
        """Send command and wait for response"""
        pass
    
    def rotate(self, degrees, direction='CW'):
        """Rotate turntable (A axis)"""
        pass
    
    def move_focus(self, mm, direction='FORWARD'):
        """Move focus rail (Z axis)"""
        pass
```

### Phase 3: Configuration System

Add configuration option to scanner_control.py to select motor backend:

```python
MOTOR_BACKEND = "linuxcnc"  # or "arduino"

LINUXCNC_CONFIG = {
    "host": "10.0.0.78",
    "port": 5000,
    "turntable_axis": "A",
    "focus_axis": "Z",
}

ARDUINO_CONFIG = {
    "port": "/dev/ttyACM0",
    "baudrate": 115200,
}
```

### Phase 4: Arduino Role (Optional I/O)

The Arduino connected to the Raspberry Pi handles auxiliary functions:
- Camera shutter trigger (for cameras requiring hardware trigger)
- LED ring/panel lighting control
- Limit switch monitoring (if not handled by LinuxCNC)
- Focus stacking signal synchronization

This is the existing Arduino sketch functionality, just with motor control removed.

## Network Protocol Details

### TCP Socket Communication

**Port**: 5000 (configurable)

**Connection Flow**:
1. Client connects to server
2. Server sends: `READY\n`
3. Client sends commands, one per line
4. Server executes and responds
5. Connection remains open for session

**Timeout Handling**:
- Command timeout: 120 seconds (for long moves)
- Connection timeout: 5 seconds
- Keepalive: Optional ping every 30 seconds

### Example Session

```
Client → Server: ROTATE A 45 CW
Server → Client: OK

Client → Server: GET_POS A
Server → Client: 45.000

Client → Server: MOVE Z 10.5 FORWARD
Server → Client: OK

Client → Server: STATUS
Server → Client: STATE:ON MODE:MDI HOMED:XYZA
```

## Setup Procedure

### Step 1: LinuxCNC Server Setup (cnc-rig)

```bash
# On cnc-rig (10.0.0.78)
cd ~/GIT/scanner-companion
git checkout linux-cnc-configuration
git pull

# Install any additional dependencies
pip3 install --user <dependencies>

# Test LinuxCNC connection
python3 -c "import linuxcnc; print('LinuxCNC OK')"

# Start LinuxCNC with Sherline config
linuxcnc ~/linuxcnc/configs/by_machine.sherline.Sherline4Axis/Sherline4Axis.ini &

# Start motion server
python3 linuxcnc_server.py
```

### Step 2: Raspberry Pi Setup

```bash
# On Raspberry Pi
cd ~/GIT/scanner-companion
git checkout linux-cnc-configuration
git pull

# Configure for LinuxCNC backend
# Edit scanner_control.py or config file

# Test network connection
ping 10.0.0.78
nc -zv 10.0.0.78 5000

# Run scanner control
python3 scanner_control.py
```

### Step 3: Integration Testing

1. **Motion Test**: Send rotation command from Pi, verify Sherline moves
2. **Position Test**: Query position, verify accuracy
3. **Camera Test**: Capture image during motion pause
4. **Full Sequence**: Run calibration wizard with network motors

## File Structure

```
scanner-companion/
├── scanner_control.py           # Main GUI (modified for backend selection)
├── linuxcnc_server.py          # NEW: TCP server for LinuxCNC (runs on cnc-rig)
├── network_motor_driver.py     # NEW: TCP client motor driver
├── arduino/
│   └── scanner_controller.ino  # Existing (for lighting/trigger only)
├── docs/
│   └── LINUXCNC_INTEGRATION.md # This document
└── config/
    └── linuxcnc_config.json    # NEW: Network motor configuration
```

## Advantages of This Architecture

1. **Precision Motion**: LinuxCNC provides industrial-grade motion control with proper acceleration profiles
2. **Existing Calibration**: Sherline's steps-per-unit already configured
3. **4-Axis Flexibility**: Can use X/Y for specimen centering, Z for focus, A for rotation
4. **Camera Quality**: Pi HQ Camera with native ribbon connection
5. **Distributed Load**: Motion control on dedicated real-time system
6. **Maintainability**: Clear separation of concerns

## Potential Challenges

1. **Network Latency**: TCP adds ~1-5ms latency per command (acceptable for photogrammetry)
2. **LinuxCNC Must Run**: Server requires active LinuxCNC instance
3. **Homing Required**: LinuxCNC needs homing before motion commands
4. **Error Handling**: Network failures need graceful recovery
5. **RAM on cnc-rig**: Only 1.6GB - keep server lightweight

## Future Enhancements

- [ ] Auto-discovery of LinuxCNC server on network
- [ ] Web-based status dashboard on cnc-rig
- [ ] Multiple camera support (USB cameras on cnc-rig)
- [ ] G-code file generation for repeatable scan patterns
- [ ] Integration with LinuxCNC's HAL for hardware I/O

## References

- [LinuxCNC Python Interface](http://linuxcnc.org/docs/html/config/python-interface.html)
- [Scanner Companion Documentation](./FEATURES.md)
- [Network Topology](../../home-network-setup/docs/network-topology.md)
