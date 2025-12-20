"""
Motor/Axis Configuration Module for Scanner Companion

This module provides a flexible, profile-based system for configuring
different scanner hardware setups. Supports arbitrary numbers of linear
and rotary axes with different motion controllers.

Profiles are stored as JSON files in the 'profiles/' directory.
"""

import json
import os
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Optional, Callable, Any
from enum import Enum
from pathlib import Path


class AxisType(Enum):
    """Type of motion axis"""
    LINEAR = "linear"
    ROTARY = "rotary"


class ControllerType(Enum):
    """Type of motion controller"""
    ARDUINO_SERIAL = "arduino_serial"  # Direct Arduino serial commands
    GRBL = "grbl"                      # GRBL G-code controller
    LINUXCNC = "linuxcnc"              # LinuxCNC via linuxcnc-rpc or serial
    MOCK = "mock"                      # Mock controller for testing


@dataclass
class AxisConfig:
    """Configuration for a single motion axis"""
    
    # Identity
    name: str                          # Short name: "X", "Y", "Z", "A", "B", "R"
    label: str                         # Display label: "Mill X", "Specimen Rotary"
    axis_type: AxisType                # LINEAR or ROTARY
    
    # Units and limits
    units: str = "mm"                  # "mm" for linear, "deg" for rotary
    min_limit: float = 0.0             # Minimum position
    max_limit: float = 100.0           # Maximum position
    home_position: float = 0.0         # Home/zero position
    
    # Motion parameters
    steps_per_unit: float = 100.0      # Steps per mm or steps per degree
    max_velocity: float = 10.0         # Max speed in units/sec
    acceleration: float = 50.0         # Acceleration in units/sec²
    
    # Controller mapping
    controller_axis: str = ""          # Axis letter for controller (e.g., "X" for GRBL)
    motor_index: int = 0               # Motor number for Arduino serial
    
    # Direction
    inverted: bool = False             # Invert direction
    
    # Homing
    has_home_switch: bool = False      # Has limit/home switch
    home_direction: int = -1           # Direction to home: -1 or +1
    
    # Role in scanning (semantic meaning)
    role: str = ""                     # "focus", "specimen_rotation", "camera_tilt", etc.
    
    def __post_init__(self):
        # Convert string to enum if needed
        if isinstance(self.axis_type, str):
            self.axis_type = AxisType(self.axis_type)
        # Default units based on type
        if self.units == "mm" and self.axis_type == AxisType.ROTARY:
            self.units = "deg"
    
    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization"""
        d = asdict(self)
        d['axis_type'] = self.axis_type.value
        return d
    
    @classmethod
    def from_dict(cls, data: dict) -> 'AxisConfig':
        """Create from dictionary"""
        data = data.copy()
        if 'axis_type' in data:
            data['axis_type'] = AxisType(data['axis_type'])
        return cls(**data)


@dataclass  
class ControllerConfig:
    """Configuration for a motion controller"""
    
    controller_type: ControllerType
    name: str = "default"              # Controller name for multi-controller setups
    
    # Connection parameters
    port: str = "/dev/ttyACM0"         # Serial port
    baudrate: int = 115200             # Baud rate
    
    # Controller-specific settings
    settings: Dict[str, Any] = field(default_factory=dict)
    
    def __post_init__(self):
        if isinstance(self.controller_type, str):
            self.controller_type = ControllerType(self.controller_type)
    
    def to_dict(self) -> dict:
        d = asdict(self)
        d['controller_type'] = self.controller_type.value
        return d
    
    @classmethod
    def from_dict(cls, data: dict) -> 'ControllerConfig':
        data = data.copy()
        if 'controller_type' in data:
            data['controller_type'] = ControllerType(data['controller_type'])
        return cls(**data)


@dataclass
class MachineProfile:
    """Complete machine configuration profile"""
    
    # Identity
    name: str                          # Profile name
    description: str = ""              # Human-readable description
    version: str = "1.0"               # Profile version
    
    # Axes
    axes: List[AxisConfig] = field(default_factory=list)
    
    # Controllers (supports multiple controllers)
    controllers: List[ControllerConfig] = field(default_factory=list)
    
    # Axis-to-controller mapping
    # Maps axis name to controller name
    axis_controller_map: Dict[str, str] = field(default_factory=dict)
    
    # Power relay configuration
    power_relay: Dict[str, Any] = field(default_factory=dict)
    
    # Scanning defaults
    scan_defaults: Dict[str, Any] = field(default_factory=dict)
    
    def get_axis(self, name: str) -> Optional[AxisConfig]:
        """Get axis by name"""
        for axis in self.axes:
            if axis.name == name:
                return axis
        return None
    
    def get_axes_by_type(self, axis_type: AxisType) -> List[AxisConfig]:
        """Get all axes of a specific type"""
        return [a for a in self.axes if a.axis_type == axis_type]
    
    def get_axes_by_role(self, role: str) -> List[AxisConfig]:
        """Get all axes with a specific role"""
        return [a for a in self.axes if a.role == role]
    
    def get_focus_axis(self) -> Optional[AxisConfig]:
        """Get the axis used for focus stacking"""
        axes = self.get_axes_by_role("focus")
        return axes[0] if axes else None
    
    def get_specimen_rotation_axis(self) -> Optional[AxisConfig]:
        """Get the axis that rotates the specimen"""
        axes = self.get_axes_by_role("specimen_rotation")
        return axes[0] if axes else None
    
    def get_camera_tilt_axis(self) -> Optional[AxisConfig]:
        """Get the axis that tilts the camera"""
        axes = self.get_axes_by_role("camera_tilt")
        return axes[0] if axes else None
    
    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization"""
        return {
            'name': self.name,
            'description': self.description,
            'version': self.version,
            'axes': [a.to_dict() for a in self.axes],
            'controllers': [c.to_dict() for c in self.controllers],
            'axis_controller_map': self.axis_controller_map,
            'power_relay': self.power_relay,
            'scan_defaults': self.scan_defaults,
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'MachineProfile':
        """Create from dictionary"""
        return cls(
            name=data.get('name', 'unknown'),
            description=data.get('description', ''),
            version=data.get('version', '1.0'),
            axes=[AxisConfig.from_dict(a) for a in data.get('axes', [])],
            controllers=[ControllerConfig.from_dict(c) for c in data.get('controllers', [])],
            axis_controller_map=data.get('axis_controller_map', {}),
            power_relay=data.get('power_relay', {}),
            scan_defaults=data.get('scan_defaults', {}),
        )
    
    def save(self, filepath: str) -> None:
        """Save profile to JSON file"""
        with open(filepath, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)
    
    @classmethod
    def load(cls, filepath: str) -> 'MachineProfile':
        """Load profile from JSON file"""
        with open(filepath, 'r') as f:
            data = json.load(f)
        return cls.from_dict(data)


# ──────────────────────────────────────────────────────────────────────────────
# PRESET PROFILES
# ──────────────────────────────────────────────────────────────────────────────

def create_raspberry_pi_3axis_profile() -> MachineProfile:
    """Create profile for the original Raspberry Pi 3-axis scanner"""
    return MachineProfile(
        name="raspberry_pi_3axis",
        description="Original 3-axis scanner: rotation platform, linear focus rail, vertical tilt",
        version="1.0",
        axes=[
            AxisConfig(
                name="M1",
                label="Rotation Platform",
                axis_type=AxisType.ROTARY,
                units="deg",
                min_limit=0.0,
                max_limit=360.0,
                steps_per_unit=17.7778,  # 32x microstepping
                motor_index=1,
                role="specimen_rotation",
            ),
            AxisConfig(
                name="M2",
                label="Focus Rail",
                axis_type=AxisType.LINEAR,
                units="mm",
                min_limit=0.0,
                max_limit=150.0,
                steps_per_unit=1281.21,  # Calibrated
                motor_index=2,
                has_home_switch=True,
                home_direction=-1,
                role="focus",
            ),
            AxisConfig(
                name="M3",
                label="Vertical Tilt",
                axis_type=AxisType.ROTARY,
                units="deg",
                min_limit=-90.0,
                max_limit=90.0,
                steps_per_unit=88.8889,  # 5:1 gearbox
                motor_index=3,
                role="camera_tilt",
            ),
        ],
        controllers=[
            ControllerConfig(
                controller_type=ControllerType.ARDUINO_SERIAL,
                name="arduino",
                port="/dev/ttyACM0",
                baudrate=115200,
            ),
        ],
        axis_controller_map={
            "M1": "arduino",
            "M2": "arduino",
            "M3": "arduino",
        },
        power_relay={
            "enabled": True,
            "pin": "A0",
            "active_low": True,
        },
        scan_defaults={
            "perspectives": 72,
            "focus_slices": 5,
            "settle_delay": 1.0,
        },
    )


def create_sherline_6axis_profile() -> MachineProfile:
    """Create profile for Sherline CNC 6-axis scanner"""
    return MachineProfile(
        name="sherline_6axis",
        description="Sherline CNC mill with 4th axis rotary and quill-mounted camera tilt",
        version="1.0",
        axes=[
            # Mill linear axes
            AxisConfig(
                name="X",
                label="Mill X",
                axis_type=AxisType.LINEAR,
                units="mm",
                min_limit=0.0,
                max_limit=230.0,  # Sherline travel
                steps_per_unit=2000.0,  # Typical Sherline
                controller_axis="X",
                role="positioning",
            ),
            AxisConfig(
                name="Y",
                label="Mill Y",
                axis_type=AxisType.LINEAR,
                units="mm",
                min_limit=0.0,
                max_limit=175.0,
                steps_per_unit=2000.0,
                controller_axis="Y",
                role="positioning",
            ),
            AxisConfig(
                name="Z",
                label="Mill Z",
                axis_type=AxisType.LINEAR,
                units="mm",
                min_limit=0.0,
                max_limit=165.0,
                steps_per_unit=2000.0,
                controller_axis="Z",
                role="positioning",
            ),
            # Camera rail (focus axis)
            AxisConfig(
                name="R",
                label="Camera Rail",
                axis_type=AxisType.LINEAR,
                units="mm",
                min_limit=0.0,
                max_limit=100.0,
                steps_per_unit=1281.21,  # Same as original
                controller_axis="U",  # Auxiliary axis in LinuxCNC
                motor_index=2,  # Or Arduino motor 2
                has_home_switch=True,
                role="focus",
            ),
            # Rotary axes
            AxisConfig(
                name="A",
                label="Specimen Rotary",
                axis_type=AxisType.ROTARY,
                units="deg",
                min_limit=0.0,
                max_limit=360.0,
                steps_per_unit=44.444,  # 4th axis rotary table
                controller_axis="A",
                role="specimen_rotation",
            ),
            AxisConfig(
                name="B",
                label="Camera Tilt",
                axis_type=AxisType.ROTARY,
                units="deg",
                min_limit=0.0,
                max_limit=90.0,  # 0=horizontal, 90=looking down
                steps_per_unit=44.444,
                controller_axis="B",
                role="camera_tilt",
            ),
        ],
        controllers=[
            # Primary controller for CNC axes
            ControllerConfig(
                controller_type=ControllerType.GRBL,  # or LINUXCNC
                name="cnc",
                port="/dev/ttyUSB0",
                baudrate=115200,
                settings={
                    "coordinate_system": "G54",
                },
            ),
            # Optional secondary Arduino for camera rail
            ControllerConfig(
                controller_type=ControllerType.ARDUINO_SERIAL,
                name="arduino",
                port="/dev/ttyACM0",
                baudrate=115200,
            ),
        ],
        axis_controller_map={
            "X": "cnc",
            "Y": "cnc",
            "Z": "cnc",
            "A": "cnc",
            "B": "cnc",
            "R": "arduino",  # Camera rail on Arduino
        },
        power_relay={
            "enabled": True,
            "controller": "arduino",
            "pin": "A0",
            "active_low": True,
        },
        scan_defaults={
            "perspectives": 72,
            "focus_slices": 5,
            "settle_delay": 0.5,
            "elevation_angles": [0, 30, 60, 90],
        },
    )


# ──────────────────────────────────────────────────────────────────────────────
# PROFILE MANAGER
# ──────────────────────────────────────────────────────────────────────────────

class ProfileManager:
    """Manages machine profiles"""
    
    def __init__(self, profiles_dir: str = None):
        if profiles_dir is None:
            # Default to 'profiles' directory next to this file
            profiles_dir = os.path.join(os.path.dirname(__file__), 'profiles')
        self.profiles_dir = Path(profiles_dir)
        self.profiles_dir.mkdir(parents=True, exist_ok=True)
        
        # Built-in profiles
        self._builtin_profiles = {
            'raspberry_pi_3axis': create_raspberry_pi_3axis_profile,
            'sherline_6axis': create_sherline_6axis_profile,
        }
    
    def list_profiles(self) -> List[str]:
        """List all available profile names"""
        profiles = list(self._builtin_profiles.keys())
        
        # Add profiles from files
        for f in self.profiles_dir.glob('*.json'):
            name = f.stem
            if name not in profiles:
                profiles.append(name)
        
        return sorted(profiles)
    
    def load_profile(self, name: str) -> MachineProfile:
        """Load a profile by name"""
        # Check for file first
        filepath = self.profiles_dir / f"{name}.json"
        if filepath.exists():
            return MachineProfile.load(str(filepath))
        
        # Check built-in profiles
        if name in self._builtin_profiles:
            return self._builtin_profiles[name]()
        
        raise ValueError(f"Profile not found: {name}")
    
    def save_profile(self, profile: MachineProfile, name: str = None) -> str:
        """Save a profile to file"""
        if name is None:
            name = profile.name
        filepath = self.profiles_dir / f"{name}.json"
        profile.save(str(filepath))
        return str(filepath)
    
    def delete_profile(self, name: str) -> bool:
        """Delete a profile file (cannot delete built-in profiles)"""
        filepath = self.profiles_dir / f"{name}.json"
        if filepath.exists():
            filepath.unlink()
            return True
        return False


# ──────────────────────────────────────────────────────────────────────────────
# TESTING
# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    # Test profile creation and serialization
    print("Testing axis configuration module...\n")
    
    # Create and test Raspberry Pi profile
    rpi_profile = create_raspberry_pi_3axis_profile()
    print(f"Profile: {rpi_profile.name}")
    print(f"  Description: {rpi_profile.description}")
    print(f"  Axes: {[a.name for a in rpi_profile.axes]}")
    print(f"  Focus axis: {rpi_profile.get_focus_axis().label}")
    print(f"  Rotation axis: {rpi_profile.get_specimen_rotation_axis().label}")
    print()
    
    # Create and test Sherline profile
    sherline_profile = create_sherline_6axis_profile()
    print(f"Profile: {sherline_profile.name}")
    print(f"  Description: {sherline_profile.description}")
    print(f"  Axes: {[a.name for a in sherline_profile.axes]}")
    print(f"  Linear axes: {[a.name for a in sherline_profile.get_axes_by_type(AxisType.LINEAR)]}")
    print(f"  Rotary axes: {[a.name for a in sherline_profile.get_axes_by_type(AxisType.ROTARY)]}")
    print(f"  Focus axis: {sherline_profile.get_focus_axis().label}")
    print(f"  Rotation axis: {sherline_profile.get_specimen_rotation_axis().label}")
    print(f"  Camera tilt axis: {sherline_profile.get_camera_tilt_axis().label}")
    print()
    
    # Test JSON serialization
    print("Testing JSON serialization...")
    json_str = json.dumps(sherline_profile.to_dict(), indent=2)
    print(f"  JSON length: {len(json_str)} bytes")
    
    # Test round-trip
    loaded = MachineProfile.from_dict(json.loads(json_str))
    print(f"  Round-trip OK: {loaded.name == sherline_profile.name}")
    print()
    
    # Test profile manager
    print("Testing ProfileManager...")
    pm = ProfileManager()
    print(f"  Profiles dir: {pm.profiles_dir}")
    print(f"  Available profiles: {pm.list_profiles()}")
    
    print("\n✓ All tests passed!")
