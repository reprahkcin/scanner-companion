"""
Motion Controller Abstraction Layer

Provides a unified interface for different motion controllers:
- Arduino Serial (custom protocol)
- GRBL (G-code)
- LinuxCNC (G-code via serial or network)
- Mock (for testing without hardware)

Each controller implements the MotionController interface.
"""

import time
import threading
import queue
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, List, Optional, Callable, Any
from enum import Enum

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

from axis_config import AxisConfig, ControllerConfig, ControllerType, AxisType


class MotionState(Enum):
    """Current state of motion controller"""
    DISCONNECTED = "disconnected"
    IDLE = "idle"
    MOVING = "moving"
    HOMING = "homing"
    ALARM = "alarm"
    ERROR = "error"


@dataclass
class AxisPosition:
    """Current position of an axis"""
    axis_name: str
    position: float
    units: str
    is_homed: bool = False


@dataclass
class MoveCommand:
    """A motion command"""
    axis_name: str
    target: float          # Target position in axis units
    is_relative: bool = False
    feedrate: Optional[float] = None  # Optional feedrate override


@dataclass
class MoveResult:
    """Result of a motion command"""
    success: bool
    axis_name: str
    final_position: float
    error_message: str = ""


# ──────────────────────────────────────────────────────────────────────────────
# ABSTRACT BASE CLASS
# ──────────────────────────────────────────────────────────────────────────────

class MotionController(ABC):
    """Abstract base class for motion controllers"""
    
    def __init__(self, config: ControllerConfig, axes: List[AxisConfig]):
        self.config = config
        self.axes = {a.name: a for a in axes}
        self._positions: Dict[str, float] = {a.name: 0.0 for a in axes}
        self._state = MotionState.DISCONNECTED
        self._connected = False
        self._lock = threading.Lock()
        self._callbacks: List[Callable[[str, Any], None]] = []
    
    @property
    def state(self) -> MotionState:
        return self._state
    
    @property
    def is_connected(self) -> bool:
        return self._connected
    
    def add_callback(self, callback: Callable[[str, Any], None]) -> None:
        """Add callback for state changes. callback(event_type, data)"""
        self._callbacks.append(callback)
    
    def _notify(self, event_type: str, data: Any = None) -> None:
        """Notify all callbacks"""
        for cb in self._callbacks:
            try:
                cb(event_type, data)
            except Exception as e:
                print(f"Callback error: {e}")
    
    @abstractmethod
    def connect(self) -> bool:
        """Connect to the controller. Returns True on success."""
        pass
    
    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the controller."""
        pass
    
    @abstractmethod
    def move(self, axis_name: str, target: float, relative: bool = False, 
             feedrate: Optional[float] = None, wait: bool = True) -> MoveResult:
        """
        Move an axis to a position.
        
        Args:
            axis_name: Name of axis to move
            target: Target position (absolute or relative based on 'relative' flag)
            relative: If True, target is relative to current position
            feedrate: Optional feedrate override
            wait: If True, block until move completes
            
        Returns:
            MoveResult with success status and final position
        """
        pass
    
    @abstractmethod
    def move_multi(self, moves: List[MoveCommand], wait: bool = True) -> List[MoveResult]:
        """
        Move multiple axes simultaneously.
        
        Args:
            moves: List of MoveCommand objects
            wait: If True, block until all moves complete
            
        Returns:
            List of MoveResult objects
        """
        pass
    
    @abstractmethod
    def get_position(self, axis_name: str) -> Optional[float]:
        """Get current position of an axis."""
        pass
    
    @abstractmethod
    def get_all_positions(self) -> Dict[str, float]:
        """Get current positions of all axes."""
        pass
    
    @abstractmethod
    def home(self, axis_name: str = None) -> bool:
        """
        Home an axis or all axes.
        
        Args:
            axis_name: Axis to home, or None for all axes
            
        Returns:
            True if homing successful
        """
        pass
    
    @abstractmethod
    def zero(self, axis_name: str) -> bool:
        """Set current position as zero for an axis."""
        pass
    
    @abstractmethod
    def stop(self) -> None:
        """Emergency stop all motion."""
        pass
    
    def set_power(self, on: bool) -> bool:
        """Set motor power state (if supported). Returns True on success."""
        return True  # Default implementation does nothing
    
    def get_power(self) -> bool:
        """Get motor power state (if supported)."""
        return True  # Default: always on


# ──────────────────────────────────────────────────────────────────────────────
# ARDUINO SERIAL CONTROLLER
# ──────────────────────────────────────────────────────────────────────────────

class ArduinoSerialController(MotionController):
    """
    Controller for Arduino with custom serial protocol.
    
    Protocol:
        ROTATE <motor> <degrees> <CW|CCW>
        MOVE <motor> <mm> <FORWARD|BACKWARD>
        TILT <motor> <degrees> <UP|DOWN>
        ZERO <motor>
        GET_POS <motor>
        POWER <ON|OFF>
        GET_POWER
    """
    
    def __init__(self, config: ControllerConfig, axes: List[AxisConfig]):
        super().__init__(config, axes)
        self._serial: Optional[serial.Serial] = None
        self._power_on = False
        
        # Build motor index to axis mapping
        self._motor_to_axis: Dict[int, str] = {}
        self._axis_to_motor: Dict[str, int] = {}
        for axis in axes:
            if axis.motor_index > 0:
                self._motor_to_axis[axis.motor_index] = axis.name
                self._axis_to_motor[axis.name] = axis.motor_index
    
    def connect(self) -> bool:
        if not SERIAL_AVAILABLE:
            print("pyserial not available")
            return False
        
        try:
            self._serial = serial.Serial(
                self.config.port,
                self.config.baudrate,
                timeout=2.0
            )
            time.sleep(2.0)  # Wait for Arduino reset
            
            # Clear any startup messages
            self._serial.reset_input_buffer()
            
            # Test connection
            self._serial.write(b"GET_POWER\n")
            response = self._serial.readline().decode().strip()
            
            self._connected = True
            self._state = MotionState.IDLE
            self._power_on = (response == "ON")
            self._notify("connected", {"port": self.config.port})
            return True
            
        except Exception as e:
            print(f"Connection failed: {e}")
            self._state = MotionState.ERROR
            return False
    
    def disconnect(self) -> None:
        if self._serial:
            self._serial.close()
            self._serial = None
        self._connected = False
        self._state = MotionState.DISCONNECTED
        self._notify("disconnected")
    
    def _send_command(self, command: str, timeout: float = 10.0) -> Optional[str]:
        """Send command and wait for response."""
        if not self._serial or not self._connected:
            return None
        
        with self._lock:
            try:
                self._serial.reset_input_buffer()
                self._serial.write(f"{command}\n".encode())
                
                start = time.time()
                while time.time() - start < timeout:
                    if self._serial.in_waiting:
                        line = self._serial.readline().decode().strip()
                        return line
                    time.sleep(0.01)
                
                return None  # Timeout
                
            except Exception as e:
                print(f"Command error: {e}")
                return None
    
    def move(self, axis_name: str, target: float, relative: bool = False,
             feedrate: Optional[float] = None, wait: bool = True) -> MoveResult:
        
        axis = self.axes.get(axis_name)
        if not axis:
            return MoveResult(False, axis_name, 0.0, f"Unknown axis: {axis_name}")
        
        motor = self._axis_to_motor.get(axis_name)
        if motor is None:
            return MoveResult(False, axis_name, 0.0, f"No motor for axis: {axis_name}")
        
        # Calculate movement
        current_pos = self._positions.get(axis_name, 0.0)
        if relative:
            target_pos = current_pos + target
            delta = target
        else:
            target_pos = target
            delta = target - current_pos
        
        if abs(delta) < 0.001:
            return MoveResult(True, axis_name, current_pos, "")
        
        # Apply direction inversion
        if axis.inverted:
            delta = -delta
        
        # Build command based on axis type and role
        if axis.axis_type == AxisType.ROTARY:
            if axis.role == "camera_tilt":
                direction = "UP" if delta > 0 else "DOWN"
                command = f"TILT {motor} {abs(delta):.4f} {direction}"
            else:
                direction = "CW" if delta > 0 else "CCW"
                command = f"ROTATE {motor} {abs(delta):.4f} {direction}"
        else:
            direction = "FORWARD" if delta > 0 else "BACKWARD"
            command = f"MOVE {motor} {abs(delta):.4f} {direction}"
        
        # Send command
        self._state = MotionState.MOVING
        self._notify("moving", {"axis": axis_name, "target": target_pos})
        
        response = self._send_command(command)
        
        self._state = MotionState.IDLE
        
        if response and (response == "OK" or response.startswith("OK ")):
            self._positions[axis_name] = target_pos
            self._notify("move_complete", {"axis": axis_name, "position": target_pos})
            return MoveResult(True, axis_name, target_pos, "")
        else:
            error = response or "Timeout"
            self._notify("move_error", {"axis": axis_name, "error": error})
            return MoveResult(False, axis_name, current_pos, error)
    
    def move_multi(self, moves: List[MoveCommand], wait: bool = True) -> List[MoveResult]:
        # Arduino controller moves axes sequentially
        results = []
        for move in moves:
            result = self.move(
                move.axis_name, 
                move.target, 
                relative=move.is_relative,
                feedrate=move.feedrate,
                wait=wait
            )
            results.append(result)
        return results
    
    def get_position(self, axis_name: str) -> Optional[float]:
        motor = self._axis_to_motor.get(axis_name)
        if motor is None:
            return self._positions.get(axis_name)
        
        response = self._send_command(f"GET_POS {motor}")
        if response:
            try:
                pos = float(response)
                self._positions[axis_name] = pos
                return pos
            except ValueError:
                pass
        return self._positions.get(axis_name)
    
    def get_all_positions(self) -> Dict[str, float]:
        for axis_name in self.axes:
            self.get_position(axis_name)
        return self._positions.copy()
    
    def home(self, axis_name: str = None) -> bool:
        # Arduino doesn't have automatic homing in current firmware
        # Just zero the position
        if axis_name:
            return self.zero(axis_name)
        else:
            success = True
            for name in self.axes:
                if not self.zero(name):
                    success = False
            return success
    
    def zero(self, axis_name: str) -> bool:
        motor = self._axis_to_motor.get(axis_name)
        if motor is None:
            return False
        
        response = self._send_command(f"ZERO {motor}")
        if response == "OK":
            self._positions[axis_name] = 0.0
            return True
        return False
    
    def stop(self) -> None:
        # Arduino doesn't have emergency stop - just try to power off
        self.set_power(False)
    
    def set_power(self, on: bool) -> bool:
        command = "POWER ON" if on else "POWER OFF"
        response = self._send_command(command)
        if response and response.startswith("OK"):
            self._power_on = on
            self._notify("power_changed", {"on": on})
            return True
        return False
    
    def get_power(self) -> bool:
        response = self._send_command("GET_POWER")
        if response:
            self._power_on = (response == "ON")
        return self._power_on


# ──────────────────────────────────────────────────────────────────────────────
# GRBL CONTROLLER
# ──────────────────────────────────────────────────────────────────────────────

class GrblController(MotionController):
    """
    Controller for GRBL-based motion systems.
    Uses standard G-code commands.
    """
    
    def __init__(self, config: ControllerConfig, axes: List[AxisConfig]):
        super().__init__(config, axes)
        self._serial: Optional[serial.Serial] = None
        
        # Build axis letter mapping
        self._axis_to_letter: Dict[str, str] = {}
        for axis in axes:
            if axis.controller_axis:
                self._axis_to_letter[axis.name] = axis.controller_axis
    
    def connect(self) -> bool:
        if not SERIAL_AVAILABLE:
            return False
        
        try:
            self._serial = serial.Serial(
                self.config.port,
                self.config.baudrate,
                timeout=2.0
            )
            time.sleep(2.0)
            
            # Wait for GRBL greeting
            self._serial.reset_input_buffer()
            self._serial.write(b"\r\n\r\n")
            time.sleep(0.5)
            
            # Send status query
            self._serial.write(b"?\n")
            response = self._serial.readline().decode().strip()
            
            if response.startswith("<"):
                self._connected = True
                self._state = MotionState.IDLE
                self._notify("connected", {"port": self.config.port})
                return True
            
            return False
            
        except Exception as e:
            print(f"GRBL connection failed: {e}")
            return False
    
    def disconnect(self) -> None:
        if self._serial:
            self._serial.close()
            self._serial = None
        self._connected = False
        self._state = MotionState.DISCONNECTED
    
    def _send_gcode(self, gcode: str, wait_for_ok: bool = True) -> Optional[str]:
        """Send G-code and optionally wait for 'ok' response."""
        if not self._serial or not self._connected:
            return None
        
        with self._lock:
            try:
                self._serial.write(f"{gcode}\n".encode())
                
                if wait_for_ok:
                    while True:
                        line = self._serial.readline().decode().strip()
                        if line == "ok":
                            return "ok"
                        if line.startswith("error"):
                            return line
                        if not line:
                            return None  # Timeout
                
                return "sent"
                
            except Exception as e:
                print(f"G-code error: {e}")
                return None
    
    def _parse_status(self, status: str) -> Dict[str, float]:
        """Parse GRBL status response like <Idle|MPos:0.000,0.000,0.000|...>"""
        positions = {}
        if "|MPos:" in status:
            try:
                mpos = status.split("|MPos:")[1].split("|")[0]
                coords = mpos.split(",")
                axis_letters = ["X", "Y", "Z", "A", "B", "C"]
                for i, val in enumerate(coords):
                    if i < len(axis_letters):
                        positions[axis_letters[i]] = float(val)
            except:
                pass
        return positions
    
    def move(self, axis_name: str, target: float, relative: bool = False,
             feedrate: Optional[float] = None, wait: bool = True) -> MoveResult:
        
        axis = self.axes.get(axis_name)
        if not axis:
            return MoveResult(False, axis_name, 0.0, f"Unknown axis: {axis_name}")
        
        letter = self._axis_to_letter.get(axis_name)
        if not letter:
            return MoveResult(False, axis_name, 0.0, f"No controller axis for: {axis_name}")
        
        # Build G-code
        mode = "G91" if relative else "G90"  # Relative or absolute
        
        if feedrate is None:
            feedrate = axis.max_velocity * 60  # Convert to mm/min
        
        gcode = f"{mode} G1 {letter}{target:.4f} F{feedrate:.0f}"
        
        self._state = MotionState.MOVING
        response = self._send_gcode(gcode, wait_for_ok=wait)
        self._state = MotionState.IDLE
        
        if response == "ok":
            if not relative:
                self._positions[axis_name] = target
            else:
                self._positions[axis_name] = self._positions.get(axis_name, 0.0) + target
            return MoveResult(True, axis_name, self._positions[axis_name], "")
        else:
            return MoveResult(False, axis_name, self._positions.get(axis_name, 0.0), 
                            response or "Timeout")
    
    def move_multi(self, moves: List[MoveCommand], wait: bool = True) -> List[MoveResult]:
        """Move multiple axes in a single G-code command."""
        if not moves:
            return []
        
        # Check if all moves are same mode (relative/absolute)
        is_relative = moves[0].is_relative
        if any(m.is_relative != is_relative for m in moves):
            # Mixed modes - move sequentially
            return [self.move(m.axis_name, m.target, m.is_relative, m.feedrate, wait) 
                    for m in moves]
        
        # Build combined G-code
        mode = "G91" if is_relative else "G90"
        parts = [mode, "G1"]
        
        feedrate = None
        for move in moves:
            axis = self.axes.get(move.axis_name)
            if not axis:
                continue
            letter = self._axis_to_letter.get(move.axis_name)
            if letter:
                parts.append(f"{letter}{move.target:.4f}")
                if move.feedrate:
                    feedrate = move.feedrate
                elif feedrate is None:
                    feedrate = axis.max_velocity * 60
        
        if feedrate:
            parts.append(f"F{feedrate:.0f}")
        
        gcode = " ".join(parts)
        
        self._state = MotionState.MOVING
        response = self._send_gcode(gcode, wait_for_ok=wait)
        self._state = MotionState.IDLE
        
        results = []
        for move in moves:
            if response == "ok":
                if not move.is_relative:
                    self._positions[move.axis_name] = move.target
                else:
                    self._positions[move.axis_name] = \
                        self._positions.get(move.axis_name, 0.0) + move.target
                results.append(MoveResult(True, move.axis_name, 
                                         self._positions[move.axis_name], ""))
            else:
                results.append(MoveResult(False, move.axis_name,
                                         self._positions.get(move.axis_name, 0.0),
                                         response or "Timeout"))
        
        return results
    
    def get_position(self, axis_name: str) -> Optional[float]:
        letter = self._axis_to_letter.get(axis_name)
        if not letter:
            return self._positions.get(axis_name)
        
        # Query GRBL status
        if self._serial:
            self._serial.write(b"?\n")
            response = self._serial.readline().decode().strip()
            positions = self._parse_status(response)
            if letter in positions:
                self._positions[axis_name] = positions[letter]
        
        return self._positions.get(axis_name)
    
    def get_all_positions(self) -> Dict[str, float]:
        if self._serial:
            self._serial.write(b"?\n")
            response = self._serial.readline().decode().strip()
            grbl_positions = self._parse_status(response)
            
            # Map back to axis names
            for axis_name, letter in self._axis_to_letter.items():
                if letter in grbl_positions:
                    self._positions[axis_name] = grbl_positions[letter]
        
        return self._positions.copy()
    
    def home(self, axis_name: str = None) -> bool:
        if axis_name:
            letter = self._axis_to_letter.get(axis_name)
            if letter:
                response = self._send_gcode(f"$H{letter}")
                return response == "ok"
            return False
        else:
            response = self._send_gcode("$H")
            return response == "ok"
    
    def zero(self, axis_name: str) -> bool:
        letter = self._axis_to_letter.get(axis_name)
        if letter:
            # G92 sets current position
            response = self._send_gcode(f"G92 {letter}0")
            if response == "ok":
                self._positions[axis_name] = 0.0
                return True
        return False
    
    def stop(self) -> None:
        if self._serial:
            # Send real-time feed hold and soft reset
            self._serial.write(b"!")  # Feed hold
            time.sleep(0.1)
            self._serial.write(b"\x18")  # Ctrl-X soft reset


# ──────────────────────────────────────────────────────────────────────────────
# MOCK CONTROLLER (for testing)
# ──────────────────────────────────────────────────────────────────────────────

class MockController(MotionController):
    """Mock controller for testing without hardware."""
    
    def __init__(self, config: ControllerConfig, axes: List[AxisConfig]):
        super().__init__(config, axes)
        self._power_on = False
        self._move_delay = 0.1  # Simulated move time
    
    def connect(self) -> bool:
        self._connected = True
        self._state = MotionState.IDLE
        print(f"[MOCK] Connected to {self.config.name}")
        return True
    
    def disconnect(self) -> None:
        self._connected = False
        self._state = MotionState.DISCONNECTED
        print(f"[MOCK] Disconnected from {self.config.name}")
    
    def move(self, axis_name: str, target: float, relative: bool = False,
             feedrate: Optional[float] = None, wait: bool = True) -> MoveResult:
        
        axis = self.axes.get(axis_name)
        if not axis:
            return MoveResult(False, axis_name, 0.0, f"Unknown axis: {axis_name}")
        
        current = self._positions.get(axis_name, 0.0)
        if relative:
            target = current + target
        
        print(f"[MOCK] Moving {axis_name}: {current:.3f} -> {target:.3f} {axis.units}")
        
        if wait:
            time.sleep(self._move_delay)
        
        self._positions[axis_name] = target
        return MoveResult(True, axis_name, target, "")
    
    def move_multi(self, moves: List[MoveCommand], wait: bool = True) -> List[MoveResult]:
        results = []
        for move in moves:
            result = self.move(move.axis_name, move.target, move.is_relative,
                              move.feedrate, wait=False)
            results.append(result)
        if wait:
            time.sleep(self._move_delay)
        return results
    
    def get_position(self, axis_name: str) -> Optional[float]:
        return self._positions.get(axis_name, 0.0)
    
    def get_all_positions(self) -> Dict[str, float]:
        return self._positions.copy()
    
    def home(self, axis_name: str = None) -> bool:
        if axis_name:
            print(f"[MOCK] Homing {axis_name}")
            self._positions[axis_name] = 0.0
        else:
            print(f"[MOCK] Homing all axes")
            for name in self.axes:
                self._positions[name] = 0.0
        return True
    
    def zero(self, axis_name: str) -> bool:
        print(f"[MOCK] Zeroing {axis_name}")
        self._positions[axis_name] = 0.0
        return True
    
    def stop(self) -> None:
        print(f"[MOCK] Emergency stop!")
        self._state = MotionState.IDLE
    
    def set_power(self, on: bool) -> bool:
        self._power_on = on
        print(f"[MOCK] Power {'ON' if on else 'OFF'}")
        return True
    
    def get_power(self) -> bool:
        return self._power_on


# ──────────────────────────────────────────────────────────────────────────────
# CONTROLLER FACTORY
# ──────────────────────────────────────────────────────────────────────────────

def create_controller(config: ControllerConfig, axes: List[AxisConfig]) -> MotionController:
    """
    Factory function to create appropriate controller based on config.
    """
    if config.controller_type == ControllerType.ARDUINO_SERIAL:
        return ArduinoSerialController(config, axes)
    elif config.controller_type == ControllerType.GRBL:
        return GrblController(config, axes)
    elif config.controller_type == ControllerType.MOCK:
        return MockController(config, axes)
    else:
        raise ValueError(f"Unsupported controller type: {config.controller_type}")


# ──────────────────────────────────────────────────────────────────────────────
# TESTING
# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    from axis_config import create_raspberry_pi_3axis_profile, create_sherline_6axis_profile
    
    print("Testing Motion Controllers...\n")
    
    # Test with mock controller
    profile = create_raspberry_pi_3axis_profile()
    
    # Override to use mock controller
    profile.controllers[0].controller_type = ControllerType.MOCK
    
    controller = create_controller(profile.controllers[0], profile.axes)
    controller.connect()
    
    print("\nTesting moves:")
    controller.move("M1", 45.0)
    controller.move("M2", 10.0)
    controller.move("M3", 30.0)
    
    print(f"\nPositions: {controller.get_all_positions()}")
    
    controller.home()
    print(f"After home: {controller.get_all_positions()}")
    
    controller.disconnect()
    print("\n✓ Mock controller test passed!")
