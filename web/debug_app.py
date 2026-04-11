"""
Scanner Debug Mode — FastAPI Web Application

Web interface for testing motor controllers with any scanner profile.
Supports mock mode for development without hardware.
"""

from motion_controller import create_controller, MotionController
from axis_config import (
    ProfileManager, MachineProfile, ControllerType, ControllerConfig,
)
from typing import Optional, Dict, List
from pydantic import BaseModel
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
from fastapi import FastAPI, HTTPException, Request
import sys
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))


# ── Request Models ────────────────────────────────────────────────────────────

class ConnectRequest(BaseModel):
    profile: str
    mock: bool = True


class JogRequest(BaseModel):
    distance: float


class MoveToRequest(BaseModel):
    position: float


class PowerRequest(BaseModel):
    on: bool


# ── Application State ─────────────────────────────────────────────────────────

class AppState:
    def __init__(self):
        self.profile_manager = ProfileManager()
        self.profile: Optional[MachineProfile] = None
        self.controllers: Dict[str, MotionController] = {}
        self.connected = False
        self.mock_mode = False

    def get_controller_for_axis(self, axis_name: str) -> Optional[MotionController]:
        if not self.profile:
            return None
        controller_name = self.profile.axis_controller_map.get(axis_name)
        return self.controllers.get(controller_name)


# ── FastAPI App ───────────────────────────────────────────────────────────────

WEB_DIR = Path(__file__).parent

app = FastAPI(title="Scanner Debug Mode")
app.mount("/static", StaticFiles(directory=str(WEB_DIR / "static")), name="static")
templates = Jinja2Templates(directory=str(WEB_DIR / "templates"))

state = AppState()


# ── Pages ─────────────────────────────────────────────────────────────────────

@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    profiles = state.profile_manager.list_profiles()
    return templates.TemplateResponse("debug.html", {
        "request": request,
        "profiles": profiles,
    })


# ── Profile API ───────────────────────────────────────────────────────────────

@app.get("/api/profiles")
async def list_profiles():
    return state.profile_manager.list_profiles()


@app.get("/api/profiles/{name}")
async def get_profile(name: str):
    try:
        profile = state.profile_manager.load_profile(name)
        return profile.to_dict()
    except ValueError:
        raise HTTPException(404, f"Profile not found: {name}")


# ── Connection API ────────────────────────────────────────────────────────────

@app.post("/api/connect")
async def connect(req: ConnectRequest):
    # Disconnect existing
    if state.connected:
        await disconnect()

    try:
        profile = state.profile_manager.load_profile(req.profile)
    except ValueError:
        raise HTTPException(404, f"Profile not found: {req.profile}")

    state.profile = profile
    state.mock_mode = req.mock
    state.controllers = {}

    # Group axes by controller
    controller_axes: Dict[str, list] = {}
    for axis in profile.axes:
        ctrl_name = profile.axis_controller_map.get(axis.name, "default")
        controller_axes.setdefault(ctrl_name, []).append(axis)

    # Create and connect controllers
    for ctrl_config in profile.controllers:
        axes = controller_axes.get(ctrl_config.name, [])
        if not axes:
            continue

        if req.mock:
            mock_config = ControllerConfig(
                controller_type=ControllerType.MOCK,
                name=ctrl_config.name,
                port=ctrl_config.port,
                baudrate=ctrl_config.baudrate,
            )
            controller = create_controller(mock_config, axes)
        else:
            controller = create_controller(ctrl_config, axes)

        if controller.connect():
            state.controllers[ctrl_config.name] = controller
        else:
            for c in state.controllers.values():
                c.disconnect()
            state.controllers = {}
            raise HTTPException(
                500, f"Failed to connect controller: {ctrl_config.name}"
            )

    state.connected = True
    return {"status": "connected", "profile": profile.name, "mock": req.mock}


@app.post("/api/disconnect")
async def disconnect():
    for controller in state.controllers.values():
        controller.disconnect()
    state.controllers = {}
    state.profile = None
    state.connected = False
    state.mock_mode = False
    return {"status": "disconnected"}


# ── Status API ────────────────────────────────────────────────────────────────

@app.get("/api/status")
async def get_status():
    if not state.connected or not state.profile:
        return {
            "connected": False,
            "profile": None,
            "axes": [],
            "positions": {},
            "power": False,
        }

    positions = {}
    for controller in state.controllers.values():
        positions.update(controller.get_all_positions())

    power = False
    for controller in state.controllers.values():
        power = controller.get_power()
        break

    return {
        "connected": True,
        "profile": state.profile.name,
        "axes": [a.to_dict() for a in state.profile.axes],
        "positions": positions,
        "power": power,
    }


# ── Motor Control API ─────────────────────────────────────────────────────────

@app.post("/api/axes/{axis_name}/jog")
async def jog_axis(axis_name: str, req: JogRequest):
    if not state.connected:
        raise HTTPException(400, "Not connected")

    controller = state.get_controller_for_axis(axis_name)
    if not controller:
        raise HTTPException(404, f"No controller for axis: {axis_name}")

    result = controller.move(axis_name, req.distance, relative=True)
    return {
        "success": result.success,
        "position": result.final_position,
        "error": result.error_message,
    }


@app.post("/api/axes/{axis_name}/move-to")
async def move_to_axis(axis_name: str, req: MoveToRequest):
    if not state.connected:
        raise HTTPException(400, "Not connected")

    controller = state.get_controller_for_axis(axis_name)
    if not controller:
        raise HTTPException(404, f"No controller for axis: {axis_name}")

    result = controller.move(axis_name, req.position, relative=False)
    return {
        "success": result.success,
        "position": result.final_position,
        "error": result.error_message,
    }


@app.post("/api/axes/{axis_name}/home")
async def home_axis(axis_name: str):
    if not state.connected:
        raise HTTPException(400, "Not connected")

    controller = state.get_controller_for_axis(axis_name)
    if not controller:
        raise HTTPException(404, f"No controller for axis: {axis_name}")

    success = controller.home(axis_name)
    pos = controller.get_position(axis_name)
    return {"success": success, "position": pos or 0.0}


@app.post("/api/axes/{axis_name}/zero")
async def zero_axis(axis_name: str):
    if not state.connected:
        raise HTTPException(400, "Not connected")

    controller = state.get_controller_for_axis(axis_name)
    if not controller:
        raise HTTPException(404, f"No controller for axis: {axis_name}")

    success = controller.zero(axis_name)
    return {"success": success, "position": 0.0}


@app.post("/api/power")
async def set_power(req: PowerRequest):
    if not state.connected:
        raise HTTPException(400, "Not connected")

    success = False
    for controller in state.controllers.values():
        if controller.set_power(req.on):
            success = True
            break
    return {"success": success, "power": req.on}


@app.post("/api/stop")
async def emergency_stop():
    for controller in state.controllers.values():
        controller.stop()
    return {"status": "stopped"}
