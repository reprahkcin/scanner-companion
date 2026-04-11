"""Scan orchestrator for scanner-companion.

Provides a reusable `Scanner` class with a state machine that runs ring or
spherical capture sequences in a background thread.  The class is UI-agnostic:
it communicates via callbacks and accepts motion/camera dependencies as
constructor arguments, making it usable from both Tkinter and a web API.
"""

from __future__ import annotations

import json
import os
import shutil
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum, auto
from typing import Any, Callable, Dict, List, Optional, Tuple

from camera import BaseCamera

# XMP pose helpers (dependency-free module)
from xmp_pose import (
    RigPose,
    ring_pose,
    spherical_pose,
    write_xmp_sidecar,
    VERIFIED_FOCAL_LENGTH_35MM,
    VERIFIED_DISTORTION_MODEL,
    VERIFIED_SKEW,
    VERIFIED_ASPECT_RATIO,
    VERIFIED_PRINCIPAL_POINT_U,
    VERIFIED_PRINCIPAL_POINT_V,
    VERIFIED_DISTORTION_COEFFS,
    VERIFIED_POSE_PRIOR,
    VERIFIED_CALIB_PRIOR,
)


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

class ScanState(Enum):
    IDLE = auto()
    RUNNING = auto()
    PAUSED = auto()
    ABORTING = auto()
    COMPLETED = auto()
    ERROR = auto()


# ---------------------------------------------------------------------------
# Scan parameters
# ---------------------------------------------------------------------------

@dataclass
class ScanParams:
    """All parameters needed to fully describe a scan run."""
    scan_mode: str = "ring"                     # "ring" or "spherical"

    # Session
    specimen_name: str = "specimen_001"
    output_dir: str = "."
    image_format: str = "JPG"                   # JPG, PNG, TIFF
    capture_resolution: Tuple[int, int] = (4056, 3040)

    # Ring / common
    perspectives: int = 72                      # angles around 360°
    focus_slices: int = 5                       # per perspective
    settle_delay: float = 1.0                   # seconds after move

    # Spherical extras
    elevation_min: float = -60.0
    elevation_max: float = 60.0
    elevation_steps: int = 5

    # XMP / pose geometry
    lens_to_object_mm: float = 250.0
    rail_to_horizon_deg: float = 0.0
    xmp_position_scale: float = 1.0
    distortion_coefficients: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    # Calibration data: {angle: {"near": mm, "far": mm}}
    calibration_data: Dict[float, Dict[str, float]
                           ] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Motion interface (callbacks the Scanner calls)
# ---------------------------------------------------------------------------

@dataclass
class MotionCallbacks:
    """Callable hooks for motor movement.

    Each callable should return ``True`` on success, ``False`` on failure.
    """
    move_to_angle: Callable[[float], bool] = lambda a: True
    move_to_elevation: Callable[[float], bool] = lambda e: True
    move_to_focus_position: Callable[[float], bool] = lambda p: True
    interpolate_focus_position: Callable[[
        float, float], float] = lambda angle, t: 0.0


# ---------------------------------------------------------------------------
# Progress / status
# ---------------------------------------------------------------------------

@dataclass
class ScanStatus:
    state: ScanState = ScanState.IDLE
    total_images: int = 0
    captured_images: int = 0
    progress_pct: float = 0.0
    current_perspective: int = 0
    current_angle: float = 0.0
    current_elevation: float = 0.0
    session_dir: str = ""
    elapsed_s: float = 0.0
    message: str = ""


# ---------------------------------------------------------------------------
# Scanner
# ---------------------------------------------------------------------------

class Scanner:
    """UI-agnostic scan orchestrator with background-thread execution."""

    def __init__(
        self,
        camera: BaseCamera,
        motion: MotionCallbacks,
        on_status: Optional[Callable[[ScanStatus], None]] = None,
        on_log: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.camera = camera
        self.motion = motion
        self._on_status = on_status or (lambda s: None)
        self._on_log = on_log or (lambda m: None)

        self._state = ScanState.IDLE
        self._lock = threading.Lock()
        self._pause_event = threading.Event()
        self._pause_event.set()  # not paused initially
        self._thread: Optional[threading.Thread] = None
        self._status = ScanStatus()
        self._start_time = 0.0

    # -- public API ---------------------------------------------------------

    @property
    def state(self) -> ScanState:
        return self._state

    def get_status(self) -> ScanStatus:
        with self._lock:
            s = self._status
            if self._start_time:
                s.elapsed_s = time.time() - self._start_time
            return s

    def start(self, params: ScanParams) -> None:
        """Launch a scan in a background thread."""
        if self._state not in (ScanState.IDLE, ScanState.COMPLETED, ScanState.ERROR):
            raise RuntimeError(f"Cannot start scan from state {self._state}")
        self._state = ScanState.RUNNING
        self._pause_event.set()
        self._thread = threading.Thread(
            target=self._run, args=(params,), daemon=True)
        self._thread.start()

    def pause(self) -> None:
        if self._state == ScanState.RUNNING:
            self._state = ScanState.PAUSED
            self._pause_event.clear()
            self._emit_status("Scan paused")

    def resume(self) -> None:
        if self._state == ScanState.PAUSED:
            self._state = ScanState.RUNNING
            self._pause_event.set()
            self._emit_status("Scan resumed")

    def abort(self) -> None:
        if self._state in (ScanState.RUNNING, ScanState.PAUSED):
            self._state = ScanState.ABORTING
            self._pause_event.set()  # unblock if paused so thread can exit
            self._emit_status("Aborting scan…")

    # -- internal helpers ---------------------------------------------------

    def _log(self, msg: str) -> None:
        self._on_log(msg)

    def _emit_status(self, message: str = "") -> None:
        with self._lock:
            self._status.state = self._state
            self._status.message = message
            if self._start_time:
                self._status.elapsed_s = time.time() - self._start_time
        self._on_status(self._status)

    def _check_abort(self) -> bool:
        """Return True if scan should stop.  Blocks while paused."""
        self._pause_event.wait()
        return self._state == ScanState.ABORTING

    # -- scan runner --------------------------------------------------------

    def _run(self, params: ScanParams) -> None:
        """Main scan thread entry point."""
        self._start_time = time.time()
        session_dir = ""
        try:
            # Build session directory
            specimen_dir = os.path.join(
                params.output_dir, params.specimen_name)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            session_dir = os.path.join(specimen_dir, f"session_{timestamp}")
            os.makedirs(session_dir, exist_ok=True)
            self._status.session_dir = session_dir

            # Write metadata
            metadata = self._build_metadata(params, timestamp)
            with open(os.path.join(session_dir, "metadata.json"), "w") as f:
                json.dump(metadata, f, indent=2)

            self._log(f"Session created: {session_dir}")

            # Dispatch
            if params.scan_mode == "ring":
                self._run_ring(params, session_dir)
            elif params.scan_mode == "spherical":
                self._run_spherical(params, session_dir)
            else:
                raise ValueError(f"Unknown scan mode: {params.scan_mode}")

            if self._state == ScanState.ABORTING:
                self._state = ScanState.IDLE
                self._log("Scan aborted by user")
            else:
                self._state = ScanState.COMPLETED
                self._log("Scan completed")

        except Exception as exc:
            self._state = ScanState.ERROR
            self._log(f"Scan error: {exc}")

        self._emit_status()

    # -- ring scan ----------------------------------------------------------

    def _run_ring(self, p: ScanParams, session_dir: str) -> None:
        total = p.perspectives * p.focus_slices
        self._status.total_images = total
        self._emit_status(f"Starting ring scan: {total} images")

        angle_step = 360.0 / max(1, p.perspectives)
        pad = max(2, len(str(max(0, p.perspectives - 1))))

        if not self.motion.move_to_angle(0.0):
            raise RuntimeError("Failed to home turntable to 0°")

        image_count = 0
        for perspective in range(p.perspectives):
            if self._check_abort():
                break

            angle = perspective * angle_step
            self._status.current_perspective = perspective
            self._status.current_angle = angle

            if not self.motion.move_to_angle(angle):
                self._log(f"Failed to move to angle {angle}")
                break

            for slice_idx in range(p.focus_slices):
                if self._check_abort():
                    break

                stack_dir = os.path.join(
                    session_dir, f"stack_{perspective:0{pad}d}")
                os.makedirs(stack_dir, exist_ok=True)

                t = (slice_idx / (p.focus_slices - 1)
                     ) if p.focus_slices > 1 else 0.0
                focus_pos = self.motion.interpolate_focus_position(angle, t)

                if not self.motion.move_to_focus_position(focus_pos):
                    self._log(f"Failed to move focus to {focus_pos:.2f}mm")
                    break

                time.sleep(p.settle_delay)

                ext = p.image_format.lower()
                filename = f"stack_{perspective:0{pad}d}_shot_{slice_idx:04d}_angle_{angle:06.2f}.{ext}"
                filepath = os.path.join(stack_dir, filename)
                self.camera.capture_image(
                    filepath, resolution=p.capture_resolution, fmt=p.image_format)

                image_count += 1
                self._status.captured_images = image_count
                self._status.progress_pct = (image_count / total) * 100
                if image_count % 10 == 0:
                    self._emit_status(f"Captured {image_count}/{total}")

            # XMP for this perspective
            self._generate_ring_xmp(p, session_dir, perspective, angle, pad)

        self._consolidate_xmp(p, session_dir, image_count, angle_step, pad)

    # -- spherical scan -----------------------------------------------------

    def _run_spherical(self, p: ScanParams, session_dir: str) -> None:
        elev_count = p.elevation_steps
        azim_count = p.perspectives
        focus_slices = p.focus_slices

        if elev_count == 1:
            elevations = [(p.elevation_min + p.elevation_max) / 2.0]
        else:
            step = (p.elevation_max - p.elevation_min) / (elev_count - 1)
            elevations = [p.elevation_min + i *
                          step for i in range(elev_count)]

        azim_step = 360.0 / max(1, azim_count)
        total = elev_count * azim_count * focus_slices
        self._status.total_images = total
        self._emit_status(f"Starting spherical scan: {total} images")

        persp_total = elev_count * azim_count
        pad = max(2, len(str(max(0, persp_total - 1))))

        if not self.motion.move_to_angle(0.0):
            raise RuntimeError("Failed to home turntable to 0°")
        if not self.motion.move_to_elevation(elevations[0]):
            raise RuntimeError(f"Failed to move to elevation {elevations[0]}°")

        image_count = 0
        persp_idx = 0

        for elevation in elevations:
            if self._check_abort():
                break

            if not self.motion.move_to_elevation(elevation):
                self._log(f"Failed to elevate to {elevation}°")
                break

            self._status.current_elevation = elevation

            for azim_idx in range(azim_count):
                if self._check_abort():
                    break

                azimuth = azim_idx * azim_step
                self._status.current_angle = azimuth
                self._status.current_perspective = persp_idx

                if not self.motion.move_to_angle(azimuth):
                    self._log(f"Failed to move to azimuth {azimuth}°")
                    break

                for slice_idx in range(focus_slices):
                    if self._check_abort():
                        break

                    stack_dir = os.path.join(
                        session_dir, f"stack_{persp_idx:0{pad}d}")
                    os.makedirs(stack_dir, exist_ok=True)

                    t = (slice_idx / (focus_slices - 1)
                         ) if focus_slices > 1 else 0.0
                    focus_pos = self.motion.interpolate_focus_position(
                        azimuth, t)

                    if not self.motion.move_to_focus_position(focus_pos):
                        self._log(f"Failed to move focus to {focus_pos:.2f}mm")
                        break

                    time.sleep(p.settle_delay)

                    ext = p.image_format.lower()
                    filename = (
                        f"stack_{persp_idx:0{pad}d}_shot_{slice_idx:04d}"
                        f"_elev_{elevation:+06.2f}_azim_{azimuth:06.2f}.{ext}"
                    )
                    filepath = os.path.join(stack_dir, filename)
                    self.camera.capture_image(
                        filepath, resolution=p.capture_resolution, fmt=p.image_format)

                    image_count += 1
                    self._status.captured_images = image_count
                    self._status.progress_pct = (image_count / total) * 100
                    if image_count % 10 == 0:
                        self._emit_status(f"Captured {image_count}/{total}")

                self._generate_spherical_xmp(
                    p, session_dir, persp_idx, azimuth, elevation, pad)
                persp_idx += 1

        self._consolidate_xmp(p, session_dir, image_count, azim_step, pad)

    # -- XMP generation -----------------------------------------------------

    def _generate_ring_xmp(self, p: ScanParams, session_dir: str,
                           perspective: int, angle: float, pad: int) -> None:
        try:
            scaled = p.lens_to_object_mm * p.xmp_position_scale / 1000.0
            camera_angle = -angle  # turntable CCW → camera CW
            pose = ring_pose(scaled, p.rail_to_horizon_deg, camera_angle)

            stack_dir = os.path.join(
                session_dir, f"stack_{perspective:0{pad}d}")
            xmp_name = f"stack_{perspective:0{pad}d}_angle_{angle:06.2f}.xmp"
            xmp_path = os.path.join(stack_dir, xmp_name)

            write_xmp_sidecar(
                xmp_path.replace(".xmp", ".jpg"),
                pose,
                p.lens_to_object_mm,
                p.rail_to_horizon_deg,
                angle,
                perspective,
                focal_length_35mm=VERIFIED_FOCAL_LENGTH_35MM,
                distortion_model=VERIFIED_DISTORTION_MODEL,
                skew=VERIFIED_SKEW,
                aspect_ratio=VERIFIED_ASPECT_RATIO,
                principal_point_u=VERIFIED_PRINCIPAL_POINT_U,
                principal_point_v=VERIFIED_PRINCIPAL_POINT_V,
                distortion_coefficients=VERIFIED_DISTORTION_COEFFS,
                position_scale=1.0,
                pose_prior=VERIFIED_POSE_PRIOR,
                calibration_prior=VERIFIED_CALIB_PRIOR,
            )
            self._log(f"XMP generated for stack {perspective} at {angle:.2f}°")
        except Exception as exc:
            self._log(f"XMP warning (stack {perspective}): {exc}")

    def _generate_spherical_xmp(self, p: ScanParams, session_dir: str,
                                persp_idx: int, azimuth: float,
                                elevation: float, pad: int) -> None:
        try:
            scaled = p.lens_to_object_mm * p.xmp_position_scale / 1000.0
            pose = spherical_pose(scaled, azimuth, elevation)

            stack_dir = os.path.join(session_dir, f"stack_{persp_idx:0{pad}d}")
            xmp_name = f"stack_{persp_idx:0{pad}d}_elev_{elevation:+06.2f}_azim_{azimuth:06.2f}.xmp"
            xmp_path = os.path.join(stack_dir, xmp_name)

            write_xmp_sidecar(
                xmp_path.replace(".xmp", ".jpg"),
                pose,
                p.lens_to_object_mm,
                elevation,
                azimuth,
                persp_idx,
                focal_length_35mm=VERIFIED_FOCAL_LENGTH_35MM,
                distortion_model=VERIFIED_DISTORTION_MODEL,
                skew=VERIFIED_SKEW,
                aspect_ratio=VERIFIED_ASPECT_RATIO,
                principal_point_u=VERIFIED_PRINCIPAL_POINT_U,
                principal_point_v=VERIFIED_PRINCIPAL_POINT_V,
                distortion_coefficients=VERIFIED_DISTORTION_COEFFS,
                position_scale=1.0,
                pose_prior=VERIFIED_POSE_PRIOR,
                calibration_prior=VERIFIED_CALIB_PRIOR,
            )
            self._log(
                f"XMP generated for stack {persp_idx} (elev={elevation:.1f}°, azim={azimuth:.1f}°)")
        except Exception as exc:
            self._log(f"XMP warning (stack {persp_idx}): {exc}")

    def _consolidate_xmp(self, p: ScanParams, session_dir: str,
                         image_count: int, angle_step: float, pad: int) -> None:
        try:
            xmp_dir = os.path.join(session_dir, "xmp_files")
            os.makedirs(xmp_dir, exist_ok=True)

            total_perspectives = (
                p.perspectives if p.scan_mode == "ring"
                else p.elevation_steps * p.perspectives
            )
            out_pad = max(2, len(str(max(0, total_perspectives - 1))))

            xmp_count = 0
            for idx in range(total_perspectives):
                stack_dir = os.path.join(session_dir, f"stack_{idx:0{pad}d}")
                if not os.path.isdir(stack_dir):
                    continue
                for fname in os.listdir(stack_dir):
                    if fname.endswith(".xmp"):
                        src = os.path.join(stack_dir, fname)
                        dst = os.path.join(
                            xmp_dir, f"stack_{idx:0{out_pad}d}.xmp")
                        shutil.copy2(src, dst)
                        xmp_count += 1
                        break

            # Copy helper templates if available
            script_dir = os.path.dirname(os.path.abspath(__file__))
            for tpl in ("rename_xmp_for_rc.py", "SESSION_README.md"):
                src = os.path.join(script_dir, "templates", tpl)
                if os.path.exists(src):
                    dst_name = "README.md" if tpl == "SESSION_README.md" else tpl
                    shutil.copy2(src, os.path.join(session_dir, dst_name))

            self._log(f"Consolidated {xmp_count} XMP files to {xmp_dir}")
        except Exception as exc:
            self._log(f"XMP consolidation warning: {exc}")

        self._log(
            f"Capture complete: {image_count} images saved to {session_dir}")

    # -- metadata -----------------------------------------------------------

    @staticmethod
    def _build_metadata(p: ScanParams, timestamp: str) -> Dict[str, Any]:
        meta: Dict[str, Any] = {
            "specimen_name": p.specimen_name,
            "timestamp": timestamp,
            "scan_mode": p.scan_mode,
            "perspectives": p.perspectives,
            "focus_slices_per_perspective": p.focus_slices,
            "angle_step_degrees": 360.0 / max(1, p.perspectives),
            "settle_delay": p.settle_delay,
            "image_format": p.image_format,
            "calibration_data": {str(k): v for k, v in p.calibration_data.items()},
            "xmp_settings": {
                "lens_to_object_mm": p.lens_to_object_mm,
                "rail_to_horizon_deg": p.rail_to_horizon_deg,
            },
            "xmp_consolidation": {"enabled": True, "directory": "xmp_files"},
        }
        if p.scan_mode == "spherical":
            meta["spherical_settings"] = {
                "elevation_min": p.elevation_min,
                "elevation_max": p.elevation_max,
                "elevation_steps": p.elevation_steps,
            }
        return meta
