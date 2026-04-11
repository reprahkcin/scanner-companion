"""Camera abstraction layer for scanner-companion.

Provides a common interface (`BaseCamera`) that decouples capture logic from
any specific camera library.  The `PiCameraHQ` implementation wraps picamera2
for the Raspberry Pi HQ Camera module.
"""

from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Callable, Dict, List, Optional, Tuple

if TYPE_CHECKING:
    import numpy as np
    from numpy import ndarray
else:
    try:
        import numpy as np
        from numpy import ndarray
    except ImportError:
        np = None
        ndarray = None

# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class CameraSettings:
    """Mutable bag of camera controls.

    All fields are optional – only non-None values are applied.
    """
    # Exposure
    shutter_speed_us: Optional[int] = None        # 0 = auto
    ae_enable: Optional[bool] = None
    analogue_gain: Optional[float] = None
    ev_compensation: Optional[float] = None
    brightness: Optional[float] = None             # -1.0 … +1.0

    # White balance
    awb_enable: Optional[bool] = None
    wb_red_gain: Optional[float] = None
    wb_blue_gain: Optional[float] = None

    # Image processing
    contrast: Optional[float] = None               # 0.0 … 2.0
    saturation: Optional[float] = None             # 0.0 … 2.0
    sharpness: Optional[float] = None              # 0.0 … 2.0


# ---------------------------------------------------------------------------
# Abstract base
# ---------------------------------------------------------------------------

class BaseCamera(ABC):
    """Hardware-agnostic camera interface."""

    # -- lifecycle ----------------------------------------------------------

    @abstractmethod
    def connect(self) -> None:
        """Initialise the camera hardware."""

    @abstractmethod
    def disconnect(self) -> None:
        """Release all camera resources."""

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        ...

    # -- streaming / preview ------------------------------------------------

    @abstractmethod
    def start_stream(self, resolution: Tuple[int, int] = (640, 480)) -> None:
        """Begin low-latency preview streaming at *resolution*."""

    @abstractmethod
    def stop_stream(self) -> None:
        """Stop preview streaming (camera stays connected)."""

    @property
    @abstractmethod
    def is_streaming(self) -> bool:
        ...

    @abstractmethod
    def get_frame(self) -> ndarray:
        """Return the latest RGB frame as an ``(H, W, 3)`` uint8 array.

        Raises ``RuntimeError`` if the camera is not streaming.
        """

    # -- still capture ------------------------------------------------------

    @abstractmethod
    def capture_image(self, filepath: str, resolution: Tuple[int, int] | None = None,
                      fmt: str = "JPG") -> None:
        """Capture a still image and write it to *filepath*.

        Parameters
        ----------
        filepath : str
            Destination path (extension is informational; *fmt* controls encoding).
        resolution : tuple, optional
            (width, height) for capture.  ``None`` = sensor default / max.
        fmt : str
            ``"JPG"``, ``"PNG"``, or ``"TIFF"``.
        """

    @abstractmethod
    def capture_array(self, resolution: Tuple[int, int] | None = None) -> ndarray:
        """Capture a still and return the raw RGB numpy array."""

    # -- settings -----------------------------------------------------------

    @abstractmethod
    def apply_settings(self, settings: CameraSettings) -> None:
        """Push *settings* to the running camera (non-None fields only)."""

    @abstractmethod
    def get_metadata(self) -> Dict[str, Any]:
        """Return current camera metadata / properties dict."""

    @abstractmethod
    def get_properties(self) -> Dict[str, Any]:
        """Return static camera properties (sensor size, model, etc.)."""


# ---------------------------------------------------------------------------
# Raspberry Pi HQ Camera (picamera2)
# ---------------------------------------------------------------------------

class PiCameraHQ(BaseCamera):
    """Concrete camera backed by ``picamera2`` on Raspberry Pi."""

    def __init__(self) -> None:
        self._cam = None                       # Picamera2 instance
        self._lock = threading.Lock()
        self._streaming = False
        self._preview_config = None            # saved for mode-switch-and-return

    # -- lifecycle ----------------------------------------------------------

    def connect(self) -> None:
        from picamera2 import Picamera2
        with self._lock:
            if self._cam is None:
                self._cam = Picamera2()

    def disconnect(self) -> None:
        with self._lock:
            if self._cam is not None:
                try:
                    self._cam.stop()
                except Exception:
                    pass
                try:
                    self._cam.close()
                except Exception:
                    pass
                self._cam = None
                self._streaming = False

    @property
    def is_connected(self) -> bool:
        return self._cam is not None

    # -- streaming ----------------------------------------------------------

    def start_stream(self, resolution: Tuple[int, int] = (640, 480)) -> None:
        with self._lock:
            if self._cam is None:
                raise RuntimeError("Camera not connected")
            self._preview_config = self._cam.create_preview_configuration(
                main={"size": resolution, "format": "RGB888"}
            )
            self._cam.configure(self._preview_config)
            self._cam.start()
            self._streaming = True

    def stop_stream(self) -> None:
        with self._lock:
            if self._cam is not None and self._streaming:
                self._cam.stop()
            self._streaming = False

    @property
    def is_streaming(self) -> bool:
        return self._streaming

    def get_frame(self) -> ndarray:
        with self._lock:
            if not self._streaming:
                raise RuntimeError("Camera is not streaming")
            return self._cam.capture_array("main")

    # -- still capture ------------------------------------------------------

    def capture_image(self, filepath: str, resolution: Tuple[int, int] | None = None,
                      fmt: str = "JPG") -> None:
        with self._lock:
            if self._cam is None:
                raise RuntimeError("Camera not connected")

            still_config = self._cam.create_still_configuration(
                main={"size": resolution} if resolution else {}
            )

            if fmt.upper() == "JPG":
                try:
                    self._cam.switch_mode_and_capture_file(still_config, filepath)
                finally:
                    if self._streaming and self._preview_config:
                        try:
                            self._cam.switch_mode(self._preview_config)
                        except Exception:
                            pass
            else:
                try:
                    array = self._cam.switch_mode_and_capture_array(still_config, "main")
                finally:
                    if self._streaming and self._preview_config:
                        try:
                            self._cam.switch_mode(self._preview_config)
                        except Exception:
                            pass
                from PIL import Image
                Image.fromarray(array).save(filepath)

    def capture_array(self, resolution: Tuple[int, int] | None = None) -> ndarray:
        with self._lock:
            if self._cam is None:
                raise RuntimeError("Camera not connected")
            still_config = self._cam.create_still_configuration(
                main={"size": resolution} if resolution else {}
            )
            try:
                return self._cam.switch_mode_and_capture_array(still_config, "main")
            finally:
                if self._streaming and self._preview_config:
                    try:
                        self._cam.switch_mode(self._preview_config)
                    except Exception:
                        pass

    # -- settings -----------------------------------------------------------

    def apply_settings(self, settings: CameraSettings) -> None:
        with self._lock:
            if self._cam is None:
                return
            controls: Dict[str, Any] = {}

            # Exposure
            if settings.shutter_speed_us is not None and settings.shutter_speed_us > 0:
                controls["ExposureTime"] = settings.shutter_speed_us

            if settings.ae_enable is not None:
                controls["AeEnable"] = settings.ae_enable
                if settings.ae_enable and settings.ev_compensation is not None:
                    if abs(settings.ev_compensation) > 0.01:
                        controls["ExposureValue"] = float(settings.ev_compensation)
                elif not settings.ae_enable and settings.analogue_gain is not None:
                    if settings.analogue_gain >= 1.0:
                        controls["AnalogueGain"] = float(settings.analogue_gain)

            if settings.brightness is not None and abs(settings.brightness) > 0.01:
                controls["Brightness"] = settings.brightness

            # White balance
            if settings.awb_enable is not None:
                controls["AwbEnable"] = settings.awb_enable
                if not settings.awb_enable:
                    r = settings.wb_red_gain or 1.8
                    b = settings.wb_blue_gain or 1.5
                    controls["ColourGains"] = (float(r), float(b))

            # Image processing
            if settings.contrast is not None and abs(settings.contrast - 1.0) > 0.01:
                controls["Contrast"] = float(settings.contrast)
            if settings.saturation is not None and abs(settings.saturation - 1.0) > 0.01:
                controls["Saturation"] = float(settings.saturation)
            if settings.sharpness is not None and abs(settings.sharpness - 1.0) > 0.01:
                controls["Sharpness"] = float(settings.sharpness)

            if controls:
                self._cam.set_controls(controls)

    def get_metadata(self) -> Dict[str, Any]:
        with self._lock:
            if self._cam is None:
                return {}
            try:
                return dict(self._cam.capture_metadata())
            except Exception:
                return {}

    def get_properties(self) -> Dict[str, Any]:
        with self._lock:
            if self._cam is None:
                return {}
            try:
                return dict(self._cam.camera_properties)
            except Exception:
                return {}


# ---------------------------------------------------------------------------
# Mock camera for development / testing without hardware
# ---------------------------------------------------------------------------

class MockCamera(BaseCamera):
    """Fake camera that generates solid-colour frames for testing."""

    def __init__(self) -> None:
        self._connected = False
        self._streaming = False
        self._resolution = (640, 480)

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False
        self._streaming = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def start_stream(self, resolution: Tuple[int, int] = (640, 480)) -> None:
        self._resolution = resolution
        self._streaming = True

    def stop_stream(self) -> None:
        self._streaming = False

    @property
    def is_streaming(self) -> bool:
        return self._streaming

    def get_frame(self) -> ndarray:
        h, w = self._resolution[1], self._resolution[0]
        if np is not None:
            return np.zeros((h, w, 3), dtype=np.uint8)
        return b'\x00' * (h * w * 3)  # type: ignore[return-value]

    def capture_image(self, filepath: str, resolution: Tuple[int, int] | None = None,
                      fmt: str = "JPG") -> None:
        try:
            from PIL import Image
            res = resolution or self._resolution
            img = Image.new("RGB", res, (64, 64, 64))
            img.save(filepath)
        except ImportError:
            # Fallback: write an empty file when PIL is not available
            with open(filepath, "wb") as f:
                f.write(b"")

    def capture_array(self, resolution: Tuple[int, int] | None = None) -> ndarray:
        res = resolution or self._resolution
        if np is not None:
            return np.zeros((res[1], res[0], 3), dtype=np.uint8)
        return b'\x00' * (res[1] * res[0] * 3)  # type: ignore[return-value]

    def apply_settings(self, settings: CameraSettings) -> None:
        pass

    def get_metadata(self) -> Dict[str, Any]:
        return {"mock": True}

    def get_properties(self) -> Dict[str, Any]:
        return {"Model": "MockCamera", "PixelArraySize": self._resolution}
