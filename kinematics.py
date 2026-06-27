"""
Kinematics for the MKII Sherline 6-axis photogrammetry scanner.

Machine (see profiles/sherline_6axis.json and docs/future/SHERLINE_6AXIS_DESIGN.md):

    SPECIMEN branch:  bed X, bed Y  ->  A rotary (about X)  ->  specimen
    CAMERA  branch:   column Z      ->  B tilt (about Y)    ->  R focus rail -> camera

Conventions (world frame: Z up, right-handed, origin = specimen/chuck centre):

    * Specimen centre sits on the A rotation axis, so A does NOT translate it.
      The bed moves the specimen:  S = [X - Xhome, Y - Yhome, 0].
    * B = 0  ->  camera optical axis points straight DOWN (-Z).
      Positive B tilts the optical axis about +Y, toward +X, reaching
      horizontal (+X) at B = 90.  So the world optical axis is
          d(B) = [ sin B, 0, -cos B ].
    * The camera+rail ride on the B rotary, whose pivot rides on the Z column.
    * R slides the camera along the optical axis (focus); +R moves toward the
      specimen.  Its direction is rigidly tied to B.

This module is intentionally dependency-free (stdlib ``math`` only) so it can
run anywhere, including headless. Visualisation/export layers may use numpy etc.

The quantitative mounting geometry (pivot location, pivot->camera offset, focus
scale) lives in the profile's ``ik_geometry`` block and is loaded into
:class:`IKGeometry`.  Until measured it falls back to documented placeholder
defaults — IK output is structurally correct but not metrically trustworthy
until calibrated (see :func:`IKGeometry.is_calibrated`).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from axis_config import MachineProfile, ProfileManager


# ──────────────────────────────────────────────────────────────────────────────
# Tiny 3-vector / 3x3-matrix helpers (pure Python, no numpy)
# ──────────────────────────────────────────────────────────────────────────────

Vec3 = Tuple[float, float, float]
Mat3 = Tuple[Vec3, Vec3, Vec3]


def v_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def v_dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_norm(a: Vec3) -> float:
    return math.sqrt(v_dot(a, a))


def v_unit(a: Vec3) -> Vec3:
    n = v_norm(a)
    if n == 0.0:
        return (0.0, 0.0, 0.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def m_vec(m: Mat3, v: Vec3) -> Vec3:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def rot_x(deg: float) -> Mat3:
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def rot_y(deg: float) -> Mat3:
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


# ──────────────────────────────────────────────────────────────────────────────
# Mounting geometry (the part that must be calibrated)
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class IKGeometry:
    """Quantitative mounting geometry consumed by the inverse kinematics.

    All lengths in mm.  ``b_pivot_to_camera_at_R0`` and ``rail_axis`` are
    expressed in the B-frame (the tilt rotary's local frame), which coincides
    with the world frame at B = 0.
    """

    # Pivot of the B tilt axis, in world coords, when Z is at its home position.
    b_pivot_at_home: Vec3 = (0.0, 0.0, 150.0)
    # Vector from the B pivot to the camera entrance pupil at R = 0, in B-frame.
    b_pivot_to_camera_at_R0: Vec3 = (0.0, 0.0, -30.0)
    # Unit direction the camera travels along the rail as R increases, in B-frame.
    rail_axis: Vec3 = (0.0, 0.0, -1.0)
    # Working-distance change per mm of R travel (1.0 if R moves straight along
    # the optical axis).
    focus_distance_per_R: float = 1.0
    # True once real measured values have replaced the placeholder defaults.
    calibrated: bool = False

    def is_calibrated(self) -> bool:
        return self.calibrated

    @classmethod
    def from_profile_dict(cls, raw: dict) -> "IKGeometry":
        """Build from a profile JSON's ``ik_geometry`` block.

        Missing or null fields fall back to the placeholder defaults; the
        ``calibrated`` flag is set only when the required fields are present
        and non-null.
        """
        g = raw.get("ik_geometry", {}) or {}
        defaults = cls()

        def vec(key: str, default: Vec3) -> Tuple[Vec3, bool]:
            val = g.get(key)
            if not val or any(c is None for c in val):
                return default, False
            return (float(val[0]), float(val[1]), float(val[2])), True

        pivot, ok_pivot = vec("b_pivot_at_home", defaults.b_pivot_at_home)
        offset, ok_offset = vec(
            "b_pivot_to_camera_at_R0_in_Bframe", defaults.b_pivot_to_camera_at_R0
        )
        rail, _ = vec("rail_axis_in_Bframe", defaults.rail_axis)
        fpr = g.get("focus_distance_per_R_mm")
        ok_fpr = fpr is not None
        fpr = float(fpr) if ok_fpr else defaults.focus_distance_per_R

        status = str(g.get("status", "")).upper()
        calibrated = (
            ok_pivot and ok_offset and ok_fpr and status != "NEEDS_CALIBRATION"
        )
        return cls(
            b_pivot_at_home=pivot,
            b_pivot_to_camera_at_R0=offset,
            rail_axis=v_unit(rail),
            focus_distance_per_R=fpr,
            calibrated=calibrated,
        )


# ──────────────────────────────────────────────────────────────────────────────
# Results
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class CameraPose:
    """Result of forward kinematics — where the camera is and what it sees."""

    camera_world: Vec3            # entrance pupil position, world frame
    optical_axis_world: Vec3      # unit look direction, world frame
    working_distance: float       # specimen distance along the optical axis (mm)
    framing_error: float          # lateral distance of specimen off the axis (mm)
    # Camera pose expressed in the (rotating) specimen frame — what photogrammetry
    # needs.  Position is the pupil relative to the specimen; directions are unit.
    camera_in_specimen: Vec3
    forward_in_specimen: Vec3     # look direction (specimen -> away), specimen frame
    up_in_specimen: Vec3


@dataclass
class AxisSolution:
    """Result of inverse kinematics."""

    axes: Dict[str, float]        # X, Y, Z, R, A, B
    feasible: bool                # all axes within limits?
    violations: List[str] = field(default_factory=list)

    def as_tuple(self) -> Tuple[float, ...]:
        return tuple(self.axes[k] for k in ("X", "Y", "Z", "R", "A", "B"))


# ──────────────────────────────────────────────────────────────────────────────
# Kinematics
# ──────────────────────────────────────────────────────────────────────────────

class SherlineKinematics:
    """Forward/inverse kinematics for the Sherline 6-axis scanner."""

    def __init__(self, profile: MachineProfile, geometry: IKGeometry):
        self.profile = profile
        self.geom = geometry
        self.Xh = profile.get_axis("X").home_position
        self.Yh = profile.get_axis("Y").home_position
        self.Zh = profile.get_axis("Z").home_position

    # -- constructors ----------------------------------------------------------

    @classmethod
    def from_profile_name(cls, name: str = "sherline_6axis",
                          profiles_dir: Optional[str] = None) -> "SherlineKinematics":
        pm = ProfileManager(profiles_dir)
        profile = pm.load_profile(name)
        # Read the raw JSON for the ik_geometry block (the dataclass loader
        # ignores extra keys); fall back to defaults if there is no file.
        raw: dict = {}
        fp = pm.profiles_dir / f"{name}.json"
        if fp.exists():
            raw = json.loads(fp.read_text())
        return cls(profile, IKGeometry.from_profile_dict(raw))

    # -- geometry primitives ---------------------------------------------------

    def optical_axis(self, B: float) -> Vec3:
        """World optical-axis direction for camera tilt B (deg)."""
        r = math.radians(B)
        return (math.sin(r), 0.0, -math.cos(r))

    def _b_frame(self, B: float) -> Mat3:
        """Rotation taking B-frame vectors to world.  R_y(-B) so that the
        camera's local -Z maps to [sin B, 0, -cos B]."""
        return rot_y(-B)

    def _pupil_offset_world(self, B: float, R: float) -> Vec3:
        """Vector from the B pivot to the camera pupil, in world coords."""
        rail = v_scale(self.geom.rail_axis, R)
        pupil_in_b = v_add(self.geom.b_pivot_to_camera_at_R0, rail)
        return m_vec(self._b_frame(B), pupil_in_b)

    def _b_pivot_world(self, Z: float) -> Vec3:
        """B-axis pivot in world coords for column position Z."""
        px, py, pz = self.geom.b_pivot_at_home
        return (px, py, pz + (Z - self.Zh))

    def specimen_center(self, X: float, Y: float) -> Vec3:
        return (X - self.Xh, Y - self.Yh, 0.0)

    def camera_world(self, Z: float, B: float, R: float) -> Vec3:
        return v_add(self._b_pivot_world(Z), self._pupil_offset_world(B, R))

    # -- forward ---------------------------------------------------------------

    def forward(self, axes: Dict[str, float]) -> CameraPose:
        X, Y, Z = axes["X"], axes["Y"], axes["Z"]
        R, A, B = axes["R"], axes["A"], axes["B"]

        S = self.specimen_center(X, Y)
        C = self.camera_world(Z, B, R)
        d = self.optical_axis(B)

        to_spec = v_sub(S, C)
        working_distance = v_dot(to_spec, d)
        along = v_scale(d, working_distance)
        framing_error = v_norm(v_sub(to_spec, along))

        # Express in the specimen frame (specimen rolls about X by A).
        inv_A = rot_x(-A)
        camera_in_specimen = m_vec(inv_A, v_sub(C, S))
        # Camera local basis in world: forward = d, up = +Y at B=0 (Y is the
        # tilt axis, invariant under B), so up_world is M @ [0,1,0] = [0,1,0].
        up_world = m_vec(self._b_frame(B), (0.0, 1.0, 0.0))
        forward_in_specimen = v_unit(m_vec(inv_A, d))
        up_in_specimen = v_unit(m_vec(inv_A, up_world))

        return CameraPose(
            camera_world=C,
            optical_axis_world=d,
            working_distance=working_distance,
            framing_error=framing_error,
            camera_in_specimen=camera_in_specimen,
            forward_in_specimen=forward_in_specimen,
            up_in_specimen=up_in_specimen,
        )

    # -- inverse ---------------------------------------------------------------

    def inverse(self, B: float, working_distance: float, A: float = 0.0,
                R: Optional[float] = None) -> AxisSolution:
        """Solve bed/column axes so the camera frames the specimen centre.

        Given camera tilt ``B`` (deg), the target ``working_distance`` (mm) and
        specimen orientation ``A`` (deg), returns X/Y/Z plus the passed-through
        R/A/B.  ``R`` defaults to the rail home and is what a focus stack later
        sweeps — X/Y/Z are solved for that reference R.
        """
        if R is None:
            R = self.profile.get_axis("R").home_position

        d = self.optical_axis(B)                      # world optical axis
        w = self._pupil_offset_world(B, R)            # pivot -> pupil, world
        px, py, pz = self.geom.b_pivot_at_home

        # C = S - WD*d, with S = [X-Xh, Y-Yh, 0] and C = pivot(Z) + w.
        #   x:  px + w_x       = (X - Xh) - WD*d_x
        #   y:  py + w_y       = (Y - Yh) - WD*d_y
        #   z:  pz + (Z-Zh) + w_z = -WD*d_z
        X = self.Xh + px + w[0] + working_distance * d[0]
        Y = self.Yh + py + w[1] + working_distance * d[1]
        Z = self.Zh - pz - w[2] - working_distance * d[2]

        axes = {"X": X, "Y": Y, "Z": Z, "R": R, "A": A % 360.0, "B": B}
        feasible, violations = self.validate(axes)
        return AxisSolution(axes=axes, feasible=feasible, violations=violations)

    # -- validation ------------------------------------------------------------

    def validate(self, axes: Dict[str, float]) -> Tuple[bool, List[str]]:
        violations: List[str] = []
        for name, value in axes.items():
            ax = self.profile.get_axis(name)
            if ax is None:
                continue
            if value < ax.min_limit - 1e-6 or value > ax.max_limit + 1e-6:
                violations.append(
                    f"{name}={value:.3f} outside [{ax.min_limit}, {ax.max_limit}]"
                )
        return (len(violations) == 0, violations)


# ──────────────────────────────────────────────────────────────────────────────
# Calibration helper
# ──────────────────────────────────────────────────────────────────────────────

def write_ik_geometry(profile_path: str, *, b_pivot_at_home: Vec3,
                      b_pivot_to_camera_at_R0_in_Bframe: Vec3,
                      rail_axis_in_Bframe: Vec3 = (0.0, 0.0, -1.0),
                      focus_distance_per_R_mm: float = 1.0) -> None:
    """Write measured geometry into a profile JSON's ``ik_geometry`` block and
    flip its status to CALIBRATED.  Preserves all other profile content."""
    path = Path(profile_path)
    data = json.loads(path.read_text())
    data["ik_geometry"] = {
        "status": "CALIBRATED",
        "specimen_position": [0, 0, 0],
        "b_pivot_at_home": list(b_pivot_at_home),
        "b_pivot_to_camera_at_R0_in_Bframe": list(b_pivot_to_camera_at_R0_in_Bframe),
        "rail_axis_in_Bframe": list(rail_axis_in_Bframe),
        "focus_distance_per_R_mm": focus_distance_per_R_mm,
    }
    path.write_text(json.dumps(data, indent=2) + "\n")


# ──────────────────────────────────────────────────────────────────────────────
# Self-test: IK -> FK round trip with placeholder geometry
# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    kin = SherlineKinematics.from_profile_name("sherline_6axis")
    print(f"Profile: {kin.profile.name} v{kin.profile.version}")
    print(f"Geometry calibrated: {kin.geom.is_calibrated()} "
          f"(placeholder defaults if False)\n")

    WD = 80.0  # target working distance, mm
    print(f"IK -> FK round trip @ working distance {WD} mm")
    print(f"{'B':>4} {'A':>5} | {'X':>8} {'Y':>8} {'Z':>8} {'R':>6} | "
          f"{'WD_fk':>7} {'frame_err':>9} {'feasible':>8}")
    max_err = 0.0
    for B in (0.0, 30.0, 60.0, 90.0):
        for A in (0.0, 90.0, 180.0):
            sol = kin.inverse(B=B, working_distance=WD, A=A)
            pose = kin.forward(sol.axes)
            max_err = max(max_err, pose.framing_error, abs(pose.working_distance - WD))
            a = sol.axes
            print(f"{B:>4.0f} {A:>5.0f} | {a['X']:>8.2f} {a['Y']:>8.2f} "
                  f"{a['Z']:>8.2f} {a['R']:>6.1f} | {pose.working_distance:>7.2f} "
                  f"{pose.framing_error:>9.2e} {str(sol.feasible):>8}")
    print(f"\nMax round-trip error: {max_err:.2e} mm "
          f"({'OK' if max_err < 1e-6 else 'CHECK'})")
