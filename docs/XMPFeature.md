Yes. You can write XMP sidecars per image and RealityScan/RealityCapture will consume them on import. Use `xcr:Position` and `xcr:Rotation` plus priors. Filenames are ignored for pose. ([RealityScan Help][1])

# Coding-agent brief: add XMP pose on capture

## What to implement

1. **Two user inputs in Calibration tab**

   * `lensToObject_mm` (distance from lens entrance pupil to object center).
   * `railToHorizon_deg` (positive = camera pitched up from horizontal; negative = down).
     Persist in the existing calibration JSON.

2. **Pose math helpers**

```python
import math
from dataclasses import dataclass

@dataclass
class RigPose:
    pos_m: tuple[float,float,float]     # (x,y,z) in meters
    R_rowmajor: tuple[float,...]        # 9 numbers, row-major 3x3

def ring_pose(distance_mm: float, rail_deg: float, theta_deg: float) -> RigPose:
    # Object center at origin. X right, Y forward, Z up in *your* rig space.
    # Project the camera center on a tilted ring defined by rail_deg.
    r = distance_mm / 1000.0
    th = math.radians(theta_deg)
    a  = math.radians(rail_deg)
    # Camera center in world coords
    x = r * math.cos(th) * math.cos(a)
    y = r * math.sin(th) * math.cos(a)
    z = r * math.sin(a)

    # Build a "look-at origin" rotation, then convert to RealityCapture XMP matrix.
    # World basis:
    C = (x, y, z)
    tgt = (0.0, 0.0, 0.0)
    upW = (0.0, 0.0, 1.0)

    def vsub(a,b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
    def vdot(a,b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
    def vcross(a,b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
    def vnorm(a):
        m = math.sqrt(vdot(a,a)); return (a[0]/m, a[1]/m, a[2]/m)

    # Camera axes: forward points from camera to target
    f = vnorm(vsub(tgt, C))
    rgt = vnorm(vcross(f, upW))
    up  = vcross(rgt, f)

    # RealityCapture expects a 3x3 rotation matrix serialized into 9 numbers.
    # Use row-major of the *camera-to-world* axes, then invert (transpose) to get world->camera.
    Rc2w = ((rgt[0], rgt[1], rgt[2]),
            (up[0],  up[1],  up[2]),
            (-f[0],  -f[1],  -f[2]))  # right, up, -forward
    Rw2c = ((Rc2w[0][0], Rc2w[1][0], Rc2w[2][0]),
            (Rc2w[0][1], Rc2w[1][1], Rc2w[2][1]),
            (Rc2w[0][2], Rc2w[1][2], Rc2w[2][2]))

    R9 = (Rw2c[0][0], Rw2c[0][1], Rw2c[0][2],
          Rw2c[1][0], Rw2c[1][1], Rw2c[1][2],
          Rw2c[2][0], Rw2c[2][1], Rw2c[2][2])
    return RigPose((x,y,z), R9)
```

RealityCapture’s XMP needs `xcr:Position` (meters) and `xcr:Rotation` (nine numbers). This matches their sample XMP and docs. ([RealityScan Help][1])

3. **Per-shot rail offset and stack averaging**

* Use your saved per-angle “near”/“far” rail positions from the Calibration tab.
* For shot `k` in a stack of `n`, linear interpolate the *along-rail* offset and add to `lensToObject_mm`.
* For the **fused** image, compute the mean of all shots’ distances and write one pose. Keep the same angle `theta_deg` for the whole stack.

4. **Write an XMP sidecar next to every saved image**
   Generate `basename.jpg` and `basename.xmp`. RealityCapture will auto-pair by filename in the same folder. ([RealityScan Help][1])

```python
from xml.sax.saxutils import escape

XCR_NS = 'http://www.capturingreality.com/ns/xcr/1.1#'
SC_NS  = 'https://reprahkcin.github.io/scanner-companion/1.0#'  # custom

def write_xmp_sidecar(img_path: str, pose: RigPose,
                      lensToObject_mm: float, railToHorizon_deg: float,
                      theta_deg: float, stack_index: int, shot_index: int):
    x, y, z = pose.pos_m
    R = pose.R_rowmajor
    xmp = f'''<x:xmpmeta xmlns:x="adobe:ns:meta/">
 <rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#">
  <rdf:Description xmlns:xcr="{XCR_NS}" xmlns:sc="{SC_NS}"
    xcr:Version="3"
    xcr:PosePrior="initial"
    xcr:Coordinates="absolute"
    xcr:CalibrationPrior="initial">
    <xcr:Position>{x:.6f} {y:.6f} {z:.6f}</xcr:Position>
    <xcr:Rotation>{R[0]:.9f} {R[1]:.9f} {R[2]:.9f} {R[3]:.9f} {R[4]:.9f} {R[5]:.9f} {R[6]:.9f} {R[7]:.9f} {R[8]:.9f}</xcr:Rotation>
    <sc:LensToObject_mm>{lensToObject_mm:.3f}</sc:LensToObject_mm>
    <sc:RailToHorizon_deg>{railToHorizon_deg:.6f}</sc:RailToHorizon_deg>
    <sc:Theta_deg>{theta_deg:.6f}</sc:Theta_deg>
    <sc:StackIndex>{stack_index}</sc:StackIndex>
    <sc:ShotIndex>{shot_index}</sc:ShotIndex>
  </rdf:Description>
 </rdf:RDF>
</x:xmpmeta>'''
    xmp_path = img_path.rsplit('.',1)[0] + '.xmp'
    with open(xmp_path, 'w', encoding='utf-8') as f:
        f.write(xmp)
```

RealityCapture’s help confirms sidecar pairing, the `xcr:` namespace, and the presence of `xcr:Position` and `xcr:Rotation`. You control prior hardness in Alignment to respect these priors. ([RealityScan Help][1])

5. **Hook into `scanner_control.py` capture path**

* After each `picamera2.capture_file(...)`, compute `theta_deg` for the current angle, compute `distance_mm` for the current shot via near/far interpolation, call `ring_pose(...)`, then `write_xmp_sidecar(...)`.
* Do the same once for the fused file after Helicon processes a stack folder, using the **mean** distance over that stack.
* Keep writing your `metadata.json` as today; add these fields so downstream tools can re-emit XMP if needed.

6. **RealityCapture settings on import**

* Put images and `.xmp` in the same folder with matching basenames. RC will assign the XMP metadata automatically. ([RealityScan Help][1])
* In **Alignment → Selected Inputs**, set **Use camera priors** and increase **Position prior hardness** and **Orientation prior hardness** to bias toward your ring. ([RealityScan Help][2])

## Notes and edge cases

* Units: RC priors are in the project’s coordinate units. Use meters in XMP; keep your UI in mm and convert. ([RealityScan Help][2])
* Matrix convention: `xcr:Rotation` is a 3×3 serialized into nine numbers. RC’s XMP sample shows this and it works with row-major order. If axes look flipped after a test import, transpose the matrix (swap row/column emission) and re-import; RC’s coordinate docs and samples vary by export settings. Validate with a 4-camera ring smoke test. ([RealityScan Help][1])
* Optional: also write a `_common.xmp` if you want shared fields across all images in a folder. ([RealityScan Help][1])
* Library choice: sidecars avoid EXIF write hassles. If you later want to embed XMP into JPEGs, use `python-xmp-toolkit`. ([python-xmp-toolkit.readthedocs.io][3])

## Minimal test plan

1. Capture a tiny session: 4 angles × 3 shots. Set `lensToObject_mm=250`, `railToHorizon_deg=0`.
2. Verify each `*.jpg` has a sibling `*.xmp` with nonzero `xcr:Position` and orthonormal `xcr:Rotation`.
3. Import in RC. Enable camera priors. Check the ring is evenly spaced and level.
4. Tilt test: set `railToHorizon_deg=-15`. Re-import. Cameras should form an inclined ring.
5. Fuse stack, copy the averaged XMP to the fused file, and confirm RC honors the fused pose.

If you want, I’ll map these hooks to your exact `scanner_control.py` functions and field names next.

[1]: https://rshelp.capturingreality.com/en-US/tools/xmpalign.htm "Metadata (XMP) files - RealityScan Help"
[2]: https://rshelp.capturingreality.com/en-US/appbasics/alignsettings.htm?utm_source=chatgpt.com "Alignment Settings - RealityScan Help - Capturing Reality"
[3]: https://python-xmp-toolkit.readthedocs.io/?utm_source=chatgpt.com "Welcome — Python XMP Toolkit 2.0.1 documentation"
