# RealityScan / RealityCapture XMP Pose Reference

This file summarizes how to define camera poses for Unreal RealityScan (and RealityCapture) using `.xmp` sidecars so that you can fully control camera position and orientation.

---

## 1. Overview

- Each image can have an XMP sidecar:
  - `image001.jpg` + `image001.xmp` in the same folder.
- On import, RealityScan reads pose and calibration from the XMP instead of/in addition to EXIF.
- Camera pose is defined by:
  - `xcr:Rotation` (3×3 matrix, 9 floats, row-major)
  - `xcr:Position` (3 floats)
  - `xcr:PosePrior` and `xcr:Coordinates` (behavior and coordinate system)

There is no dedicated “look-at” tag. Look-at and up are encoded into `xcr:Rotation`.

---

## 2. Minimal XMP Template

Create one file per image with the same basename.

```xml
<x:xmpmeta xmlns:x="adobe:ns:meta/">
  <rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#">
    <rdf:Description
      xmlns:xcr="http://www.capturingreality.com/ns/xcr/1.1#"
      xcr:Version="3"
      xcr:PosePrior="locked"
      xcr:Coordinates="absolute"
      xcr:DistortionModel="perspective"
      xcr:FocalLength35mm="35"
      xcr:Skew="0"
      xcr:AspectRatio="1"
      xcr:PrincipalPointU="0"
      xcr:PrincipalPointV="0"
      xcr:CalibrationPrior="exact"
      xcr:CalibrationGroup="-1"
      xcr:DistortionGroup="-1"
      xcr:InTexturing="1"
      xcr:InMeshing="1">

      <!-- Row-major 3x3 rotation matrix -->
      <xcr:Rotation>
        R11 R12 R13
        R21 R22 R23
        R31 R32 R33
      </xcr:Rotation>

      <!-- Camera center in XMP/world coordinates -->
      <xcr:Position>PX PY PZ</xcr:Position>

    </rdf:Description>
  </rdf:RDF>
</x:xmpmeta>
```

Replace `Rij` and `PX PY PZ` with your values.

Key behavior notes:

- `xcr:PosePrior="locked"`:
  - Treats poses as fixed constraints. Use this for fully scripted cameras.
- `xcr:Coordinates="absolute"`:
  - Positions are interpreted directly in the XMP coordinate system (see next section).

---

## 3. Coordinate System and Up-Axis

RealityScan/RealityCapture use an internal **XMP coordinate system**.

Important points:

- It is right-handed.
- Z is up in the default local/XMP system used in docs and examples.
- `xcr:Position` is in this XMP system.
- `xcr:Rotation` and `xcr:Position` always refer to this same system.
- You do not declare an “up-axis” in XMP; you enforce it through:
  - How you choose your coordinate system.
  - How you build the rotation matrix.

If you are generating poses from another DCC (Blender, Unreal, etc.), you must convert from that tool’s convention into the XMP coordinate system before writing `Position` and `Rotation`.

---

## 4. Semantics of `xcr:Rotation` and `xcr:Position`

Let:

- `R` = `xcr:Rotation` as a 3×3 matrix.
- `C` = `xcr:Position` as a 3D vector.

Then:

- `C` is the camera center in XMP/world coordinates.
- For a world point `Xw`, camera coordinates `Xc` are:

  `Xc = R * (Xw - C)`

So:

- `R` maps from world space to camera space.
- `xcr:Position` is not `t` from `[R|t]`; instead:
  - `t = -R * C`
  - and the projection matrix is `P = K [R | t]`.

Practically:

- You choose `C` (where the camera is in your world).
- You choose camera axes (look direction and up).
- You construct `R` so that applying `Xc = R (Xw - C)` matches that orientation.
- You write `R` row-major into `<xcr:Rotation>` and `C` into `<xcr:Position>`.

---

## 5. Building `xcr:Rotation` From Look-At + Up

Given:

- Camera position: `C = (cx, cy, cz)`
- Look-at point: `L = (lx, ly, lz)`
- World up vector in XMP system: `U = (ux, uy, uz)` (for Z-up: `U = (0,0,1)`)

Step 1: Forward (camera viewing direction)

`f = normalize(L - C)`

Step 2: Right vector

`s = normalize(cross(f, U))`

Step 3: True up

`u = s x f`

RealityCapture’s convention is that `R` transforms world to camera. A consistent choice is:

- Define camera axes in world coordinates:
  - Camera x-axis: `s`
  - Camera y-axis: `u`
  - Camera z-axis: `-f` (looking along -Z in camera space)

Then set `R` rows as:

Row 1: `s.x  s.y  s.z`  
Row 2: `u.x  u.y  u.z`  
Row 3: `-f.x -f.y -f.z`

And:

```xml
<xcr:Rotation>
  sx sy sz
  ux uy uz
  -fx -fy -fz
</xcr:Rotation>
<xcr:Position>cx cy cz</xcr:Position>
```

This guarantees:

- `Xc = R * (Xw - C)` points the camera from `C` toward `L` with `U` as the up reference.

You can change which axis is “forward” in camera space if you prefer, as long as you stay self-consistent with the above world-to-camera rule.

---

## 6. Practical Usage Notes

1. **Per-image control**  
   Generate one `.xmp` per image with your own `Rotation` and `Position`. RealityScan will ingest them on import.

2. **Locking poses**  
   Use `xcr:PosePrior="locked"` when you want RealityScan to keep your scripted poses unchanged.

3. **Calibration**  
   - If intrinsics are known, fill the calibration fields.
   - If not critical, you can still provide approximate values and let RealityScan refine if `PosePrior` is not locked.

4. **Consistency**  
   All cameras and any downstream geometry must share:
   - The same XMP/world coordinate system.
   - The same axis/up conventions when you build `R`.

---

## 7. Skeleton Generator (Pseudocode)

Use this structure in your own tooling:

```pseudo
for each image:
    C = camera_position_world
    L = look_at_world
    U = world_up  # e.g. (0, 0, 1)

    f = normalize(L - C)
    s = normalize(cross(f, U))
    u = cross(s, f)

    R = [
      [s.x, s.y, s.z],
      [u.x, u.y, u.z],
      [-f.x, -f.y, -f.z]
    ]

    write_xmp(
      rotation = R row-major,
      position = C,
      posePrior = "locked",
      coordinates = "absolute",
      intrinsics = known_or_guess
    )
```
