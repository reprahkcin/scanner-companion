#!/usr/bin/env python3
"""
Batch-fix XMP sidecars for RealityScan/RealityCapture.

This tool:
- Normalizes XMP schema (writes <xcr:Rotation> and <xcr:Position> elements)
- Optionally recomputes camera poses along a horizontal circle
- Optionally renames XMPs to match a desired zero-padding (e.g., stack_000.xmp)
- Can align XMP names to actual image basenames when an images directory is provided

Usage examples:
  python fix_xmp_for_realityscan.py temp/small_fly_9 --recompute-poses --start-angle 0 --direction cw --pad 3
  python fix_xmp_for_realityscan.py path/to/xmps --images-dir path/to/images --pad 3

Notes:
- Positions are in meters. Radius is inferred from current XMPs if not provided.
- The output overwrites files in-place unless --dry-run is provided.
"""
from __future__ import annotations
import argparse
import glob
import math
import os
import re
import shutil
import statistics
import sys
import xml.etree.ElementTree as ET

XCR_NS = "http://www.capturingreality.com/ns/xcr/1.1#"
ET.register_namespace('x', 'adobe:ns:meta/')
ET.register_namespace('rdf', 'http://www.w3.org/1999/02/22-rdf-syntax-ns#')
ET.register_namespace('xcr', XCR_NS)

# Calibration-related attribute names (local names) we may want to copy from a template
CALIB_ATTR_LOCALNAMES = {
    'DistortionModel',
    'DistortionCoeficients',
    'FocalLength35mm',
    'Skew',
    'AspectRatio',
    'PrincipalPointU',
    'PrincipalPointV',
    'CalibrationPrior',
    'CalibrationGroup',
    'DistortionGroup',
    'InTexturing',
    'InMeshing',
}


def parse_position(text: str) -> tuple[float,float,float]:
    parts = str(text).strip().split()
    if len(parts) != 3:
        raise ValueError(f"Invalid xcr:Position contents: {text}")
    return tuple(float(p) for p in parts)  # type: ignore


def norm(v):
    return math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])


def ring_pose(radius_m: float, theta_deg: float, roll_deg: float = 0.0, cross_order: str = 'up_x_f') -> tuple[tuple[float,float,float], tuple[float,float,float], tuple[float,float,float], tuple[float,float,float]]:
    """Compute a horizontal ring camera pose basis vectors.

    Returns position, right, up, forward. The caller decides how to assemble the
    rotation matrix rows and the sign of the forward row.

    cross_order controls tangent construction:
      - 'f_x_up'  => right = normalize(cross(forward, worldUp)); up = cross(right, forward)
      - 'up_x_f'  => right = normalize(cross(worldUp, forward)); up = cross(forward, right)

    A roll (sensor rotation about forward) can be applied after building the
    initial right/up vectors.
    """
    th = math.radians(theta_deg)
    x = radius_m * math.cos(th)
    y = radius_m * math.sin(th)
    z = 0.0

    # forward FROM camera TO origin (inward)
    f = (-math.cos(th), -math.sin(th), 0.0)
    upW = (0.0, 0.0, 1.0)
    if cross_order == 'up_x_f':
        # right = cross(worldUp, forward)
        rgt = (
            upW[1]*f[2] - upW[2]*f[1],
            upW[2]*f[0] - upW[0]*f[2],
            upW[0]*f[1] - upW[1]*f[0],
        )
    else:
        # default: right = cross(forward, worldUp)
        rgt = (
            f[1]*upW[2] - f[2]*upW[1],
            f[2]*upW[0] - f[0]*upW[2],
            f[0]*upW[1] - f[1]*upW[0],
        )
    rgt_len = math.sqrt(rgt[0]*rgt[0] + rgt[1]*rgt[1] + rgt[2]*rgt[2]) or 1.0
    rgt = (rgt[0]/rgt_len, rgt[1]/rgt_len, rgt[2]/rgt_len)
    if cross_order == 'up_x_f':
        # up = cross(forward, right)
        up = (
            f[1]*rgt[2] - f[2]*rgt[1],
            f[2]*rgt[0] - f[0]*rgt[2],
            f[0]*rgt[1] - f[1]*rgt[0],
        )
    else:
        # up = cross(right, forward)
        up = (
            rgt[1]*f[2] - rgt[2]*f[1],
            rgt[2]*f[0] - rgt[0]*f[2],
            rgt[0]*f[1] - rgt[1]*f[0],
        )
    # Apply roll about forward axis if requested (right-handed)
    if abs(roll_deg) > 1e-9:
        psi = math.radians(roll_deg)
        c, s = math.cos(psi), math.sin(psi)
        rgt = (
            rgt[0]*c + up[0]*s,
            rgt[1]*c + up[1]*s,
            rgt[2]*c + up[2]*s,
        )
        up = (
            up[0]*c - rgt[0]*s,
            up[1]*c - rgt[1]*s,
            up[2]*c - rgt[2]*s,
        )
    return (x, y, z), rgt, up, f


def ensure_rotation_element(desc: ET.Element) -> None:
    # Some older writers put xcr:Rotation as an attribute; convert to a child element.
    rot_attr_key = f"{{{XCR_NS}}}Rotation"
    rot_val = desc.attrib.pop(rot_attr_key, None)
    if rot_val is not None:
        # Create/overwrite child element
        rot_el = desc.find(f".//{{{XCR_NS}}}Rotation")
        if rot_el is None:
            rot_el = ET.SubElement(desc, f"{{{XCR_NS}}}Rotation")
        rot_el.text = str(rot_val)


def write_pose(desc: ET.Element, position_m: tuple[float,float,float], R9: tuple[float,...]) -> None:
    rot_el = desc.find(f".//{{{XCR_NS}}}Rotation")
    if rot_el is None:
        rot_el = ET.SubElement(desc, f"{{{XCR_NS}}}Rotation")
    rot_el.text = " ".join(f"{v:.10f}" for v in R9)

    pos_el = desc.find(f".//{{{XCR_NS}}}Position")
    if pos_el is None:
        pos_el = ET.SubElement(desc, f"{{{XCR_NS}}}Position")
    pos_el.text = " ".join(f"{v:.6f}" for v in position_m)


def load_description(root: ET.Element) -> ET.Element:
    # <x:xmpmeta><rdf:RDF><rdf:Description ...> structure
    rdf = root.find('.//{http://www.w3.org/1999/02/22-rdf-syntax-ns#}RDF')
    if rdf is None:
        raise ValueError("Missing rdf:RDF element")
    desc = rdf.find('.//{http://www.w3.org/1999/02/22-rdf-syntax-ns#}Description')
    if desc is None:
        raise ValueError("Missing rdf:Description element")
    return desc


def get_xcr_localname(attr_key: str) -> str | None:
    """Return local name if the attribute key is in xcr namespace, else None."""
    if attr_key.startswith('{') and '}' in attr_key:
        ns, local = attr_key[1:].split('}', 1)
        if ns == XCR_NS:
            return local
    return None


def copy_calibration_attrs(template_desc: ET.Element, target_desc: ET.Element) -> None:
    """Copy a whitelist of calibration-related xcr:* attributes from template to target."""
    # Build a map of desired attrs from template
    tpl = {}
    for k, v in template_desc.attrib.items():
        local = get_xcr_localname(k)
        if local and local in CALIB_ATTR_LOCALNAMES:
            tpl[local] = v
    # Apply to target
    for local, v in tpl.items():
        target_desc.set(f"{{{XCR_NS}}}{local}", v)


def infer_radius(xmp_paths: list[str]) -> float:
    radii = []
    for xp in xmp_paths:
        try:
            tree = ET.parse(xp)
            desc = load_description(tree.getroot())
            pos_el = desc.find(f".//{{{XCR_NS}}}Position")
            if pos_el is not None and pos_el.text:
                p = parse_position(pos_el.text)
                radii.append(norm(p))
        except Exception:
            pass
    if not radii:
        return 0.5  # meters default fallback
    return statistics.median(radii)


def recompute_all(xmp_paths: list[str], start_angle: float, step_deg: float, direction: str, radius_m: float, dry_run: bool, roll_deg: float, cross_order: str, last_row: str) -> None:
    sign = -1.0 if direction.lower() == 'cw' else 1.0
    for idx, path in enumerate(sorted(xmp_paths)):
        angle = start_angle + sign * (idx * step_deg)
        pos, rgt, up, f = ring_pose(radius_m, angle, roll_deg, cross_order)
        tree = ET.parse(path)
        root = tree.getroot()
        desc = load_description(root)
        ensure_rotation_element(desc)
        # Set mandatory attributes
        desc.set(f"{{{XCR_NS}}}Version", "3")
        desc.set(f"{{{XCR_NS}}}Coordinates", "absolute")
        # Preserve existing priors if present
        if f"{{{XCR_NS}}}PosePrior" not in desc.attrib:
            desc.set(f"{{{XCR_NS}}}PosePrior", "locked")
        if f"{{{XCR_NS}}}CalibrationPrior" not in desc.attrib:
            desc.set(f"{{{XCR_NS}}}CalibrationPrior", "exact")
        # Winning variant (16) uses last row = forward (positive)
        if last_row == 'neg-forward':
            # Allow override for experimentation
            R9 = (rgt[0], rgt[1], rgt[2],
                  up[0],  up[1],  up[2],
                  -f[0],  -f[1],  -f[2])
        else:
            R9 = (rgt[0], rgt[1], rgt[2],
                  up[0],  up[1],  up[2],
                  f[0],   f[1],   f[2])
        write_pose(desc, pos, R9)
        if not dry_run:
            tree.write(path, encoding='utf-8', xml_declaration=False)
        print(f"fixed pose: {os.path.basename(path)} -> angle={angle:.3f} deg, radius={radius_m:.4f} m")


def rename_to_match_images(xmp_dir: str, images_dir: str, pad: int, dry_run: bool) -> None:
    # Map image indices by basename pattern containing a number
    images = sorted(glob.glob(os.path.join(images_dir, '*.*')))
    idx_from_name = lambda name: int(re.search(r'(\d+)', os.path.basename(name)).group(1)) if re.search(r'(\d+)', os.path.basename(name)) else None
    pairs = [(idx_from_name(p), p) for p in images]
    pairs = [(i,p) for (i,p) in pairs if i is not None]
    if not pairs:
        print("No numbered images found to align names with; skipping image-based rename.")
        return
    # Sort by numeric index
    pairs.sort(key=lambda t: t[0])

    xmp_files = sorted(glob.glob(os.path.join(xmp_dir, 'stack_*.xmp')))
    if len(xmp_files) != len(pairs):
        print(f"Warning: {len(xmp_files)} XMPs vs {len(pairs)} images; proceeding by index order.")

    for i, (img_idx, img_path) in enumerate(pairs):
        if i >= len(xmp_files):
            break
        src = xmp_files[i]
        stem = re.sub(r'\.[^.]+$', '', os.path.basename(img_path))
        new_name = f"{stem}.xmp"
        dst = os.path.join(xmp_dir, new_name)
        if not dry_run:
            shutil.move(src, dst)
        print(f"rename: {os.path.basename(src)} -> {new_name}")


def pad_filenames(xmp_dir: str, pad: int, dry_run: bool) -> None:
    for path in sorted(glob.glob(os.path.join(xmp_dir, 'stack_*.xmp'))):
        m = re.match(r'stack_(\d+)\.xmp$', os.path.basename(path))
        if not m:
            continue
        idx = int(m.group(1))
        new_name = f"stack_{idx:0{pad}d}.xmp"
        if os.path.basename(path) == new_name:
            continue
        dst = os.path.join(xmp_dir, new_name)
        if not dry_run:
            shutil.move(path, dst)
        print(f"rename: {os.path.basename(path)} -> {new_name}")


def main():
    ap = argparse.ArgumentParser(description="Fix/normalize XMP sidecars for RealityScan/RealityCapture")
    ap.add_argument('xmp_dir', help='Directory containing XMP files (e.g., stack_00.xmp ...)')
    ap.add_argument('--images-dir', help='Directory containing images to match basenames', default=None)
    ap.add_argument('--count', type=int, help='Number of cameras; defaults to number of XMPs')
    ap.add_argument('--start-angle', type=float, default=0.0, help='Angle of index 0 (degrees)')
    ap.add_argument('--direction', choices=['cw','ccw'], default='cw', help='Ordering direction (default cw)')
    ap.add_argument('--radius', type=float, default=None, help='Radius in meters (override)')
    ap.add_argument('--recompute-poses', action='store_true', help='Recompute position/rotation to follow index order')
    ap.add_argument('--pad', type=int, default=None, help='Zero-pad count for filenames, e.g. 3 -> stack_000.xmp')
    ap.add_argument('--roll', type=float, default=0.0, help='Camera roll in degrees about forward axis (e.g., 90 for portrait sensors)')
    ap.add_argument('--dry-run', action='store_true', help='Do not write changes, just print actions')
    # Defaults aligned to empirically validated variant 16
    ap.add_argument('--cross-order', choices=['f_x_up','up_x_f'], default='up_x_f', help='Tangent construction method (right vector definition); default up_x_f')
    ap.add_argument('--last-row', choices=['forward','neg-forward'], default='forward', help='Last rotation matrix row; default forward (positive)')
    ap.add_argument('--copy-calibration-from', dest='calib_template', default=None,
                    help='Path to a template XMP whose xcr:* calibration attributes should be copied to all files in xmp_dir')

    args = ap.parse_args()
    xmp_dir = os.path.abspath(args.xmp_dir)

    xmp_paths = sorted(glob.glob(os.path.join(xmp_dir, 'stack_*.xmp')))
    if not xmp_paths:
        print(f"No XMP files found in {xmp_dir}")
        return 1

    n = args.count or len(xmp_paths)
    step_deg = 360.0 / n

    radius_m = args.radius if args.radius is not None else infer_radius(xmp_paths)
    print(f"Using radius = {radius_m:.6f} m, step = {step_deg:.6f} deg, start = {args.start_angle:.3f} deg, dir = {args.direction}")

    # First normalize schema to ensure Rotation element exists; also recompute if requested
    if args.recompute_poses:
        recompute_all(xmp_paths, args.start_angle, step_deg, args.direction, radius_m, args.dry_run, args.roll, args.cross_order, args.last_row)
    else:
        # Even without recompute, ensure Rotation element exists
        for path in xmp_paths:
            try:
                tree = ET.parse(path)
                desc = load_description(tree.getroot())
                ensure_rotation_element(desc)
                if not args.dry_run:
                    tree.write(path, encoding='utf-8', xml_declaration=False)
                print(f"normalized schema: {os.path.basename(path)}")
            except Exception as e:
                print(f"skip {os.path.basename(path)}: {e}")

    # Optionally copy calibration attributes from a template XMP
    if args.calib_template:
        try:
            tpl_tree = ET.parse(os.path.abspath(args.calib_template))
            tpl_desc = load_description(tpl_tree.getroot())
        except Exception as e:
            print(f"Failed to load calibration template: {e}")
            return 2
        for path in xmp_paths:
            try:
                tree = ET.parse(path)
                root = tree.getroot()
                desc = load_description(root)
                copy_calibration_attrs(tpl_desc, desc)
                if not args.dry_run:
                    tree.write(path, encoding='utf-8', xml_declaration=False)
                print(f"copied calibration: {os.path.basename(path)}")
            except Exception as e:
                print(f"skip calibration copy for {os.path.basename(path)}: {e}")

    # Renaming
    if args.images_dir:
        rename_to_match_images(xmp_dir, os.path.abspath(args.images_dir), args.pad or 3, args.dry_run)
    elif args.pad:
        pad_filenames(xmp_dir, args.pad, args.dry_run)

    print("Done.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
