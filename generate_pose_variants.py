#!/usr/bin/env python3
"""Generate multiple XMP pose variant sets for RealityCapture troubleshooting.

This script brute‑forces several rotation matrix conventions so you can quickly
import each variant directory and see which one RealityCapture interprets as:
  - Cameras placed on a horizontal circle
  - Ordered (index increasing CCW around +Z)
  - Frusta pointing inward to the common center

Variants generated (8 total by default):
  cross order:  f_x_up  (right = cross(forward, upW))
                up_x_f  (right = cross(upW, forward))
  forward sign: f  | -f  (last basis uses forward or its negative)
  layout mode:  rows | cols (whether basis vectors are written as matrix rows or columns)

Directory naming pattern: <index>_<cross>__sign<sgn>__mode<mode>
Example: 00_up_x_f__sign-negf__mode-rows

Usage:
  python generate_pose_variants.py --out-dir temp/pose_variants --count 64 --radius 4.9 --start-angle 0 --direction ccw

After running, copy ONE variant directory's XMPs next to the images and import.

You can prune / extend patterns easily inside PATTERNS list.
"""
from __future__ import annotations
import math, os, argparse, textwrap
from xml.sax.saxutils import escape

XCR_NS = 'http://www.capturingreality.com/ns/xcr/1.1#'

PATTERNS = []  # filled dynamically below

CROSS_ORDERS = ['f_x_up','up_x_f']
FORWARD_SIGNS = ['f','-f']
MODES = ['rows','cols']
for c in CROSS_ORDERS:
    for s in FORWARD_SIGNS:
        for m in MODES:
            PATTERNS.append((c,s,m))


def basis(angle_deg: float, radius: float, cross_order: str):
    th = math.radians(angle_deg)
    # position on circle (Z=0)
    x = radius * math.cos(th)
    y = radius * math.sin(th)
    z = 0.0
    # inward forward vector (camera->origin)
    f = (-math.cos(th), -math.sin(th), 0.0)
    upW = (0.0, 0.0, 1.0)
    # cross order variants
    if cross_order == 'up_x_f':
        # right = upW x f
        r = (
            upW[1]*f[2] - upW[2]*f[1],
            upW[2]*f[0] - upW[0]*f[2],
            upW[0]*f[1] - upW[1]*f[0],
        )
    else:
        # default right = f x upW
        r = (
            f[1]*upW[2] - f[2]*upW[1],
            f[2]*upW[0] - f[0]*upW[2],
            f[0]*upW[1] - f[1]*upW[0],
        )
    # normalize right
    rl = math.sqrt(r[0]*r[0]+r[1]*r[1]+r[2]*r[2]) or 1.0
    r = (r[0]/rl, r[1]/rl, r[2]/rl)
    # construct up depending on cross order to keep right-handed, orthonormal frame
    if cross_order == 'up_x_f':
        # up = f x r
        u = (
            f[1]*r[2] - f[2]*r[1],
            f[2]*r[0] - f[0]*r[2],
            f[0]*r[1] - f[1]*r[0],
        )
    else:
        # up = r x f
        u = (
            r[1]*f[2] - r[2]*f[1],
            r[2]*f[0] - r[0]*f[2],
            r[0]*f[1] - r[1]*f[0],
        )
    return (x,y,z), r, u, f


def assemble(r,u,f, mode: str, forward_sign: str):
    f_use = f if forward_sign == 'f' else (-f[0], -f[1], -f[2])
    if mode == 'rows':
        rows = [r,u,f_use]
        R9 = rows[0] + rows[1] + rows[2]
    else:
        # cols: columns are the basis vectors, so rows are the transposed arrangement
        rows = [ (r[0], u[0], f_use[0]), (r[1], u[1], f_use[1]), (r[2], u[2], f_use[2]) ]
        R9 = rows[0] + rows[1] + rows[2]
    return R9


def write_xmp(path: str, pos, R9):
    rotation_str = ' '.join(f'{v:.10f}' for v in R9)
    position_str = ' '.join(f'{v:.6f}' for v in pos)
    xmp = f"""<x:xmpmeta xmlns:x='adobe:ns:meta/'>\n  <rdf:RDF xmlns:rdf='http://www.w3.org/1999/02/22-rdf-syntax-ns#'>\n    <rdf:Description xmlns:xcr='{XCR_NS}'\n        xcr:Version='3'\n        xcr:PosePrior='locked'\n        xcr:CalibrationPrior='exact'\n        xcr:Coordinates='absolute'\n        xcr:DistortionModel='brown3'\n        xcr:DistortionCoeficients='0 0 0 0 0 0'\n        xcr:FocalLength35mm='50'\n        xcr:Skew='0'\n        xcr:AspectRatio='1'\n        xcr:PrincipalPointU='0'\n        xcr:PrincipalPointV='0'\n        xcr:CalibrationGroup='-1'\n        xcr:DistortionGroup='-1'\n        xcr:InTexturing='1'\n        xcr:InMeshing='1'>\n      <xcr:Rotation>{rotation_str}</xcr:Rotation>\n      <xcr:Position>{position_str}</xcr:Position>\n    </rdf:Description>\n  </rdf:RDF>\n</x:xmpmeta>\n"""
    with open(path,'w',encoding='utf-8') as f:
        f.write(xmp)


def generate(out_dir: str, count: int, radius: float, start_angle: float, direction: str):
    os.makedirs(out_dir, exist_ok=True)
    step = 360.0 / count
    sign = -1.0 if direction.lower() == 'cw' else 1.0
    for idx in range(count):
        angle = start_angle + sign * idx * step
        pos, r, u, f = basis(angle, radius, cross_order=CURRENT_CROSS)
        R9 = assemble(r,u,f, mode=CURRENT_MODE, forward_sign=CURRENT_SIGN)
        fname = os.path.join(out_dir, f'stack_{idx:02d}.xmp')
        write_xmp(fname, pos, R9)

if __name__ == '__main__':
    ap = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
        description='Generate multiple pose matrix convention variants for RC testing.')
    ap.add_argument('--out-root', required=True, help='Root directory to create variant subdirectories')
    ap.add_argument('--count', type=int, default=64)
    ap.add_argument('--radius', type=float, default=4.9)
    ap.add_argument('--start-angle', type=float, default=0.0)
    ap.add_argument('--direction', choices=['cw','ccw'], default='ccw')
    args = ap.parse_args()

    for idx,(cross,sign,mode) in enumerate(PATTERNS):
        variant_dir = os.path.join(args.out_root, f'{idx:02d}_{cross}__sign-{sign}__mode-{mode}')
        CURRENT_CROSS = cross
        CURRENT_SIGN = sign
        CURRENT_MODE = mode
        generate(variant_dir, args.count, args.radius, args.start_angle, args.direction)
        print(f'Generated variant: {variant_dir}')

    print('Done. Test each variant by copying its XMPs next to the images and importing into RealityCapture.')
