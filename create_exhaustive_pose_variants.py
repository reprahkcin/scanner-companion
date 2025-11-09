#!/usr/bin/env python3
"""Exhaustive RealityCapture pose variant generator.

Generates a large grid of XMP sidecar sets, each exploring a different
rotation matrix convention and basis construction.

Dimensions enumerated:
  up_axis:        z_up | y_up                (world up vector)
  cross_order:    f_x_up | up_x_f            (right = f x upW OR upW x f)
  forward_sign:   f | -f                     (use forward inward vector or negate)
  row_order:      r_u_f | r_u_negf | r_f_u | u_r_f   (permutation/sign of basis rows)
  transpose:      noT | T                    (if T, matrix is transposed after assembly)

Total variants: 2 * 2 * 2 * 4 * 2 = 64

Each variant gets its own folder named:
  <index>_<upAxis>__<cross>__<fSign>__<rowOrder>__<transpose>
Example:
  00_z_up__f_x_up__f__r_u_f__noT

Usage:
  python create_exhaustive_pose_variants.py --out-root temp/exhaustive_variants --count 64 --radius 4.9 --start-angle 0 --direction ccw

Copy ONE variant's XMPs next to the images and import into RealityCapture.
Record which index yields inward-facing, correctly ordered frusta.
"""
from __future__ import annotations
import math, os, argparse
from xml.sax.saxutils import escape

XCR_NS = 'http://www.capturingreality.com/ns/xcr/1.1#'

UP_AXES = {
    'z_up': (0.0, 0.0, 1.0),
    'y_up': (0.0, 1.0, 0.0),
}
CROSS_ORDERS = ['f_x_up','up_x_f']
FORWARD_SIGNS = ['f','-f']
ROW_ORDERS = ['r_u_f','r_u_negf','r_f_u','u_r_f']
TRANSPOSE = ['noT','T']


def normalize(v):
    m = math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]) or 1.0
    return (v[0]/m, v[1]/m, v[2]/m)


def basis(angle_deg: float, radius: float, upW):
    th = math.radians(angle_deg)
    x = radius * math.cos(th)
    y = radius * math.sin(th)
    z = 0.0
    f = (-math.cos(th), -math.sin(th), 0.0)  # inward forward
    return (x,y,z), f, upW


def compute_right(f, upW, cross_order):
    if cross_order == 'up_x_f':
        r = (
            upW[1]*f[2] - upW[2]*f[1],
            upW[2]*f[0] - upW[0]*f[2],
            upW[0]*f[1] - upW[1]*f[0],
        )
    else:
        r = (
            f[1]*upW[2] - f[2]*upW[1],
            f[2]*upW[0] - f[0]*upW[2],
            f[0]*upW[1] - f[1]*upW[0],
        )
    return normalize(r)


def compute_up(f, r, cross_order):
    # preserve right-handedness depending on which cross produced r
    if cross_order == 'up_x_f':
        # r = upW x f  -> up = f x r
        u = (
            f[1]*r[2] - f[2]*r[1],
            f[2]*r[0] - f[0]*r[2],
            f[0]*r[1] - f[1]*r[0],
        )
    else:
        # r = f x upW  -> up = r x f
        u = (
            r[1]*f[2] - r[2]*f[1],
            r[2]*f[0] - r[0]*f[2],
            r[0]*f[1] - r[1]*f[0],
        )
    return normalize(u)


def assemble_rows(r, u, f, forward_sign, row_order):
    f_use = f if forward_sign == 'f' else (-f[0], -f[1], -f[2])
    if row_order == 'r_u_f':
        rows = [r,u,f_use]
    elif row_order == 'r_u_negf':  # explicit with sign ignored above
        rows = [r,u,(-f_use[0], -f_use[1], -f_use[2])]
    elif row_order == 'r_f_u':
        rows = [r,f_use,u]
    elif row_order == 'u_r_f':
        rows = [u,r,f_use]
    else:
        rows = [r,u,f_use]
    R9 = rows[0] + rows[1] + rows[2]
    return R9


def maybe_transpose(R9, transpose_flag):
    if transpose_flag == 'noT':
        return R9
    # R9 is row-major 3x3 -> transpose
    r1 = R9[0:3]; r2 = R9[3:6]; r3 = R9[6:9]
    T = (r1[0], r2[0], r3[0],
         r1[1], r2[1], r3[1],
         r1[2], r2[2], r3[2])
    return T


def write_xmp(path: str, pos, R9):
    rotation_str = ' '.join(f'{v:.10f}' for v in R9)
    position_str = ' '.join(f'{v:.6f}' for v in pos)
    xmp = f"""<x:xmpmeta xmlns:x='adobe:ns:meta/'>\n  <rdf:RDF xmlns:rdf='http://www.w3.org/1999/02/22-rdf-syntax-ns#'>\n    <rdf:Description xmlns:xcr='{XCR_NS}' xcr:Version='3' xcr:PosePrior='locked' xcr:CalibrationPrior='exact' xcr:Coordinates='absolute' xcr:DistortionModel='brown3' xcr:DistortionCoeficients='0 0 0 0 0 0' xcr:FocalLength35mm='50' xcr:Skew='0' xcr:AspectRatio='1' xcr:PrincipalPointU='0' xcr:PrincipalPointV='0' xcr:CalibrationGroup='-1' xcr:DistortionGroup='-1' xcr:InTexturing='1' xcr:InMeshing='1'>\n      <xcr:Rotation>{rotation_str}</xcr:Rotation>\n      <xcr:Position>{position_str}</xcr:Position>\n    </rdf:Description>\n  </rdf:RDF>\n</x:xmpmeta>\n"""
    with open(path,'w',encoding='utf-8') as f:
        f.write(xmp)


def generate_variant(out_dir: str, count: int, radius: float, start_angle: float, direction: str,
                     up_axis_key: str, cross_order: str, forward_sign: str, row_order: str, transpose_flag: str):
    os.makedirs(out_dir, exist_ok=True)
    upW = UP_AXES[up_axis_key]
    step = 360.0 / count
    sign = -1.0 if direction.lower() == 'cw' else 1.0
    for idx in range(count):
        angle = start_angle + sign * idx * step
        pos, f, upW_local = basis(angle, radius, upW)
        r = compute_right(f, upW_local, cross_order)
        u = compute_up(f, r, cross_order)
        R9 = assemble_rows(r,u,f, forward_sign, row_order)
        R9 = maybe_transpose(R9, transpose_flag)
        fname = os.path.join(out_dir, f'stack_{idx:02d}.xmp')
        write_xmp(fname, pos, R9)


def main():
    ap = argparse.ArgumentParser(description='Generate exhaustive matrix convention variants for RC testing.')
    ap.add_argument('--out-root', required=True, help='Root directory for variant subfolders')
    ap.add_argument('--count', type=int, default=64)
    ap.add_argument('--radius', type=float, default=4.9)
    ap.add_argument('--start-angle', type=float, default=0.0)
    ap.add_argument('--direction', choices=['cw','ccw'], default='ccw')
    args = ap.parse_args()

    combinations = []
    for up_axis in UP_AXES.keys():
        for cross in CROSS_ORDERS:
            for fsign in FORWARD_SIGNS:
                for row_order in ROW_ORDERS:
                    for tflag in TRANSPOSE:
                        combinations.append((up_axis,cross,fsign,row_order,tflag))

    for idx, (up_axis,cross,fsign,row_order,tflag) in enumerate(combinations):
        dirname = f"{idx:02d}_{up_axis}__{cross}__{fsign}__{row_order}__{tflag}"
        out_dir = os.path.join(args.out_root, dirname)
        generate_variant(out_dir, args.count, args.radius, args.start_angle, args.direction,
                         up_axis, cross, fsign, row_order, tflag)
        print(f"Generated {dirname}")

    print(f"Done. {len(combinations)} variants created under {args.out_root}\nSelect ONE folder at a time, copy XMPs next to images, import, report working index.")

if __name__ == '__main__':
    main()
