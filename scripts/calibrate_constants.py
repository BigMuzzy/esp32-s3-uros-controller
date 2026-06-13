#!/usr/bin/env python3
"""
calibrate_constants.py — turn bench measurements into diff-drive constants.

Pure math, no ROS, moves nothing.  Feed it the numbers you read off the
floor during an odometry-closure run (see docs/odometry-calibration.md)
and it prints the corrected values to paste into
``firmware/main/diff_drive.h``, plus suggested odom covariance diagonals
for ``firmware/main/uros_task.c``.

The geometry corrections are multiplicative scale factors, so they are
independent of how far you drove:

    wheel diameter:  WHEEL_DIAMETER_M *= tape_distance / odom_distance
    track width:     TRACK_WIDTH_M    *= odom_yaw      / physical_yaw

Ground truth is always the tape measure / counted turns, never odom.

Usage
-----
    # straight-line test → wheel diameter
    ./calibrate_constants.py wheel --odom 5.00 --tape 4.82

    # in-place spin test → track width
    ./calibrate_constants.py track --odom-deg 3600 --physical-deg 3540

    # square closure errors → covariance diagonals
    #   CSV: one "x,y,theta_deg" closure error per line (or stdin)
    ./calibrate_constants.py cov --csv closures.csv

These functions are also imported by ``calibration_drive.py`` so the
driving tool can print suggestions inline.
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
from typing import List, Tuple

# Current compile-time defaults from firmware/main/diff_drive.h.  Used as
# the baseline a correction scales; override with --current if your tree
# already differs.
DEFAULT_WHEEL_DIAMETER_M = 0.170
DEFAULT_TRACK_WIDTH_M = 0.600


def corrected_wheel_diameter(current_m: float,
                             odom_distance_m: float,
                             tape_distance_m: float) -> float:
    """New WHEEL_DIAMETER_M from a straight-line run.

    Odom distance scales linearly with the assumed wheel diameter, so a
    wheel that is physically smaller than configured makes odom over-report
    distance.  Correct by the measured ratio.
    """
    if odom_distance_m == 0:
        raise ValueError("odom_distance_m must be non-zero")
    return current_m * (tape_distance_m / odom_distance_m)


def corrected_track_width(current_m: float,
                          odom_yaw_deg: float,
                          physical_yaw_deg: float) -> float:
    """New TRACK_WIDTH_M from an in-place spin run.

    Odom integrates yaw as (d_right - d_left) / track_width, so a track
    width that is configured too small makes odom over-report rotation.
    If odom_yaw > physical_yaw the width must increase, hence the ratio
    odom / physical (the inverse direction of the wheel-diameter fix).
    """
    if physical_yaw_deg == 0:
        raise ValueError("physical_yaw_deg must be non-zero")
    return current_m * (odom_yaw_deg / physical_yaw_deg)


def closure_stats(closures: List[Tuple[float, float, float]]) -> dict:
    """Mean + variance of square-closure errors.

    ``closures`` is a list of (x_m, y_m, theta_deg) residuals — how far
    the robot physically was from its start mark after a square that odom
    believed returned to the origin.

    The variances map onto the odom covariance *diagonals* (pose x/y in
    m^2, yaw in rad^2).  The means are the systematic residual: if they
    are large, re-run the wheel/track tests before trusting the spread.
    """
    n = len(closures)
    if n == 0:
        raise ValueError("need at least one closure sample")

    xs = [c[0] for c in closures]
    ys = [c[1] for c in closures]
    ths = [math.radians(c[2]) for c in closures]

    def var(vals: List[float]) -> float:
        # Sample variance needs >= 2 points; fall back to 0 for a single
        # sample (no spread information yet).
        return statistics.variance(vals) if len(vals) >= 2 else 0.0

    return {
        "n": n,
        "mean_x": statistics.fmean(xs),
        "mean_y": statistics.fmean(ys),
        "mean_theta_rad": statistics.fmean(ths),
        "var_x": var(xs),
        "var_y": var(ys),
        "var_theta": var(ths),
    }


# ── CLI ──────────────────────────────────────────────────────────────

def _cmd_wheel(args: argparse.Namespace) -> int:
    new = corrected_wheel_diameter(args.current, args.odom, args.tape)
    ratio = args.tape / args.odom
    print(f"straight-line test:")
    print(f"  odom reported : {args.odom:.4f} m")
    print(f"  tape measured : {args.tape:.4f} m")
    print(f"  scale (tape/odom) : {ratio:.5f}  ({(ratio - 1) * 100:+.2f} %)")
    print()
    print(f"  current WHEEL_DIAMETER_M : {args.current:.5f}")
    print(f"  -> set WHEEL_DIAMETER_M  : {new:.5f}f")
    print(f"     (firmware/main/diff_drive.h, then rebuild + reflash)")
    return 0


def _cmd_track(args: argparse.Namespace) -> int:
    new = corrected_track_width(args.current, args.odom_deg, args.physical_deg)
    ratio = args.odom_deg / args.physical_deg
    print(f"in-place spin test:")
    print(f"  odom reported  : {args.odom_deg:.2f} deg")
    print(f"  physical turns : {args.physical_deg:.2f} deg")
    print(f"  scale (odom/physical) : {ratio:.5f}  ({(ratio - 1) * 100:+.2f} %)")
    print()
    print(f"  current TRACK_WIDTH_M : {args.current:.5f}")
    print(f"  -> set TRACK_WIDTH_M  : {new:.5f}f")
    print(f"     (firmware/main/diff_drive.h, then rebuild + reflash)")
    return 0


def _read_closures(path: str) -> List[Tuple[float, float, float]]:
    fh = sys.stdin if path == "-" else open(path, "r", encoding="utf-8")
    out: List[Tuple[float, float, float]] = []
    try:
        for lineno, raw in enumerate(fh, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p for p in line.replace(",", " ").split() if p]
            if len(parts) != 3:
                raise ValueError(
                    f"line {lineno}: expected 'x,y,theta_deg', got {raw!r}")
            out.append((float(parts[0]), float(parts[1]), float(parts[2])))
    finally:
        if fh is not sys.stdin:
            fh.close()
    return out


def _cmd_cov(args: argparse.Namespace) -> int:
    closures = _read_closures(args.csv)
    s = closure_stats(closures)
    print(f"square closure ({s['n']} runs):")
    print(f"  mean residual  : x={s['mean_x']:+.4f} m  y={s['mean_y']:+.4f} m  "
          f"theta={math.degrees(s['mean_theta_rad']):+.2f} deg")
    if s["n"] < 2:
        print("  (need >= 2 runs for a variance / covariance estimate)")
        return 0
    print(f"  variance       : x={s['var_x']:.5f} m^2  y={s['var_y']:.5f} m^2  "
          f"theta={s['var_theta']:.5f} rad^2")
    print()
    print("  suggested odom pose covariance diagonals "
          "(firmware/main/uros_task.c):")
    print(f"    pose.covariance[0]  = {max(s['var_x'], 1e-6):.5f}; /* x   */")
    print(f"    pose.covariance[7]  = {max(s['var_y'], 1e-6):.5f}; /* y   */")
    print(f"    pose.covariance[35] = {max(s['var_theta'], 1e-6):.5f}; /* yaw */")
    print("  (starting points — large mean residual? fix wheel/track first.)")
    return 0


def main(argv: List[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description="Convert bench odometry measurements into diff-drive "
                    "constants and covariance diagonals.")
    sub = p.add_subparsers(dest="cmd", required=True)

    pw = sub.add_parser("wheel", help="straight-line test -> WHEEL_DIAMETER_M")
    pw.add_argument("--odom", type=float, required=True,
                    help="distance odom reported (m)")
    pw.add_argument("--tape", type=float, required=True,
                    help="distance measured with a tape (m)")
    pw.add_argument("--current", type=float, default=DEFAULT_WHEEL_DIAMETER_M,
                    help=f"current WHEEL_DIAMETER_M (default "
                         f"{DEFAULT_WHEEL_DIAMETER_M})")
    pw.set_defaults(func=_cmd_wheel)

    pt = sub.add_parser("track", help="spin test -> TRACK_WIDTH_M")
    pt.add_argument("--odom-deg", type=float, required=True, dest="odom_deg",
                    help="total yaw odom reported (deg)")
    pt.add_argument("--physical-deg", type=float, required=True,
                    dest="physical_deg",
                    help="physical yaw observed (deg, e.g. 10 turns = 3600)")
    pt.add_argument("--current", type=float, default=DEFAULT_TRACK_WIDTH_M,
                    help=f"current TRACK_WIDTH_M (default "
                         f"{DEFAULT_TRACK_WIDTH_M})")
    pt.set_defaults(func=_cmd_track)

    pc = sub.add_parser("cov", help="square closures -> covariance diagonals")
    pc.add_argument("--csv", default="-",
                    help="CSV of 'x,y,theta_deg' closure errors, one per "
                         "line ('-' = stdin, the default)")
    pc.set_defaults(func=_cmd_cov)

    args = p.parse_args(argv)
    try:
        return args.func(args)
    except (ValueError, OSError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
