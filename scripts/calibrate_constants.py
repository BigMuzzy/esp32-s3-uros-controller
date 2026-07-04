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

    # spin via the back+front centreline chords (no protractor, no axle
    # centre): 10 turns, back->front L=0.60 m, |F F'|=0.060, |B B'|=0.038,
    # marks stopped just past the start
    ./calibrate_constants.py spin-marks --odom-deg 3600 --turns 10 \
        --baseline 0.60 --chord-front 0.060 --chord-back 0.038

    # one square closure from four tape distances between floor marks
    ./calibrate_constants.py closure -L 0.40 --aa 0.21 --bb 0.19 \
        --ab 0.45 --ba 0.43 --side left

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
DEFAULT_WHEEL_DIAMETER_M = 0.17068
DEFAULT_TRACK_WIDTH_M    = 0.54481


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


# ── distance-only pose readout ───────────────────────────────────────
#
# Measuring a planar closure (x, y, heading) with a protractor is the
# cumbersome part of the bench procedure.  Distances are easier and more
# precise to read with a tape, so capture the pose from tape distances
# between two start datums and their end marks instead.
#
# Datums (mark on the floor with a plumb-bob / down-laser):
#   A  = position datum   — the drive-axle centre (the spin centre).
#   B  = heading datum     — a second centreline point (e.g. the front),
#                            a fixed body distance L = |A B| ahead of A.
# After the motion, re-mark the same two points as A' and B'.
#
# Start frame: A at the origin, B on +x at L, so +x is FORWARD and +y is
# LEFT (matches the closure sign convention used by closure_stats).


def pose_from_marks(baseline_m: float,
                    d_aa: float, d_bb: float,
                    d_ab: float, d_ba: float,
                    side: float = 1.0) -> Tuple[float, float, float, float]:
    """Signed planar closure from four tape distances between floor marks.

    ``baseline_m`` is the body distance L = |A B|.  The four distances are::

        d_aa = |A A'|   d_ba = |B A'|     (locate A')
        d_ab = |A B'|   d_bb = |B B'|     (locate B')

    Four mutual distances fix the configuration only up to a reflection
    across the start-heading line (A-B); ``side`` resolves it: +1 if the
    end pose fell to the LEFT of the start heading, -1 if to the RIGHT.

    Returns ``(dx, dy, dtheta_deg, baseline_residual_m)`` in the start
    frame: ``dx`` = +forward, ``dy`` = +left, ``dtheta_deg`` = +CCW.
    ``baseline_residual_m`` is ``| |A'B'| - L |`` — a tape-consistency
    check; a large value means a mis-measured distance, re-read the tape.
    """
    L = float(baseline_m)
    if L <= 0:
        raise ValueError("baseline (|A B|) must be > 0")

    # A' and B' each lie at the intersection of two circles centred on the
    # start datums A=(0,0) and B=(L,0); solve the x first (unambiguous),
    # then the y magnitude.
    xa = (d_aa ** 2 - d_ba ** 2 + L ** 2) / (2.0 * L)
    xb = (d_ab ** 2 - d_bb ** 2 + L ** 2) / (2.0 * L)
    ya_mag = math.sqrt(max(0.0, d_aa ** 2 - xa ** 2))
    yb_mag = math.sqrt(max(0.0, d_ab ** 2 - xb ** 2))

    # The relative sign of (ya, yb) is fixed by the rigid constraint
    # |A'B'| = L; pick the combination that satisfies it best.
    best = None
    for sa in (1.0, -1.0):
        for sb in (1.0, -1.0):
            ya, yb = sa * ya_mag, sb * yb_mag
            err = abs(math.hypot(xb - xa, yb - ya) - L)
            if best is None or err < best[0]:
                best = (err, sa, sb)
    _, sa, sb = best

    # Apply the global left/right `side` via the more reliable (longer) y
    # arm; the two valid solutions are exact mirrors, so flip both signs
    # together.
    want = math.copysign(1.0, side if side != 0 else 1.0)
    if ya_mag >= yb_mag:
        if sa != want:
            sa, sb = -sa, -sb
    else:
        if sb != want:
            sa, sb = -sa, -sb
    ya, yb = sa * ya_mag, sb * yb_mag

    dtheta = math.degrees(math.atan2(yb - ya, xb - xa))
    residual = abs(math.hypot(xb - xa, yb - ya) - L)
    return xa, ya, dtheta, residual


def leftover_from_chords(baseline_m: float,
                         chord_front_m: float, chord_back_m: float,
                         past: bool = True) -> float:
    """Spin leftover angle (deg) from the back + front centreline chords.

    An in-place spin rotates the rigid chassis by the leftover angle about a
    centre that lies on the centreline *between* the back datum B and the
    front datum F.  Each datum sweeps a chord proportional to its radius from
    that centre; because the two radii sum to the front-to-back baseline
    ``L = |B F|``, the two chords sum to ``2 L sin(theta/2)`` no matter where
    the centre sits::

        |F F'| + |B B'| = 2 L sin(theta / 2)

    so the hard-to-locate axle / spin centre never has to be marked — only
    the two easy chassis-end points.  ``past`` is the spin-direction sign:
    the marks stopped just PAST the start (True, +) or SHORT of it
    (False, -).  Magnitude only — keep the leftover under 180 deg (stop
    within half a turn of an integer count) so the chords are unambiguous.
    """
    L = float(baseline_m)
    if L <= 0:
        raise ValueError("baseline (|B F| back->front) must be > 0")
    chord_sum = float(chord_front_m) + float(chord_back_m)
    ratio = chord_sum / (2.0 * L)
    if ratio > 1.0 + 1e-6:
        raise ValueError(
            f"chords sum {chord_sum:.4f} m > 2L ({2 * L:.4f} m): "
            "check L or the chords")
    mag = math.degrees(2.0 * math.asin(min(1.0, max(0.0, ratio))))
    return mag if past else -mag


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


def _cmd_closure(args: argparse.Namespace) -> int:
    side = 1.0 if args.side == "left" else -1.0
    dx, dy, dth, resid = pose_from_marks(
        args.baseline, args.aa, args.bb, args.ab, args.ba, side)
    print("square closure from marks:")
    print(f"  baseline L = |A B|      : {args.baseline:.4f} m")
    print(f"  end pose fell to the    : {args.side}")
    print(f"  closure x (fwd)  : {dx:+.4f} m")
    print(f"  closure y (left) : {dy:+.4f} m")
    print(f"  closure heading  : {dth:+.2f} deg")
    print(f"  tape check ||A'B'|-L|   : {resid:.4f} m", end="")
    if resid > max(0.02, 0.03 * args.baseline):
        print("   <-- LARGE: re-read a distance")
    else:
        print("   (ok)")
    print()
    print("  append this row to your closures CSV (x,y,theta_deg):")
    print(f"    {dx:.4f},{dy:.4f},{dth:.2f}")
    return 0


def _cmd_spin_marks(args: argparse.Namespace) -> int:
    leftover = leftover_from_chords(
        args.baseline, args.chord_front, args.chord_back, past=not args.short)
    physical = args.turns * 360.0 + leftover
    chord_sum = args.chord_front + args.chord_back
    print("in-place spin test (chord readout):")
    print(f"  full turns counted : {args.turns:g}  ({args.turns * 360.0:.0f} deg)")
    print(f"  baseline L = |B F| : {args.baseline:.4f} m  (back->front)")
    print(f"  chord |F F'|       : {args.chord_front:.4f} m")
    print(f"  chord |B B'|       : {args.chord_back:.4f} m")
    print(f"  chords sum         : {chord_sum:.4f} m -> leftover "
          f"{leftover:+.2f} deg ({'short' if args.short else 'past'})")
    print(f"  physical total     : {physical:.2f} deg")
    print()
    return _cmd_track(argparse.Namespace(
        odom_deg=args.odom_deg, physical_deg=physical, current=args.current))


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

    pcl = sub.add_parser(
        "closure",
        help="four tape distances between floor marks -> x,y,theta closure")
    pcl.add_argument("--baseline", "-L", type=float, required=True,
                     help="body distance |A B| between the two datums (m)")
    pcl.add_argument("--aa", type=float, required=True, help="|A A'| (m)")
    pcl.add_argument("--bb", type=float, required=True, help="|B B'| (m)")
    pcl.add_argument("--ab", type=float, required=True, help="|A B'| (m)")
    pcl.add_argument("--ba", type=float, required=True, help="|B A'| (m)")
    pcl.add_argument("--side", choices=("left", "right"), required=True,
                     help="which side of the start heading the end pose fell")
    pcl.set_defaults(func=_cmd_closure)

    psm = sub.add_parser(
        "spin-marks",
        help="spin leftover from the back+front centreline chords -> "
             "TRACK_WIDTH_M")
    psm.add_argument("--odom-deg", type=float, required=True, dest="odom_deg",
                     help="total yaw odom reported (deg)")
    psm.add_argument("--turns", type=float, required=True,
                     help="full turns you counted (e.g. 10)")
    psm.add_argument("--baseline", "-L", type=float, required=True,
                     help="back -> front centreline distance |B F| (m)")
    psm.add_argument("--chord-front", type=float, required=True,
                     dest="chord_front",
                     help="chord |F F'| between start/end FRONT marks (m)")
    psm.add_argument("--chord-back", type=float, required=True,
                     dest="chord_back",
                     help="chord |B B'| between start/end BACK marks (m)")
    psm.add_argument("--short", action="store_true",
                     help="marks stopped SHORT of start (default: past)")
    psm.add_argument("--current", type=float, default=DEFAULT_TRACK_WIDTH_M,
                     help=f"current TRACK_WIDTH_M (default "
                          f"{DEFAULT_TRACK_WIDTH_M})")
    psm.set_defaults(func=_cmd_spin_marks)

    args = p.parse_args(argv)
    try:
        return args.func(args)
    except (ValueError, OSError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
