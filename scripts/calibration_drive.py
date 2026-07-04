#!/usr/bin/env python3
"""
calibration_drive.py — closed-loop odometry-closure calibration driver.

Drives the robot through the three bench tests used to calibrate the
diff-drive constants in ``firmware/main/diff_drive.h`` and the odom
covariance in ``firmware/main/uros_task.c`` (roadmap M1).  See
``docs/odometry-calibration.md`` for the full procedure and the theory.

Tests (subcommands):
    straight   drive forward a fixed odom distance      -> WHEEL_DIAMETER_M
    spin       rotate in place a fixed odom yaw          -> TRACK_WIDTH_M
    square     drive an N x N square, N times CW + CCW   -> covariance

How it works
------------
* Streams ``/cmd_vel`` at a fixed rate so the firmware never drops to
  FAILSAFE_STOP (the firmware times cmd_vel out after 500 ms).
* Closes each motion segment on ``/odom`` feedback and auto-stops.
* Calls the ``/reset_odom`` service (std_srvs/Trigger) to zero the pose
  between trials — no reboot needed.
* Ground truth is always your tape measure: the tool reports what odom
  believed and prompts you for the physical measurement.

Safety
------
* Hard speed caps (``--max-linear`` / ``--max-angular``); requested
  speeds are clamped.
* Aborts a segment if ``/failsafe/active`` goes true mid-drive (RC
  override, agent hiccup, cmd_vel starvation).
* Aborts a segment that does not complete in time (robot disarmed?).
* Ctrl-C and every exit path publish a zero Twist before quitting.

This drives a real robot.  Keep the area clear and stay on the
e-stop / RC kill.

Requires a ROS 2 environment with rclpy + the micro-ROS agent running.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

# ── DDS vendor selection ──────────────────────────────────────
# The micro-ROS Agent bridges the ESP32 onto the DDS graph using Fast DDS.
# ROS 2 *topics* interoperate across DDS vendors (so /cmd_vel and /odom work
# fine from a Cyclone DDS client), but service *reply* correlation does not: a
# Cyclone DDS client never receives the reply to a Fast-DDS-backed service.
# The firmware still runs the request — you see "reset_odom: pose zeroed" in
# its log — but the reply is dropped and the caller times out.  This tool
# depends on the /reset_odom service, so it defaults to Fast DDS to match the
# agent.  The calibration node only talks to the firmware via the agent, so
# this does not affect the rest of the (Cyclone) system.  Override with
# CALIB_RMW=<rmw_name>, or CALIB_RMW=keep to leave RMW_IMPLEMENTATION alone.
_calib_rmw = os.environ.get("CALIB_RMW", "rmw_fastrtps_cpp")
if _calib_rmw != "keep" and os.environ.get("RMW_IMPLEMENTATION") != _calib_rmw:
    print(f"[calibration_drive] setting RMW_IMPLEMENTATION={_calib_rmw} "
          f"(was {os.environ.get('RMW_IMPLEMENTATION', 'unset')}) to match the "
          "micro-ROS agent so /reset_odom replies arrive; "
          "set CALIB_RMW=keep to override.", file=sys.stderr)
    os.environ["RMW_IMPLEMENTATION"] = _calib_rmw

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger
except ImportError as e:  # pragma: no cover - depends on host ROS env
    print(f"error: ROS 2 / rclpy not available: {e}\n"
          "Source your ROS 2 setup (e.g. 'source /opt/ros/<distro>/setup.bash')",
          file=sys.stderr)
    raise SystemExit(1)

# Reuse the pure-math helpers so we can print suggestions inline.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from calibrate_constants import (
        corrected_wheel_diameter,
        corrected_track_width,
        closure_stats,
        pose_from_marks,
        leftover_from_chords,
        DEFAULT_WHEEL_DIAMETER_M,
        DEFAULT_TRACK_WIDTH_M,
    )
    _HAVE_MATH = True
except ImportError:
    _HAVE_MATH = False
    DEFAULT_WHEEL_DIAMETER_M = 0.17037
    DEFAULT_TRACK_WIDTH_M = 0.54481


class Abort(Exception):
    """Raised when a motion segment must stop early (safety / timeout)."""


def _yaw_from_quat(z: float, w: float) -> float:
    """Planar yaw from the z,w quaternion terms the firmware sets."""
    return 2.0 * math.atan2(z, w)


# Terminal-approach creep speeds. The last few percent of every segment is
# driven this slow (or 0.12x the segment speed, whichever is larger, but never
# above the 0.4x ease-out speed) so the robot is barely moving when it reaches
# the target and the post-stop coast is negligible. Single-segment straight/spin
# tests hide stop overshoot; the square pays it at all four corners, so a clean
# terminal creep is what keeps the corners from over-running.
_CREEP_LINEAR_MPS = 0.05
_CREEP_ANGULAR_RPS = 0.10
# Ease-in ramp time: each segment accelerates from the creep speed up to full
# over this many seconds instead of stepping from rest to full, to limit the
# wheel slip on a hard start (slip over-counts the encoders -> odom over-reports
# -> the segment stops short). Negligible over the long straight/spin moves; the
# square benefits at all eight starts per run.
_EASE_IN_TIME_S = 1.5
# Stop-lead: a turn coasts a roughly fixed TIME past the moment the stop is
# commanded (cmd/odom pipeline + firmware decel ramp) — measured ~0.40 s and
# constant across speed, so the overshoot is _STOP_LEAD_S * rate. We command the
# stop one lead early (in angle: _STOP_LEAD_S * rate) so the coast lands on the
# target instead of past it. Re-measure with `spin --constant` and adjust here
# if your drives differ.
_STOP_LEAD_S = 0.4255


class CalibrationDriver(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("odom_calibration")
        self._args = args
        self._lock = threading.Lock()

        self._target = Twist()
        self._x = 0.0
        self._y = 0.0
        self._yaw_cont = 0.0        # unwrapped, continuous
        self._prev_yaw = None       # last raw yaw sample
        self._have_odom = False
        self._failsafe = False

        self._pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.create_subscription(Odometry, args.odom_topic, self._odom_cb, 10)
        self.create_subscription(Bool, args.failsafe_topic, self._fs_cb, 10)
        self._reset_cli = self.create_client(Trigger, args.reset_service)

        # Continuous cmd_vel stream: always publishes the current target
        # (zero when stopped) so the firmware watchdog stays satisfied.
        self.create_timer(1.0 / args.rate_hz, self._stream_cb)

    # ── callbacks ────────────────────────────────────────────────────
    def _stream_cb(self):
        with self._lock:
            self._pub.publish(self._target)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        yaw = _yaw_from_quat(p.orientation.z, p.orientation.w)
        with self._lock:
            self._x = p.position.x
            self._y = p.position.y
            if self._prev_yaw is None:
                self._yaw_cont = yaw
            else:
                d = yaw - self._prev_yaw
                # unwrap across the ±π seam
                while d > math.pi:
                    d -= 2.0 * math.pi
                while d < -math.pi:
                    d += 2.0 * math.pi
                self._yaw_cont += d
            self._prev_yaw = yaw
            self._have_odom = True

    def _fs_cb(self, msg: Bool):
        with self._lock:
            self._failsafe = bool(msg.data)

    # ── shared-state accessors ───────────────────────────────────────
    def _snapshot(self):
        with self._lock:
            return (self._x, self._y, self._yaw_cont,
                    self._have_odom, self._failsafe)

    def _set_target(self, lin: float, ang: float):
        with self._lock:
            self._target.linear.x = float(lin)
            self._target.angular.z = float(ang)

    def stop(self):
        self._set_target(0.0, 0.0)

    # ── primitives ───────────────────────────────────────────────────
    def wait_for_odom(self, timeout: float = 10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._snapshot()[3]:
                return
            time.sleep(0.05)
        raise Abort("no /odom received — is the agent up and the node running?")

    def reset_odom(self):
        if not self._reset_cli.wait_for_service(timeout_sec=5.0):
            raise Abort(f"service {self._args.reset_service} unavailable")
        fut = self._reset_cli.call_async(Trigger.Request())
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > 5.0:
                raise Abort("reset_odom call timed out")
            time.sleep(0.02)
        res = fut.result()
        if res is None or not res.success:
            raise Abort("reset_odom call failed")
        # Let any in-flight odom flush, then re-seed the continuous-yaw
        # accumulator so it tracks from the firmware's fresh zero.
        time.sleep(0.3)
        with self._lock:
            self._prev_yaw = None
            self._yaw_cont = 0.0
            self._x = 0.0
            self._y = 0.0

    def _guard(self, t0: float, timeout: float, what: str):
        _, _, _, _, failsafe = self._snapshot()
        if failsafe:
            self.stop()
            raise Abort(f"{what}: /failsafe/active went TRUE — RC override, "
                        "agent hiccup, or cmd_vel starvation")
        if time.time() - t0 > timeout:
            self.stop()
            raise Abort(f"{what}: did not complete in {timeout:.0f}s — "
                        "robot armed? on the ground? check RC mode")

    def drive_distance(self, dist_m: float, speed: float):
        """Drive straight until |displacement| from start >= dist_m."""
        speed = min(speed, self._args.max_linear)
        creep = min(0.4 * speed, max(_CREEP_LINEAR_MPS, 0.12 * speed))
        x0, y0, _, _, _ = self._snapshot()
        target = abs(dist_m)
        timeout = target / max(speed, 1e-3) * 3.0 + 4.0
        t0 = time.time()
        while True:
            x, y, _, _, _ = self._snapshot()
            d = math.hypot(x - x0, y - y0)
            if d >= target:
                break
            self._guard(t0, timeout, "forward segment")
            frac_left = (target - d) / target
            if frac_left < 0.03:
                v = creep             # terminal creep — kills stop overshoot
            elif frac_left < 0.15:
                v = 0.4 * speed       # ease-out
            else:
                v = speed
            # ease-in: ramp up from creep over the first _EASE_IN_TIME_S so the
            # start is soft and the wheels do not slip out of the gate.
            ramp = min(1.0, (time.time() - t0) / _EASE_IN_TIME_S)
            v = min(v, creep + (speed - creep) * ramp)
            self._set_target(math.copysign(v, dist_m), 0.0)
            time.sleep(0.02)
        self.stop()
        self.wait_until_settled()     # let the coast bleed off before the next

    def rotate_angle(self, angle_rad: float, rate: float, lead: bool = True):
        """Rotate in place until |yaw change| >= |angle_rad|.

        A turn coasts a fixed time (~_STOP_LEAD_S) past the moment the stop is
        commanded, so the overshoot is _STOP_LEAD_S * rate at any speed. We turn
        at a flat ``rate`` (soft ease-in only) and command the stop one lead
        early, so the robot coasts onto the target instead of past it.
        ``lead=False`` disables the compensation — used by the overshoot probe
        to measure the raw coast.
        """
        rate = min(rate, self._args.max_angular)
        creep = min(0.4 * rate, max(_CREEP_ANGULAR_RPS, 0.12 * rate))
        _, _, yaw0, _, _ = self._snapshot()
        target = abs(angle_rad)
        stop_lead = _STOP_LEAD_S * rate if lead else 0.0
        stop_at = target - min(stop_lead, 0.5 * target)
        timeout = target / max(rate, 1e-3) * 3.0 + 4.0
        t0 = time.time()
        while True:
            _, _, yaw, _, _ = self._snapshot()
            d = abs(yaw - yaw0)
            if d >= stop_at:
                break
            self._guard(t0, timeout, "turn segment")
            # ease-in only: ramp creep -> rate over _EASE_IN_TIME_S for a soft
            # start; the stop is handled by the lead, not by tapering down.
            ramp = min(1.0, (time.time() - t0) / _EASE_IN_TIME_S)
            w = min(rate, creep + (rate - creep) * ramp)
            self._set_target(0.0, math.copysign(w, angle_rad))
            time.sleep(0.02)
        self.stop()
        self.wait_until_settled()

    def wait_until_settled(self, lin_eps: float = 0.002,
                           ang_eps: float = math.radians(0.2),
                           settle_time: float = 0.25,
                           timeout: float = 3.0):
        """Block until odom stops changing (the robot has coasted to rest).

        Samples the pose; once it moves less than (lin_eps, ang_eps) between
        consecutive samples for `settle_time` continuously, returns. Bounded
        by `timeout` so a terminal creep can never hang the run. Dead-reckoned
        odom does not move while the wheels are stopped, so the thresholds can
        be tight; this both removes stop overshoot from the measurement and
        guarantees the next segment's start pose is captured at rest.
        """
        t0 = time.time()
        px, py, pyaw, _, _ = self._snapshot()
        stable_since = None
        while time.time() - t0 < timeout:
            time.sleep(0.05)
            x, y, yaw, _, _ = self._snapshot()
            moved = (math.hypot(x - px, y - py) >= lin_eps
                     or abs(yaw - pyaw) >= ang_eps)
            px, py, pyaw = x, y, yaw
            if moved:
                stable_since = None
            elif stable_since is None:
                stable_since = time.time()
            elif time.time() - stable_since >= settle_time:
                return

    def odom_distance(self) -> float:
        x, y, _, _, _ = self._snapshot()
        return math.hypot(x, y)

    def odom_yaw_deg(self) -> float:
        _, _, yaw, _, _ = self._snapshot()
        return math.degrees(yaw)


# ── test orchestration ───────────────────────────────────────────────

def _prompt_float(msg: str) -> float:
    while True:
        try:
            return float(input(msg).strip())
        except ValueError:
            print("  please enter a number")


def _prompt_choice(msg: str, choices: dict):
    """Prompt until the user types one of the keys in `choices` (case- and
    whitespace-insensitive); return the mapped value."""
    keys = "/".join(choices)
    while True:
        ans = input(f"{msg} [{keys}]: ").strip().lower()
        if ans in choices:
            return choices[ans]
        print(f"  please enter one of: {keys}")


def run_straight(node: CalibrationDriver, args: argparse.Namespace) -> int:
    print(f"\n=== STRAIGHT test: drive {args.distance:.2f} m forward ===")
    print("Mark the start. Keep ~5 m clear ahead.")
    input("Press Enter to reset odom and drive (Ctrl-C aborts)... ")
    node.reset_odom()
    node.drive_distance(args.distance, args.linear_speed)
    odom_d = node.odom_distance()
    print(f"\nodom reported {odom_d:.4f} m.")
    tape = _prompt_float("Measure the physical distance with a tape (m): ")
    if _HAVE_MATH:
        new = corrected_wheel_diameter(args.current_diameter, odom_d, tape)
        ratio = tape / odom_d
        print(f"\n  scale (tape/odom): {ratio:.5f}  ({(ratio - 1) * 100:+.2f} %)")
        print(f"  current WHEEL_DIAMETER_M : {args.current_diameter:.5f}")
        print(f"  -> set WHEEL_DIAMETER_M  : {new:.5f}f   "
              "(diff_drive.h, then rebuild+reflash)")
    else:
        print(f"  feed to: calibrate_constants.py wheel "
              f"--odom {odom_d:.4f} --tape {tape:.4f}")
    return 0


def _run_spin_constant(node: CalibrationDriver, args: argparse.Namespace,
                       total_deg: float) -> int:
    """Overshoot probe: hold a flat turn rate, report odom past the command.

    No tape needed. Sweep ``--angular-speed`` to characterise how much the
    robot coasts past the commanded angle at each speed; a roughly constant
    "implied latency" across speeds means the overshoot is a fixed time delay
    (best fixed with a stop-lead) rather than cruise-speed kinetics.
    """
    rate = args.angular_speed
    print(f"\n=== SPIN overshoot probe: {total_deg:.1f} deg at a flat "
          f"{rate:.2f} rad/s ===")
    print("No tape — reports odom overshoot past the commanded angle.")
    input("Press Enter to reset odom and turn (Ctrl-C aborts)... ")
    node.reset_odom()
    direction = 1.0 if not args.clockwise else -1.0
    node.rotate_angle(direction * math.radians(total_deg), rate, lead=False)
    odom_deg = abs(node.odom_yaw_deg())
    overshoot = odom_deg - total_deg
    print(f"\n  commanded : {total_deg:8.2f} deg")
    print(f"  odom      : {odom_deg:8.2f} deg")
    print(f"  overshoot : {overshoot:+8.2f} deg   at {rate:.2f} rad/s")
    if rate > 1e-6:
        tau = math.radians(overshoot) / rate
        print(f"  implied stop latency ~ {tau:+.2f} s  (overshoot / rate)")
        print("  -> if this latency stays ~constant as you vary "
              "--angular-speed, the overshoot is a fixed delay and a stop-lead"
              " fixes it without slowing the turn.")
    return 0


def run_spin(node: CalibrationDriver, args: argparse.Namespace) -> int:
    total_deg = args.turns * 360.0
    if args.constant:
        return _run_spin_constant(node, args, total_deg)
    print(f"\n=== SPIN test: rotate {args.turns:g} turns "
          f"({total_deg:.0f} deg) in place ===")
    print("Mark TWO centreline datums on the floor "
          "(plumb-bob / down-laser):")
    print("  B = BACK centreline point   F = FRONT centreline point")
    print("  (chassis ends sit on the centreline; no axle centre needed.)")
    print("  measure L = |B F| once (back->front), passed via --baseline "
          "or prompted.")
    input("Press Enter to reset odom and spin (Ctrl-C aborts)... ")
    node.reset_odom()
    direction = 1.0 if not args.clockwise else -1.0
    node.rotate_angle(direction * math.radians(total_deg), args.angular_speed)
    odom_deg = abs(node.odom_yaw_deg())
    print(f"\nodom reported {odom_deg:.2f} deg of rotation.")
    print("Re-mark BOTH datums as B' and F'. "
          "Distance-only readout (no protractor):")
    # default the counted turns to the commanded count (Enter to accept)
    raw = input(f"Full turns you counted [{args.turns:g}]: ").strip()
    full = float(raw) if raw else float(args.turns)
    L = args.baseline if args.baseline else _prompt_float(
        "Baseline L = |B F| back->front (m): ")
    chord_front = _prompt_float(
        "Chord |F -> F'| between start/end FRONT marks (m): ")
    chord_back = _prompt_float(
        "Chord |B -> B'| between start/end BACK marks (m): ")
    past = _prompt_choice(
        "Marks stopped PAST or SHORT of start (in spin direction)?",
        {"p": True, "past": True, "s": False, "short": False})
    if _HAVE_MATH:
        leftover = leftover_from_chords(L, chord_front, chord_back, past=past)
        physical = full * 360.0 + leftover
        new = corrected_track_width(args.current_width, odom_deg, physical)
        ratio = odom_deg / physical
        print(f"\n  chords |F F'|+|B B'| : "
              f"{chord_front + chord_back:.4f} m")
        print(f"  leftover from chords : {leftover:+.2f} deg "
              f"({'past' if past else 'short'})")
        print(f"  physical total       : {physical:.2f} deg")
        print(f"  scale (odom/physical): {ratio:.5f}  "
              f"({(ratio - 1) * 100:+.2f} %)")
        print(f"  current TRACK_WIDTH_M : {args.current_width:.5f}")
        print(f"  -> set TRACK_WIDTH_M  : {new:.5f}f   "
              "(diff_drive.h, then rebuild+reflash)")
    else:
        sgn = "" if past else "--short "
        print(f"  feed to: calibrate_constants.py spin-marks "
              f"--odom-deg {odom_deg:.2f} --turns {full:g} "
              f"--baseline {L:.4f} --chord-front {chord_front:.4f} "
              f"--chord-back {chord_back:.4f} {sgn}".rstrip())
    return 0


def _drive_square(node: CalibrationDriver, args: argparse.Namespace,
                  clockwise: bool):
    sign = -1.0 if clockwise else 1.0
    for i in range(4):
        if args.step:
            input(f"    side {i + 1}/4: forward {args.side:.2f} m — Enter... ")
        node.drive_distance(args.side, args.linear_speed)
        if args.step:
            input(f"    corner {i + 1}/4: turn 90 deg — Enter... ")
        node.rotate_angle(sign * math.pi / 2.0, args.angular_speed)


def run_square(node: CalibrationDriver, args: argparse.Namespace) -> int:
    print(f"\n=== SQUARE test: {args.side:.2f} m sides, "
          f"{args.trials} runs each direction ===")
    print(f"Clear a ~{args.side + 1:.0f} x {args.side + 1:.0f} m area.")
    print("Mark TWO start datums on the floor (plumb-bob / down-laser):")
    print("  A = axle centre (position)   B = front centreline point")
    L = args.baseline if args.baseline else _prompt_float(
        "Baseline L = |A B| axle->front, measured once (m): ")
    print("Each run: drive the square, re-mark A' and B', then tape FOUR "
          "distances (no protractor).")
    closures: list[tuple[float, float, float]] = []
    for direction, cw in (("CW", True), ("CCW", False)):
        for run in range(1, args.trials + 1):
            print(f"\n-- {direction} run {run}/{args.trials} --")
            input("Place the robot on the start mark, Enter to drive... ")
            node.reset_odom()
            _drive_square(node, args, clockwise=cw)
            print("Square complete (odom believes it is back at the start).")
            print("Re-mark A' (axle) and B' (front), then tape these four:")
            d_aa = _prompt_float("  |A  -> A'| (m): ")
            d_ba = _prompt_float("  |B  -> A'| (m): ")
            d_ab = _prompt_float("  |A  -> B'| (m): ")
            d_bb = _prompt_float("  |B  -> B'| (m): ")
            sidev = _prompt_choice(
                "  end pose fell to the LEFT or RIGHT of the start heading?",
                {"l": 1.0, "left": 1.0, "r": -1.0, "right": -1.0})
            if _HAVE_MATH:
                dx, dy, dth, resid = pose_from_marks(
                    L, d_aa, d_bb, d_ab, d_ba, sidev)
                if resid > max(0.02, 0.03 * L):
                    print(f"    ! tape check ||A'B'|-L| = {resid:.3f} m is "
                          "large — re-measure a distance and re-enter.")
                    if _prompt_choice("    keep this run anyway?",
                                      {"y": True, "n": False}) is False:
                        d_aa = _prompt_float("  |A  -> A'| (m): ")
                        d_ba = _prompt_float("  |B  -> A'| (m): ")
                        d_ab = _prompt_float("  |A  -> B'| (m): ")
                        d_bb = _prompt_float("  |B  -> B'| (m): ")
                        dx, dy, dth, resid = pose_from_marks(
                            L, d_aa, d_bb, d_ab, d_ba, sidev)
                print(f"    closure: x={dx:+.3f} y={dy:+.3f} "
                      f"theta={dth:+.2f} deg  (tape check {resid:.3f} m)")
                closures.append((dx, dy, dth))
            else:
                print("    calibrate_constants not importable; raw distances:")
                print(f"    closure -L {L:.4f} --aa {d_aa:.4f} --bb {d_bb:.4f} "
                      f"--ab {d_ab:.4f} --ba {d_ba:.4f} "
                      f"--side {'left' if sidev > 0 else 'right'}")

    print("\n=== closure summary ===")
    for i, (x, y, th) in enumerate(closures, 1):
        print(f"  run {i:2d}: x={x:+.3f} y={y:+.3f} theta={th:+.2f}")
    if _HAVE_MATH and len(closures) >= 2:
        s = closure_stats(closures)
        print(f"\n  mean residual: x={s['mean_x']:+.4f} y={s['mean_y']:+.4f} "
              f"theta={math.degrees(s['mean_theta_rad']):+.2f} deg")
        print(f"  variance:      x={s['var_x']:.5f} m^2  "
              f"y={s['var_y']:.5f} m^2  theta={s['var_theta']:.5f} rad^2")
        print("\n  suggested pose covariance diagonals (uros_task.c):")
        print(f"    pose.covariance[0]  = {max(s['var_x'], 1e-6):.5f};")
        print(f"    pose.covariance[7]  = {max(s['var_y'], 1e-6):.5f};")
        print(f"    pose.covariance[35] = {max(s['var_theta'], 1e-6):.5f};")
        print("\n  A residual that flips sign CW vs CCW -> wheel-diameter "
              "error (redo 'straight').")
        print("  A residual consistent in both -> track-width error "
              "(redo 'spin').")
    return 0


# ── entry point ──────────────────────────────────────────────────────

# Common topic / safety options. They are accepted both BEFORE and AFTER the
# subcommand (added to the top-level parser and to every subparser). The copies
# use SUPPRESS so an unspecified side never overwrites a value given on the
# other side; real defaults are filled in afterwards by _apply_common_defaults.
_COMMON_DEFAULTS = {
    "cmd_vel_topic": "/cmd_vel",
    "odom_topic": "/odom",
    "failsafe_topic": "/failsafe/active",
    "reset_service": "/reset_odom",
    "rate_hz": 20.0,
    "linear_speed": 0.3,
    "angular_speed": 0.5,
    "max_linear": 0.6,
    "max_angular": 1.0,
}


def _add_common(parser: argparse.ArgumentParser):
    """Add the common topic/safety options to `parser` (top-level or a sub)."""
    g = parser.add_argument_group(
        "common options (accepted before OR after the subcommand)")
    g.add_argument("--cmd-vel-topic", default=argparse.SUPPRESS)
    g.add_argument("--odom-topic", default=argparse.SUPPRESS)
    g.add_argument("--failsafe-topic", default=argparse.SUPPRESS)
    g.add_argument("--reset-service", default=argparse.SUPPRESS)
    g.add_argument("--rate-hz", type=float, default=argparse.SUPPRESS,
                   help="cmd_vel stream rate (Hz, keep > 2 to beat the "
                        "500 ms firmware timeout) [20.0]")
    g.add_argument("--linear-speed", type=float, default=argparse.SUPPRESS,
                   help="drive speed (m/s) [0.3]")
    g.add_argument("--angular-speed", type=float, default=argparse.SUPPRESS,
                   help="turn rate (rad/s) [0.5]")
    g.add_argument("--max-linear", type=float, default=argparse.SUPPRESS,
                   help="hard linear cap (m/s) [0.6]")
    g.add_argument("--max-angular", type=float, default=argparse.SUPPRESS,
                   help="hard angular cap (rad/s) [1.0]")


def _apply_common_defaults(args: argparse.Namespace):
    """Fill in any common option not supplied on either side of the sub."""
    for key, val in _COMMON_DEFAULTS.items():
        if not hasattr(args, key):
            setattr(args, key, val)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    _add_common(p)

    sub = p.add_subparsers(dest="test", required=True)

    ps = sub.add_parser("straight", help="wheel-diameter test")
    _add_common(ps)
    ps.add_argument("--distance", type=float, default=5.0)
    ps.add_argument("--current-diameter", type=float,
                    default=DEFAULT_WHEEL_DIAMETER_M)
    ps.set_defaults(func=run_straight)

    pn = sub.add_parser("spin", help="track-width test")
    _add_common(pn)
    pn.add_argument("--turns", type=float, default=10.0)
    pn.add_argument("--clockwise", action="store_true")
    pn.add_argument("--baseline", type=float, default=0.0,
                    help="back -> front centreline distance |B F| (m); "
                         "prompted if omitted")
    pn.add_argument("--constant", action="store_true",
                    help="overshoot probe: hold a flat --angular-speed (no "
                         "ease-out/creep) and report odom overshoot past the "
                         "commanded angle; no tape needed")
    pn.add_argument("--current-width", type=float,
                    default=DEFAULT_TRACK_WIDTH_M)
    pn.set_defaults(func=run_spin)

    pq = sub.add_parser("square", help="closure + covariance test")
    _add_common(pq)
    pq.add_argument("--side", type=float, default=2.0)
    pq.add_argument("--baseline", type=float, default=0.0,
                    help="axle-centre -> front datum distance |A B| (m); "
                         "prompted if omitted")
    pq.add_argument("--trials", type=int, default=5,
                    help="runs per direction (CW + CCW)")
    pq.add_argument("--step", action="store_true",
                    help="pause for Enter before each side/corner "
                         "(cautious first run)")
    pq.set_defaults(func=run_square)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    _apply_common_defaults(args)

    rclpy.init()
    node = CalibrationDriver(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    rc = 0
    try:
        node.wait_for_odom()
        rc = args.func(node, args)
    except Abort as e:
        print(f"\nABORTED: {e}", file=sys.stderr)
        rc = 1
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        rc = 130
    finally:
        # Make sure the robot is commanded to stop, several times, before
        # the stream stops.
        for _ in range(10):
            node.stop()
            try:
                node._pub.publish(node._target)  # noqa: SLF001 - belt & braces
            except Exception:
                pass
            time.sleep(0.03)
        # Stop the executor and join its thread BEFORE destroying the node.
        # Tearing down rcl entities while the daemon spin thread is still
        # running races the C++ teardown and aborts the process with
        # "terminate called without an active exception" (core dump on exit).
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
