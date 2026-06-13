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
        DEFAULT_WHEEL_DIAMETER_M,
        DEFAULT_TRACK_WIDTH_M,
    )
    _HAVE_MATH = True
except ImportError:
    _HAVE_MATH = False
    DEFAULT_WHEEL_DIAMETER_M = 0.170
    DEFAULT_TRACK_WIDTH_M = 0.600


class Abort(Exception):
    """Raised when a motion segment must stop early (safety / timeout)."""


def _yaw_from_quat(z: float, w: float) -> float:
    """Planar yaw from the z,w quaternion terms the firmware sets."""
    return 2.0 * math.atan2(z, w)


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
            v = speed * (0.4 if frac_left < 0.15 else 1.0)   # ease-out
            self._set_target(math.copysign(v, dist_m), 0.0)
            time.sleep(0.02)
        self.stop()

    def rotate_angle(self, angle_rad: float, rate: float):
        """Rotate in place until |yaw change| >= |angle_rad|."""
        rate = min(rate, self._args.max_angular)
        _, _, yaw0, _, _ = self._snapshot()
        target = abs(angle_rad)
        timeout = target / max(rate, 1e-3) * 3.0 + 4.0
        t0 = time.time()
        while True:
            _, _, yaw, _, _ = self._snapshot()
            d = abs(yaw - yaw0)
            if d >= target:
                break
            self._guard(t0, timeout, "turn segment")
            frac_left = (target - d) / target
            w = rate * (0.4 if frac_left < 0.15 else 1.0)    # ease-out
            self._set_target(0.0, math.copysign(w, angle_rad))
            time.sleep(0.02)
        self.stop()

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


def run_spin(node: CalibrationDriver, args: argparse.Namespace) -> int:
    total_deg = args.turns * 360.0
    print(f"\n=== SPIN test: rotate {args.turns:g} turns "
          f"({total_deg:.0f} deg) in place ===")
    print("Mark a reference on the chassis against a floor mark.")
    input("Press Enter to reset odom and spin (Ctrl-C aborts)... ")
    node.reset_odom()
    direction = 1.0 if not args.clockwise else -1.0
    node.rotate_angle(direction * math.radians(total_deg), args.angular_speed)
    odom_deg = abs(node.odom_yaw_deg())
    print(f"\nodom reported {odom_deg:.2f} deg of rotation.")
    print("Read the physical rotation: count full turns + the leftover "
          "angle the chassis mark stopped at.")
    physical = _prompt_float("Physical yaw (deg, e.g. 10 turns = 3600): ")
    if _HAVE_MATH:
        new = corrected_track_width(args.current_width, odom_deg, physical)
        ratio = odom_deg / physical
        print(f"\n  scale (odom/physical): {ratio:.5f}  "
              f"({(ratio - 1) * 100:+.2f} %)")
        print(f"  current TRACK_WIDTH_M : {args.current_width:.5f}")
        print(f"  -> set TRACK_WIDTH_M  : {new:.5f}f   "
              "(diff_drive.h, then rebuild+reflash)")
    else:
        print(f"  feed to: calibrate_constants.py track "
              f"--odom-deg {odom_deg:.2f} --physical-deg {physical:.2f}")
    return 0


def _drive_square(node: CalibrationDriver, args: argparse.Namespace,
                  clockwise: bool):
    sign = -1.0 if clockwise else 1.0
    for i in range(4):
        if args.step:
            input(f"    side {i + 1}/4: forward {args.side:.2f} m — Enter... ")
        node.drive_distance(args.side, args.linear_speed)
        time.sleep(0.3)
        if args.step:
            input(f"    corner {i + 1}/4: turn 90 deg — Enter... ")
        node.rotate_angle(sign * math.pi / 2.0, args.angular_speed)
        time.sleep(0.3)


def run_square(node: CalibrationDriver, args: argparse.Namespace) -> int:
    print(f"\n=== SQUARE test: {args.side:.2f} m sides, "
          f"{args.trials} runs each direction ===")
    print(f"Clear a ~{args.side + 1:.0f} x {args.side + 1:.0f} m area. "
          "Mark the start pose (position + heading).")
    closures: list[tuple[float, float, float]] = []
    for direction, cw in (("CW", True), ("CCW", False)):
        for run in range(1, args.trials + 1):
            print(f"\n-- {direction} run {run}/{args.trials} --")
            input("Place the robot on the start mark, Enter to drive... ")
            node.reset_odom()
            _drive_square(node, args, clockwise=cw)
            print("Square complete (odom believes it is back at the start).")
            print("Measure the PHYSICAL gap from the start mark:")
            dx = _prompt_float("  x error (m, +forward): ")
            dy = _prompt_float("  y error (m, +left): ")
            dth = _prompt_float("  heading error (deg, +CCW): ")
            closures.append((dx, dy, dth))

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

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)

    # common topic / safety options
    p.add_argument("--cmd-vel-topic", default="/cmd_vel")
    p.add_argument("--odom-topic", default="/odom")
    p.add_argument("--failsafe-topic", default="/failsafe/active")
    p.add_argument("--reset-service", default="/reset_odom")
    p.add_argument("--rate-hz", type=float, default=20.0,
                   help="cmd_vel stream rate (Hz, keep > 2 to beat the "
                        "500 ms firmware timeout)")
    p.add_argument("--linear-speed", type=float, default=0.3,
                   help="drive speed (m/s, default 0.3)")
    p.add_argument("--angular-speed", type=float, default=0.5,
                   help="turn rate (rad/s, default 0.5)")
    p.add_argument("--max-linear", type=float, default=0.6,
                   help="hard linear cap (m/s)")
    p.add_argument("--max-angular", type=float, default=1.0,
                   help="hard angular cap (rad/s)")

    sub = p.add_subparsers(dest="test", required=True)

    ps = sub.add_parser("straight", help="wheel-diameter test")
    ps.add_argument("--distance", type=float, default=5.0)
    ps.add_argument("--current-diameter", type=float,
                    default=DEFAULT_WHEEL_DIAMETER_M)
    ps.set_defaults(func=run_straight)

    pn = sub.add_parser("spin", help="track-width test")
    pn.add_argument("--turns", type=float, default=10.0)
    pn.add_argument("--clockwise", action="store_true")
    pn.add_argument("--current-width", type=float,
                    default=DEFAULT_TRACK_WIDTH_M)
    pn.set_defaults(func=run_spin)

    pq = sub.add_parser("square", help="closure + covariance test")
    pq.add_argument("--side", type=float, default=2.0)
    pq.add_argument("--trials", type=int, default=5,
                    help="runs per direction (CW + CCW)")
    pq.add_argument("--step", action="store_true",
                    help="pause for Enter before each side/corner "
                         "(cautious first run)")
    pq.set_defaults(func=run_square)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

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
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
