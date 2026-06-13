# Odometry-Closure Calibration (Roadmap M1)

How to calibrate the diff-drive constants and odometry covariance on the
bench. This tunes the geometry constants in
[`firmware/main/diff_drive.h`](../firmware/main/diff_drive.h) and the odom
covariance diagonals in
[`firmware/main/uros_task.c`](../firmware/main/uros_task.c).

Tooling lives in [`../scripts/`](../scripts):

- `calibration_drive.py` — drives the robot through each test closed-loop
  on `/odom`, calls `/reset_odom` between trials, and prompts you for the
  physical measurement.
- `calibrate_constants.py` — pure math; converts a measurement into the
  corrected constant (also imported by the driver to print suggestions
  inline).

## The idea

Diff-drive dead reckoning has two dominant systematic error sources, both
compile-time constants:

| Constant | Wrong value shows up as |
|---|---|
| `WHEEL_DIAMETER_M` (with encoder counts/rev) | straight-line distance error |
| `TRACK_WIDTH_M` | heading / turn error |

The one rule: **ground truth is the tape measure / counted turns, never
odom.** It is fine to *close the loop on odom* (drive until odom reports a
target) as long as you *measure the result physically*. The error you are
solving for is a multiplicative scale factor, so the ratio is independent
of how far you drove.

## Prerequisites

- micro-ROS agent running; `/odom`, `/cmd_vel`, `/failsafe/active`, and
  the `/reset_odom` service all visible (`ros2 topic list`,
  `ros2 service list`).
- A ROS 2 environment with `rclpy` sourced.
- Open floor, tape measure, floor markers, robot e-stoppable (RC to
  MANUAL or kill `cmd_vel`).
- Drive slowly (the scripts default to 0.3 m/s — well under the
  `MAX_WHEEL_RPM` cap) for clean tracking.

> No odom reset over ROS? Older firmware lacked the `reset_odom` service;
> then the only way to zero the pose was a reboot. The service
> (`std_srvs/Trigger`) removes that — see
> [ADR-0005](adr/0005-odometry-computation.md).

## Step 1 — Wheel diameter (straight line)

Isolate translation. Do **not** start with the square — it conflates the
two errors.

```bash
./scripts/calibration_drive.py straight --distance 5.0
```

The tool resets odom, drives until odom reads 5 m, stops, then asks for
the tape-measured distance. It prints the corrected `WHEEL_DIAMETER_M`.

Edit [`diff_drive.h`](../firmware/main/diff_drive.h), rebuild, reflash:

```bash
cd firmware && idf.py build flash
```

(Counts/rev is read live from the drive at bring-up, so wheel diameter is
the free variable here.)

## Step 2 — Track width (in-place spin)

Isolate rotation.

```bash
./scripts/calibration_drive.py spin --turns 10
```

Resets odom, spins until odom reads 3600°, stops. Read the **physical**
rotation — count full turns plus the leftover angle the chassis mark
stopped at — and enter it. The tool prints the corrected `TRACK_WIDTH_M`.
Edit, rebuild, reflash as above.

> Closing on odom stops the robot at an awkward partial angle. If you
> prefer a clean integer ground truth, spin until the chassis mark
> visually lines up after N turns, then read odom and pass the numbers to
> `calibrate_constants.py track` directly.

## Step 3 — Square drive (validate + covariance)

With both constants corrected, confirm they work together and measure the
residual. This is the UMBmark benchmark.

```bash
./scripts/calibration_drive.py square --side 2.0 --trials 5
# cautious first run: add --step to confirm before each side/corner
```

For each run the tool resets odom and drives a square (4 sides + 4
turns), ending where odom *believes* it is back at the start. You then
tape-measure the **physical** gap from the start mark (x, y, heading).
It runs `--trials` times clockwise and counter-clockwise.

Interpreting the closure errors (printed at the end):

- Residual that **flips sign** CW vs CCW → wheel-diameter / wheel-balance
  error → redo Step 1.
- Residual **consistent** in both directions → track-width error → redo
  Step 2.
- The **run-to-run spread** (variance) is your random odom error. The
  tool prints suggested `pose.covariance` diagonals — paste them into the
  covariance block of [`uros_task.c`](../firmware/main/uros_task.c) (they
  currently hold placeholder "starting points").

## Done criteria

- Straight-line and spin residuals within a few % after one or two
  iterations.
- Square closure small and **not direction-biased** across CW/CCW runs.
- Covariance diagonals in `uros_task` replaced with measured values.

## Offline math (no robot)

If you already have measurements, skip the driver:

```bash
./scripts/calibrate_constants.py wheel --odom 5.00 --tape 4.82
./scripts/calibrate_constants.py track --odom-deg 3600 --physical-deg 3540
./scripts/calibrate_constants.py cov --csv closures.csv   # x,y,theta_deg per line
```
