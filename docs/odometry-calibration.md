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

> **`reset_odom` times out (firmware log shows `pose zeroed`)?** The request
> reached the firmware but the *reply* was dropped. The micro-ROS agent speaks
> Fast DDS, and service reply correlation does not work cross-vendor (a Cyclone
> DDS client ↔ Fast DDS agent) even though topics do. `calibration_drive.py`
> works around this by defaulting `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (set
> `CALIB_RMW=keep` to opt out). For manual calls, match the agent's vendor:
> `RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 service call /reset_odom std_srvs/srv/Trigger`.

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

Isolate rotation. Mark **two** datums on the floor (plumb-bob / down-laser):

- `A` = the drive-axle centre (the spin centre),
- `B` = a second centreline point, e.g. the front, a fixed body distance
  `L = |A B|` ahead of `A`.

Measure `L` once with a tape.

```bash
./scripts/calibration_drive.py spin --turns 10 --baseline 0.40
```

Resets odom, spins until odom reads 3600°, stops. Instead of reading the
leftover angle with a protractor, **measure it from a distance**: an
in-place spin turns about `A`, so the front datum `B` rides a circle of
radius `L` and the chord between its start and end marks gives the angle:

$$\theta_\text{leftover} = 2\arcsin\!\left(\frac{|B\,B'|}{2L}\right)$$

Re-mark the front datum as `B'` after the spin, then enter:

1. the **full turns** you counted (the mark passing the start; Enter
   accepts the commanded count),
2. the baseline `L` (or pass `--baseline`),
3. the chord `|B → B'|` (tape),
4. whether the front mark stopped **past** or **short** of the start in
   the spin direction (one glance — fixes the sign).

The tool computes the physical total (`turns × 360° + leftover`) and prints
the corrected `TRACK_WIDTH_M`. Edit, rebuild, reflash as above.

> Keep the leftover under 180° (stop within half a turn of an integer
> count) so the chord is unambiguous. Offline equivalent:
> `calibrate_constants.py spin-marks --odom-deg 3600 --turns 10
> --baseline 0.40 --chord 0.098 [--short]`.

> Prefer a clean integer ground truth instead? Spin until the chassis mark
> visually lines up after N turns, read odom, and pass the numbers to
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
capture the **physical** closure pose from tape distances between floor
marks. It runs `--trials` times clockwise and counter-clockwise.

### Measuring the closure (four distances, no protractor)

The closure is a **pose** error, not just a distance — a single point only
captures x and y and silently drops the yaw error (the robot can sit
dead-on the start point while rotated several degrees). Reading that yaw
with a protractor on the floor is awkward, so capture the whole pose from
**distances**, which a tape reads quickly and precisely.

Mark **two** datums (same as the spin test):

1. **Position datum** `A` — the drive-axle centre. Plumb-bob / down-laser
   it and mark the floor.
2. **Heading datum** `B` — a second centreline point (e.g. the front), a
   fixed body distance `L = |A B|` ahead of `A`. The line `A → B` is the
   start heading. Measure `L` once.

After the square, re-mark the same two points as `A'` and `B'` and tape
these **four** distances:

```
              |A → A'|   |B → A'|     (locate A')
              |A → B'|   |B → B'|     (locate B')
```

Four mutual distances fix the configuration up to a mirror across the
start heading line, so add **one glance**: did the end pose fall to the
**left** or **right** of the start heading? (For a few-cm closure offset
this is obvious — unlike the yaw angle, which is why we stopped reading
it directly.) That one bit resolves the sign of both `y` and `heading`.

```
start:                  end (exaggerated):
   B                         B'
   |  ← heading                \   ← rotated by heading error
   A  ← position datum      A' ·····→ offset (x fwd, y left) from A
```

The tool solves the rigid transform and prints the signed closure in the
start frame (`x` = +forward, `y` = +left, `heading` = +CCW). It also
prints a tape-consistency check `||A'B'| − L|`; if that is large you
mis-read a distance — re-measure. Offline equivalent:
`calibrate_constants.py closure -L 0.40 --aa .. --bb .. --ab .. --ba ..
--side left`.

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

# distance-only variants (no protractor):
#   spin leftover from the front-datum chord -> TRACK_WIDTH_M
./scripts/calibrate_constants.py spin-marks --odom-deg 3600 --turns 10 \
    --baseline 0.40 --chord 0.098
#   one square closure from four floor-mark distances -> x,y,theta row
./scripts/calibrate_constants.py closure -L 0.40 \
    --aa 0.21 --ba 0.43 --ab 0.45 --bb 0.19 --side left
```
