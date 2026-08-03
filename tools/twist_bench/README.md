# SpineSense Twist Bench

Part 2 starts here: two IMUs, a neutral still window, gyro bias calibration, and short-term relative axial twist.

> [!important] Algorithm status (2026-07-15)
> The authoritative coordinate contract is [[SpineSense-Multi-IMU-Coordinate-Calibration]]. Its general order is `q_segment = q_filter * C^-1`, `q_common = q_z(delta) * q_segment`, then `q_rel = inverse(q_common_parent) * q_common_child`. The frozen v1 executables below implement only the restricted gravity-alignment / no-explicit-common-heading case. Under the recorded same-direction placement protocol and a genuinely shared SFLP reference this is the identity-calibration special case; that shared-reference condition has not yet been closed by a rigid common-motion bench. The shadow scripts are robustness experiments, not the code that generated the frozen 13-participant results. Neutral tare defines change from a bout baseline; it is not a substitute for frame calibration.

Two pipelines live here:

- `twist_bench_v0.py`: gyro-only sanity check. Baseline only.
- `twist_bench_fusion.py`: the restricted v1 main line. Per-IMU 6D fusion (VQF, Madgwick fallback) -> gravity-only alignment -> relative quaternion under a shared-reference assumption -> neutral baseline -> swing-twist decomposition. Requires `numpy` and `pip install vqf`.
- `five_imu_fusion.py`: the frozen 5-IMU v1 body-chain line. Per-IMU 6D fusion -> gravity-only alignment -> root/adjacent relative quaternions under the same assumption -> neutral baseline -> regional swing/twist outputs.

Neither is an absolute yaw estimator; both report short-term tared relative twist only.

## T01-T15 participant placement (corrected 2026-07-13)

The participant cohort must use `config/placement_maps_v1.json`; socket order is not anatomy.
The current field-confirmed cohort binding is:

```text
IMU0 = sternum
IMU1 = sacrum / S1
IMU2 = lower
IMU3 = mid
IMU4 = upper
```

`layout_preset="t01"` is a quarantined historical mapping.  Old callers with a recognized raw
filename are resolved through the registry and stamped with placement provenance; an unknown raw
file or an unconfirmed trial fails closed.  T01 and T07 remain outside production validation/ML
because they lack usable independent paired-placement evidence.  Production code should resolve
`trial5` arguments with `placement_maps.resolve_placement(...).fusion_kwargs()` and must preserve
the mapping version and SHA256 in every derived artifact.

For corrected readouts, whole-back bend is `sacrum_to_upper`, whole-trunk twist is
`sacrum_to_sternum`, and posterior regional outputs are the three separate relations
`sacrum_to_lower`, `lower_to_mid`, and `mid_to_upper`.  `upper_to_sternum` is an anterior/posterior
thorax cross-check, not a fourth spinal region.  The old fixed-sign scalar chain-sum is invalid
under the corrected anatomy and is blocked by default.

## Five-IMU fusion (v1 body-chain)

Recommended body layout:

```text
pelvis  = IMU0   # sacrum / root
lower   = IMU1   # lower lumbar, around L4-L5
mid     = IMU2   # thoracolumbar, around T12-L1
upper   = IMU3   # upper thoracic, around T2-T4
sternum = IMU4   # sternum / manubrium
```

Run a synthetic chain check:

```powershell
python "SpineSense FYP\tools\twist_bench\five_imu_fusion.py" --demo
```

Run on a recorded 5-IMU log:

```powershell
python "SpineSense FYP\tools\twist_bench\five_imu_fusion.py" `
  --input "SpineSense FYP\tools\twist_bench\data\visual_test_YYYYMMDD_HHMMSS.log" `
  --filter vqf `
  --auto-markers
```

If all five IMUs are on the back and the board order is **U5 -> U1 from top to bottom**,
use the spine-chain preset:

```powershell
python "SpineSense FYP\tools\twist_bench\five_imu_fusion.py" `
  --input "SpineSense FYP\tools\twist_bench\data\visual_test_YYYYMMDD_HHMMSS.log" `
  --filter vqf `
  --auto-markers `
  --layout-preset spine5-u5-top
```

This assumes the board sockets map as `U1=IMU0`, `U2=IMU1`, `U3=IMU2`, `U4=IMU3`,
`U5=IMU4`, so the body chain is:

```text
top    = IMU4  # U5
high   = IMU3  # U4
mid    = IMU2  # U3
low    = IMU1  # U2
bottom = IMU0  # U1
```

For non-participant bench layouts, explicit overrides remain available:

```powershell
python "SpineSense FYP\tools\twist_bench\five_imu_fusion.py" `
  --input "...\trial.log" `
  --pelvis IMU4 --lower IMU3 --mid IMU2 --upper IMU1 --sternum IMU0
```

Do not use that free-form example for T01-T15 participant data; those runs require the registry
and provenance gate described above.

Outputs are written to `plots/<stem>_five_imu_vqf.{csv,svg}` and
`plots/<stem>_five_imu_vqf_summary.json`. The CSV contains the five segment
quaternions and per-relation twist/swing signals for:

```text
pelvis_to_lower
lower_to_mid
mid_to_upper
pelvis_to_upper
pelvis_to_sternum
sternum_to_upper_check
```

The sternum/upper number is a cross-check, not an automatic correction. This v1
pipeline reports short-term neutral-tared relative regional orientation only:
not 3D position, not absolute yaw, and not per-vertebra rotation.

## Static-hold drift test (5-IMU)

To answer "can we hold the static case?" the bench measures the drift of the tared twist while
the whole chain is held still. The true value is flat, so any slope is residual gyro-bias +
heading drift. Each relation summary now reports `static_drift_deg_per_min` (deg/min, linear fit
over the drift window; same convention as `batch_analyze.end_still_drift_deg_per_min`).

**Record (GUI, recommended).** Lay the chain flat on the table, hands off. In `twist.html` use the
**Static Hold** panel: pick 60 / 120 / 300 s and press *Static Hold (record)*. After a 3 s
hands-off countdown the bridge writes `data/visual_test_*.log` + `_markers.json` with a single
`static` phase.

**Record (CLI alternative).**

```powershell
python "SpineSense FYP\tools\twist_bench\capture_trial.py" --serial COM3 --baud 921600 --seconds 120 `
  --output "SpineSense FYP\tools\twist_bench\data\spine5_static_120s.log"
```

**Analyze (GUI recording, with markers).** `--auto-markers` reads the `static` phase and uses the
first 8 s as bias/tare and the remainder as the drift window:

```powershell
python "SpineSense FYP\tools\twist_bench\five_imu_fusion.py" `
  --input "SpineSense FYP\tools\twist_bench\data\visual_test_YYYYMMDD_HHMMSS.log" `
  --filter vqf --layout-preset spine5-u5-top --auto-markers
```

**Analyze (CLI recording, explicit windows).**

```powershell
python "SpineSense FYP\tools\twist_bench\five_imu_fusion.py" `
  --input "SpineSense FYP\tools\twist_bench\data\spine5_static_120s.log" `
  --filter vqf --layout-preset spine5-u5-top `
  --bias-start-s 0 --bias-end-s 8 --tare-start-s 0 --tare-end-s 8 `
  --drift-start-s 10 --drift-end-s 120
```

The console and `*_summary.json` then carry per-relation + `bottom_to_top` drift in deg/min.
Reference target: still-drift < 3 deg/min. Do **not** enable Auto Zero / closed-loop when measuring
drift — re-zeroing during stillness hides the very drift you are trying to quantify. A noisy hold
(hand microtremor on the structure) raises the bias-window gyro std and trips the motion warning;
hands off the table and cables for a clean window.

## Python environment

Install the tool dependencies in the Python environment you use for the bench scripts:

```powershell
python -m pip install -r "SpineSense FYP\tools\twist_bench\requirements.txt"
```

If `python` raises `ModuleNotFoundError: No module named 'numpy'`, you are using a different Python than the one with the project dependencies installed. Switch to the dependency-ready interpreter, or install the requirements above into your current interpreter.

## Fusion pipeline (v1 main line)

```text
raw acc/gyro per IMU
-> still-window gyro bias subtraction
-> VQF 6D fusion (offlineVQF)        # or --filter madgwick
-> gravity-only sensor-to-segment approximation  # up axis from still-window gravity; heading not identified
-> [no explicit per-sensor common-heading registration in frozen v1]
-> q_rel = inverse(q_seg_parent) * q_seg_child   # requires a shared filter reference as an assumption
-> neutral tare (quaternion average; defines change from baseline, not frame calibration)
-> swing-twist decomposition about segment Z
-> twist_deg, swing_deg, return-to-zero, drift
```

The canonical *mathematical* pipeline inserts full right-side `C^-1` and left-side `q_z(delta)` corrections before `q_rel`. `common_heading_shadow.py` and `functional_frame_shadow.py` explore estimators for those terms, but neither is promoted into `five_imu_fusion.py`. Do not describe an existing v1 output as if it had passed the new T0/PCA calibration.

Synthetic self-test with known truth (flipped child mounting included):

```powershell
python "SpineSense FYP\tools\twist_bench\twist_bench_fusion.py" --demo
```

Run on a captured log (writes `plots/<stem>_fusion_vqf.{csv,svg}` and `..._summary.json`, including a v0 baseline comparison):

```powershell
python "SpineSense FYP\tools\twist_bench\twist_bench_fusion.py" `
  --input "SpineSense FYP\tools\twist_bench\data\first_trial.log"
```

If the start of the log is not still, find a quiet segment and point the bias window at it explicitly:

```powershell
python "SpineSense FYP\tools\twist_bench\scan_still_windows.py" --input data\first_trial.log
python "SpineSense FYP\tools\twist_bench\twist_bench_fusion.py" --input data\first_trial.log `
  --bias-start-s 36 --bias-end-s 40 --tare-start-s 0 --tare-end-s 1.5
```

The tare window must cover the neutral pose; the bias window only needs stillness, any pose works.

Do not use the final return-to-zero segment as the tare window when evaluating return-to-zero. The script will warn if the tare window overlaps the return window because that makes the final error artificially near zero.

## Visual guided pair smoke test

The realtime twist viewer can guide and record a basic two-IMU smoke test without opening Arduino Serial Monitor or `capture_trial.py`.

Start the viewer:

```powershell
cd "SpineSense FYP\tools\imu_cube_viewer"
run_twist_viewer.cmd
```

Open:

```text
http://localhost:8765/twist.html
```

Use the **Pair Smoke Test** panel:

```text
Start Test
-> 3 s get ready        (not recorded)
-> 8 s hold neutral     (recorded, use 0-8 s for bias/tare)
-> 20 s move top IMU    (recorded)
-> 12 s return neutral  (recorded)
```

The bridge records raw serial lines while the test runs and writes:

```text
SpineSense FYP/tools/twist_bench/data/visual_test_YYYYMMDD_HHMMSS.log
SpineSense FYP/tools/twist_bench/data/visual_test_YYYYMMDD_HHMMSS_markers.json
```

Analyze the saved log:

```powershell
python "SpineSense FYP\tools\twist_bench\twist_bench_fusion.py" `
  --input "SpineSense FYP\tools\twist_bench\data\visual_test_YYYYMMDD_HHMMSS.log" `
  --filter vqf `
  --bias-start-s 0 `
  --bias-end-s 8 `
  --tare-start-s 0 `
  --tare-end-s 8
```

The realtime cube is still a gyro-only visual aid. The saved log is the source of truth for the offline fusion analysis.
The viewer's `Auto Zero` control is a live demo correction only: when the pair is still near neutral, it slowly re-tares
the displayed angle so the cube does not visibly drift away after returning to the start pose. It does not replace the
offline VQF + closed-loop analysis.

## Stream rate

The IMUs are configured at 120 Hz ODR in firmware. For the 5-IMU text stream, 115200 baud is not enough to carry
120 Hz frames reliably because each frame prints one line per IMU. The high-rate firmware and viewer therefore use:

```text
STREAM_BAUD = 921600
STREAM_INTERVAL_US = 8333   # target ~120 Hz
```

After flashing the high-rate firmware, start the bridge/viewer at the same baud rate. The batch analyzer reports the
actual median sample rate; use that measured value rather than assuming the target was reached.

## Closed-loop / return-to-neutral correction (v1.5, optional, never default)

The fusion script can optionally apply a `start_pose == end_pose` drift correction to the tared twist:

```powershell
python "SpineSense FYP\tools\twist_bench\twist_bench_fusion.py" `
  --input "...\visual_test_YYYYMMDD_HHMMSS.log" --filter vqf `
  --bias-start-s 0 --bias-end-s 8 --tare-start-s 0 --tare-end-s 8 `
  --end-still-start-s 28 --end-still-end-s 40 `
  --closed-loop linear   # none (default) | linear | piecewise
```

- `none` (default): output is byte-for-byte the v1 behavior. Nothing below runs.
- `linear`: one drift ramp from the start-still centroid to the end-still centroid.
- `piecewise`: correction held flat across each neutral hold, ramping only across the moving interval.

It **never overwrites** the VQF/Madgwick quaternions or the raw tared twist. It only adds, both to the CSV and the JSON summary:

```text
raw_fusion_twist_deg   closed_loop_twist_deg   correction_deg
raw_return_to_zero_deg   corrected_return_to_zero_deg
correction_method   start_still_quality   end_still_quality
```

where `correction_deg == raw_return_to_zero_deg - corrected_return_to_zero_deg`.

The correction is **refused** (not applied; corrected == raw) when a neutral hold is not trustworthy:

- gyro std in a still window exceeds `--still-gyro-std-max` (default 0.5 dps);
- VQF `rest_detected` fraction in a still window is below `--min-rest-fraction` (default 0.5, VQF only);
- the end-still window overlaps the tare window (circular return-to-zero);
- the still window has too few samples / anchors are not separable in time.

A low sample rate (`< --min-sample-rate-hz`, default 25) is a **soft** flag: the correction still applies but is marked `degraded`, because a coarse rate degrades but does not invalidate the boundary equality.

> [!warning] What closed-loop does and does NOT prove
> Closed-loop / return-to-neutral correction **only improves start/end self-consistency**. It can flatten
> residual drift/bias between the two neutral holds. It does **not** prove the accuracy of the mid-trial ROM,
> the peak twist, or the swing-twist decomposition: infinitely many mid-trial trajectories share the same two
> endpoints. Swing and `q_display` are left uncorrected on purpose (swing is a non-negative magnitude with no
> signed drift model). Absolute angular accuracy still requires mocap / optical tracking / mechanical-jig
> ground truth. Never report corrected return-to-zero as evidence of absolute accuracy.

## Batch analyzer (multi-trial stability)

To judge whether many trials are repeatable, run the batch analyzer over `data/`:

```powershell
python "SpineSense FYP\tools\twist_bench\batch_analyze.py" --filter vqf --closed-loop linear
```

It auto-discovers `visual_test_*.log` (+ `--also-trial-logs` for `twist_trial_*.log`), reads the matching
`*_markers.json` to set the neutral / return windows from the Pair-Smoke-Test phases, runs the fusion +
closed-loop pipeline per trial, and writes:

```text
plots/multi_trial_summary.csv
plots/multi_trial_summary.md
```

Each trial reports: sample rate, start/end gyro std, start/end VQF rest fraction, raw vs corrected
return-to-zero, candidate correction, twist range, swing max, end-still drift rate, correction method, and a
`valid` flag with the reason. The `.md` also gives an across-trial stability block (mean / std / abs-max of the
**raw** return-to-zero = bench repeatability) and repeats the self-consistency caveat above. A trial with a dirty
neutral hold is reported `valid=NO` and is **not** corrected.

By default the analyzer uses only the last 4 seconds of the `RETURN NEUTRAL` phase as the end-still window,
because that phase includes the movement back to neutral. Change this with `--end-still-tail-s 2` for a shorter
hold window, or `--end-still-tail-s 0` to use the full return phase.

## What v0 estimates

```text
relative_gyro = (gyro_child - child_bias) - (gyro_parent - parent_bias)
twist_deg += dot(relative_gyro, twist_axis) * dt
```

Use it for the first bench questions:

- Does `0 -> 30 -> 0` produce the expected sign and scale?
- Does the estimate return near zero?
- How bad is drift during a locked neutral hold?
- Does the chosen axis match the physical block axis?

## Run a synthetic check

From the vault root:

```powershell
python "SpineSense FYP\tools\twist_bench\twist_bench_v0.py" --demo
```

Outputs are written to:

```text
SpineSense FYP/tools/twist_bench/plots/
```

The demo creates:

- `demo_twist_v0.csv`
- `demo_twist_v0.svg`
- `demo_summary.json`

## Run on the current firmware serial log

For the five-IMU bus setup, upload this connectivity sketch first:

```text
SpineSense FYP/firmware/imu_i3c_connectivity_5/imu_i3c_connectivity_5.ino
```

Expected serial result:

```text
SUMMARY: 5/5 IMUs connected
RESULT: ALL_PASS
```

Then upload the streaming sketch:

```text
SpineSense FYP/firmware/imu_i3c_xyz/imu_i3c_xyz.ino
```

It assumes:

```text
IMU1 = parent/bottom/fixed block, TA0 = D4/PB5, dyn addr = 0x33
IMU2 = child/top/moving block, TA0 = D7/PA8, dyn addr = 0x34
```

Then capture one trial:

```powershell
python "SpineSense FYP\tools\twist_bench\capture_trial.py" --serial COM3 --seconds 40
```

The current firmware prints a long table:

```text
t_ms    imu    position    addr    ax_mg    ay_mg    az_mg    gx_dps    gy_dps    gz_dps
```

Save the serial output to `data/first_trial.log`, then run:

```powershell
python "SpineSense FYP\tools\twist_bench\twist_bench_v0.py" `
  --input "SpineSense FYP\tools\twist_bench\data\first_trial.log" `
  --parent IMU1 `
  --child IMU2 `
  --axis 0,0,1
```

If the sign is backwards, change the axis to `0,0,-1` after confirming the physical convention.

## Wide CSV format

The same script also accepts a parent/child wide CSV:

```text
t_us,
ax_parent,ay_parent,az_parent,gx_parent,gy_parent,gz_parent,
ax_child,ay_child,az_child,gx_child,gy_child,gz_child
```

Gyro values must be in `deg/s`. Time may be `t_s`, `t_ms`, or `t_us`.

## Useful options

```powershell
--bias-seconds 5
--tare-seconds 5
--axis 0,0,1
--drift-start-s 0
--drift-end-s 300
--return-window-seconds 2
```

For a 5 min locked neutral hold, use `--drift-start-s 0 --drift-end-s 300` over the static trial. For moving trials, the important number is usually return-to-zero.

## Visualize a trial

Generate a quick HTML plot with `0`, `+30`, `-30`, and cue timing markers:

```powershell
python "SpineSense FYP\tools\twist_bench\visualize_trial.py" `
  --input "SpineSense FYP\tools\twist_bench\data\first_trial.log"
```

The output is written to `plots/*_visual.html`.

## Bench protocol

Start every trial with 5-10 s still neutral:

1. Estimate parent and child gyro bias.
2. Tare twist to zero.
3. Run `0 -> 30 -> 0`.
4. Check sign, scale, and return-to-zero.
5. Then run static holds at `0, +/-15, +/-30, +/-45, +/-60, +/-90`.

Do not interpret this v0 output as absolute heading, long-term drift-free twist, or true vertebral rotation. It is the first engineering signal for the two-block bench.
