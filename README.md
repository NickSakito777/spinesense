# SpineSense

Five-IMU garment-based trunk motion monitoring — firmware, fusion pipeline, and analysis code from an MSc dissertation in Rehabilitation Engineering & Assistive Technologies, UCL.

The system streams synchronised inertial data from five IMUs distributed along the trunk, reconstructs inter-segment orientation, and classifies pre-segmented trunk movements. It was developed and evaluated as a proof of concept for posture monitoring and movement classification, validated against optical motion capture in a controlled offline task.

**This repository contains code and method notes only. No participant data is included — see [Participant data](#participant-data-and-why-it-is-not-here).**

---

## What this is (and is not)

Built and evaluated:

- Firmware that brings up five ISM6HG256X IMUs on a single I3C bus with per-device dynamic addressing, and streams raw inertial data plus on-chip SFLP game-rotation quaternions at a nominal 120 Hz.
- An offline pipeline: serial-log parsing → clock alignment → per-segment orientation fusion → body-chain reconstruction → twist/bend decomposition.
- A validation chain against optical motion capture, including per-action empirical calibration and leave-one-subject-out generalisation.
- A movement classification branch over pre-segmented bouts, with a sensor-configuration sensitivity analysis (which subset of the five positions is actually needed).

Not built, and not claimed:

- No clinical diagnostic capability. Diagnosis was explicitly scoped out as future work.
- No real-time on-device estimation. All analysis is offline.
- No automatic segmentation. Classification operates on pre-segmented bouts; IMU-only segmentation feeding causal classification is proposed architecture, not implemented.
- No validated absolute joint-angle measurement. The optical comparison establishes a movement proxy with action-specific calibration, and documents where that calibration fails to transfer.

The dissertation is the authority on what the numbers mean. This repository is the authority on how they were computed — with one scoped exception: the classification branch ships as a reusable framework rather than as the study's experiment pipeline, for the reasons in [Classification](#classification-what-is-and-is-not-here).

---

## Hardware

| Item | Qty | Notes |
|---|---|---|
| STEVAL-MKI248KA (ISM6HG256X 6-axis IMU adapter board) | 5 | I3C-capable; SFLP game-rotation quaternion computed on-chip |
| NUCLEO-U385RG-Q | 1 | STM32U385RGTxQ; host MCU, I3C2 controller, USB CDC streaming |
| Passive fan-out board | 1 | Shared I3C bus + per-device TA0 line; no active components |
| Garment / strapping | 1 | Positions the five sensors along the trunk |

### Bus and addressing

All five devices share one I3C bus. Static address `0x6B` is common to every ISM6HG256X, so devices are individually enabled through their TA0 line and assigned a dynamic address one at a time via SETDASA:

| Slot | Nucleo pin | MCU pin | Dynamic address |
|---|---|---|---|
| IMU0 | D2 | PC8 | `0x32` |
| IMU1 | D4 | PB5 | `0x33` |
| IMU2 | D7 | PA8 | `0x34` |
| IMU3 | D8 | PC7 | `0x35` |
| IMU4 | D9 | PC6 | `0x36` |

I3C2 bus lines are `PB13` (D15, SCL) and `PB14` (D14, SDA). Pull-ups are enabled internally through `HAL_PWREx_EnableI3CPullUp` — no external pull-up resistors are fitted.

Streaming runs at 921600 baud over USB CDC. Accelerometer, gyroscope, and SFLP game ODR are all set to 120 Hz; the frame interval target is 8333 µs. **The analysis does not assume 120 Hz** — it uses the measured median rate from each log, because the achieved rate drifts from the target.

Sensor positions are named by role in the firmware (`PARENT_BOTTOM_FIXED`, `CHILD_TOP_MOVING`, etc.) because the same firmware served both bench tests and body-worn sessions. The body-chain mapping from slot to anatomical position is a per-session configuration, not a firmware constant — see [Placement mapping](#placement-mapping).

---

## Repository layout

```
firmware/
  imu_i3c_connectivity*/   I3C bring-up: TA0 gating, SETDASA dynamic addressing, bus checks
  imu_i3c_xyz/             Raw-only streaming (accel + gyro)
  imu_i3c_xyz_sflp/        Raw + on-chip SFLP quaternion streaming  ← used for data collection
tools/
  twist_bench/             Fusion pipeline, alignment, validation, analysis
    session_recipe.py      Reusable recipe pieces: loading, decomposition, segmentation, scoring
    dataset_adapter.py     Config-driven dataset access (path layout, manifests, quality)
    locked_track_a/        Classification framework: estimators, metrics, statistics
  imu_cube_viewer/         Browser-based live orientation viewer + serial bridge
技术笔记/                   Hardware bring-up notes (Chinese)
```

If you are here for a specific piece of the method, this is where each one lives:

| Layer | Where |
|---|---|
| **Sensor bring-up** — I3C enumeration, per-device TA0 gating, SETDASA addressing, SFLP/ODR register setup | `firmware/imu_i3c_connectivity*/`, `firmware/imu_i3c_xyz_sflp/` |
| **Frame assembly on the MCU** — five-device polling, single timestamp per frame, tab-separated emission at a fixed interval | `firmware/imu_i3c_xyz_sflp/imu_i3c_xyz_sflp.ino` |
| **Reference-frame reconstruction** — sensor frame → body-chain frame, heading resolution, functional calibration | `validation3_cluster_orientation.py`, `functional_frame_shadow.py`, `common_heading_shadow.py` |
| **Orientation fusion** — five-IMU body chain, twist/bend decomposition | `five_imu_fusion.py`, `twist_bench_fusion.py`, `signed_diagnostic.py` |
| **Mocap alignment** — clock fitting, drift estimation, sync quality gating | `mocap_adapter.py`, `sync_audit.py` |
| **Dataset access** — path layout, block manifests, per-session corrections, quality tiers | `dataset_adapter.py` (see [Pointing the analysis at your own data](#pointing-the-analysis-at-your-own-data)) |
| **Validation & analysis** — calibration rebuild, regional QC, sensor ablation, configuration comparison, permutation tests | `corrected_validation_rebuild.py`, `corrected_regional_qc.py`, `ablation_*.py`, `posthoc_label_permutation.py` |
| **Classification framework** — model registry, estimator construction, participant-first metrics, sign-flip and Holm-corrected statistics | `locked_track_a/core.py` (see [Classification](#classification-what-is-and-is-not-here)) |

Build artefacts, recorded sessions, and every analysis output directory are excluded from the repository.

One convention matters before reading the fusion code: the IMU's on-chip SFLP output is a *game-rotation* quaternion. These are 6-axis parts with no magnetometer, so absolute heading is unobservable — the body-chain reconstruction resolves relative heading between adjacent segments instead of assuming a shared global yaw. Everything downstream re-tares against a still window rather than trusting an absolute reference.

`技术笔记/` (Chinese) holds the hardware bring-up notes: the I3C feasibility investigation, multi-device bring-up including what failed, the pull-up and physical-layer work, the fan-out board brief, and why a 6-axis part was chosen over a 9-axis one. They cover what the pin table above cannot — the dead ends. The algorithm side is documented in the code itself and in this README rather than in separate notes.

---

## Getting started

### Firmware

Toolchain: **Arduino IDE 2.3.8** (or `arduino-cli`) with the STM32 core `STMicroelectronics:stm32@2.12.0`.

Board settings:

| Setting | Value |
|---|---|
| Board | Generic STM32U3 series |
| Board part number | Generic U385RGTxQ |
| USART support | Enabled (generic Serial) |
| Upload method | STM32CubeProgrammer (SWD) |

Each firmware directory carries its own `build_opt.h`, which enables the HAL I3C module (`HAL_I3C_MODULE_ENABLED`). The STM32duino core does not expose I3C through the Arduino API, so the firmware calls HAL directly.

Start with `imu_i3c_connectivity` to confirm all five devices enumerate before flashing the streaming firmware. On SETDASA failure the firmware prints which stage failed — check TA0 wiring, SDA/SCL, power, and CS high.

### Python

```bash
python3 -m venv .venv && source .venv/bin/activate    # Windows: .venv\Scripts\activate
pip install -r tools/twist_bench/requirements.txt
```

For numerically reproducing dissertation results, use the pinned set instead:

```bash
pip install -r requirements-lock.txt
```

Requires Python ≥ 3.9. Dissertation results were produced on 3.12.10; other minor versions are not guaranteed to give bit-identical numbers. Key dependencies: numpy, scipy, pandas, scikit-learn, matplotlib, joblib, threadpoolctl, [vqf](https://github.com/dlaidig/vqf) (orientation filter), pyserial.

### Capture

```bash
python tools/twist_bench/capture_trial.py --serial auto --out trial.log
```

`--serial auto` probes for the ST-Link VCP (USB VID `0x0483`). Pass an explicit port if you have several boards attached.

### Analysis

```bash
python tools/twist_bench/twist_bench_v0.py trial.log
python tools/twist_bench/batch_analyze.py <dir-of-logs>
```

`tools/twist_bench/README.md` documents the fusion pipeline, the coordinate contract, and the static-hold drift protocol in detail. `PILOT_RUNBOOK.md` and `CONFIG_COMPARE_RUNBOOK.md` document the collection protocol and the configuration-comparison run.

### Log format and sample data

The streaming firmware emits tab-separated frames, one row per IMU per frame:

```
t_ms  imu  position  addr  ax_mg  ay_mg  az_mg  gx_dps  gy_dps  gz_dps  [qw  qx  qy  qz]
```

The four quaternion columns are present only in the SFLP firmware. Note that the achieved frame rate is not the 120 Hz target — the samples below measure 125 Hz, which is why every downstream step uses the measured median rate.

Three ways to exercise the pipeline without hardware, in increasing order of realism:

**1. Synthetic — no data at all.** The parser and integrator have a built-in demo:

```bash
python tools/twist_bench/twist_bench_v0.py --demo
```

Generates a 0 → 30 → 0 → −30 → 0 degree trial at 120 Hz and returns to zero exactly. Useful for checking your install, not for judging real behaviour.

**2. A real protocol run** — `docs/sample_visual_test.log` (40.6 s, five IMUs, raw-only firmware):

```bash
python tools/twist_bench/twist_bench_v0.py --input docs/sample_visual_test.log
```

```
samples: 4873, duration_s: 40.599, median_rate_hz: 125.000
twist range deg: -13.859 to 20.980
return-to-zero estimate deg: -2.701
```

This is a visual-guided run: hold neutral 0–8 s, move 8–28 s, return to neutral 28–40 s. `docs/sample_visual_test_markers.json` carries the phase boundaries. The first 8 s is the still window the gyro bias estimate needs — **this is the single most important thing to get right in your own recordings.** Without a genuinely still opening window the bias estimate is wrong and the integrated twist drifts without bound; the −2.7° return-to-zero above is what a good bias estimate buys you.

**3. The SFLP frame format** — `docs/sample_sflp_frames.log` (10 s excerpt, 14 columns) documents what the on-chip quaternion columns look like in a real log.

All three sample files are bench recordings made by the author with no study participants involved, with timestamps re-based to zero.

**One parsing constraint worth knowing:** the MCU timestamp can wrap or reset within a long session. Logs with a timestamp reset must be split with `split_imu_log.py` before parsing — the parser raises rather than silently reordering samples, because a reset that gets absorbed into a time-keyed dictionary produces plausible-looking but wrong synchronisation.

### Placement mapping

The mapping from firmware slot to anatomical position is per-session, established during setup and verified from the data. The verification tooling (`placement_maps.py`, `test_placement_maps.py`) is included; **the session mapping file itself is not**, because it is keyed by session and therefore participant-linkable. Supply your own in the same schema — `placement_maps.py` documents the expected structure.

### Session recipes

Real sessions do not arrive clean. A log may be a hard concatenation of several recordings with timestamp resets between them; the sync clock may need fitting per segment; some protocol blocks may be covered by only part of the recording. The pipeline handles this with a *session recipe*: a declaration of the segment boundaries, the per-segment clock fit, and the mocap overlap window, followed by the standard processing over each segment.

`session_recipe.py` holds the reusable half of that:

| Group | Functions |
|---|---|
| Loading | `tolerant_load_five_streams` — five-stream loader that survives dropped rows and refuses logs with an unhandled timestamp reset |
| Decomposition | `local_twist`, `local_swing`, `qrel`, `seg_twists`, `SIGN`, `CHAIN` |
| Segmentation | `movement_bouts`, `peak_bouts` (deterministic detector), `find_reps`, `pre_mask` |
| Scoring | `score_block`, `rom`, `choose_bend`, `mark_bend_gain_corrected` |

`session_recipe_example.py` shows what a recipe declares around those functions, using the bench sample in `docs/` rather than a study session.

**You will have to write your own recipe.** Session constants — segment boundaries, clock fit `{a, b}`, `overlap_mocap_s`, `sync_corr` — are specific to one recording and meaningless for yours. The example shows the shape; the functions above are what you reuse. Fit your own clocks against your own sync signal and check `sync_corr` before trusting anything downstream.

The per-session recipes from the study itself are not included — they are the same logic with different constants, and the sessions they refer to are not public.

### Pointing the analysis at your own data

The analysis scripts need to know four things about a cohort: where each session's files live, which protocol blocks it contains, which of those blocks are trustworthy, and how to load and fuse it. None of that belongs in the analysis code — it describes one dataset, and hard-coding it turns a general method into a script that runs on exactly one cohort.

`dataset_adapter.py` holds the logic and reads the dataset from a config file:

```python
import dataset_adapter as da
da.configure("my_dataset.json")
for bid, block, res, a, b, bouts in da.subject_blocks("S01", tm, msig):
    ...
```

`docs/dataset_config.example.json` documents the schema: path templates, the session list, per-session block corrections (a window the automatic segmentation got wrong, a quality tier downgraded after review), and which sessions need the VQF fallback because their on-chip SFLP stream is unreliable.

The config used for the dissertation is not published, for the reason in [Participant data](#participant-data-and-why-it-is-not-here) — it is a description of human-subject recordings, down to which block of which session was judged unusable. The code that consumes it is here in full.

### Classification: what is and is not here

The classification branch is published as its **framework**, not as the study's experiment pipeline.

`locked_track_a/core.py` contains the parts that are reusable independently of any dataset:

- `build_model_registry()` / `make_estimator()` — the model registry and estimator construction, including a custom `WeightedShrinkageLDA` that stays stable under exact collinearity and p > n
- `compute_training_weights()` — the subject-balancing weight schemes (unweighted, subject-only, capped subject-class)
- `fixed_six_metrics()`, `participant_first_macro_f1()`, `participant_normalized_confusion()` — metrics computed participant-first rather than pooled over rows, which matters when participants contribute unequal numbers of bouts
- `select_config()` — tie-breaking toward the simplest configuration
- `exhaustive_sign_flip()`, `holm_adjust()`, `bootstrap_mean_ci()` — the paired statistics used for configuration comparisons

`locked_track_a/test_core.py` covers this with 23 synthetic tests. They need no data:

```bash
python -m unittest locked_track_a.test_core -v
```

**Not included**: the nested-CV runner, the reporting/figure stage, and the feature-construction code. These are tightly coupled to the participant-level dataset — subject-wise fold assignment, per-session block manifests, leave-one-subject-out splits keyed to the study cohort — and that dataset is not published. The full pipeline could not run from this repository regardless of whether the code were here.

If you want to reproduce the approach rather than the study, the framework above plus your own feature table is enough: build a table of one row per movement bout with a `subject` column, then drive `make_estimator` and `fit_predict_once` under whatever grouped cross-validation your design calls for.

---

## Participant data, and why it is not here

**No recorded sessions, derived features, model outputs, per-subject figures, or session mapping files are in this repository, and none will be added.**

Human-subject data collection for this project was approved by UCL under **Project ID 2758**. That approval, and the informed consent participants gave under it, covers analysis of the recordings for the dissertation. **It does not extend to publishing the dataset.** Releasing the recordings — or derived per-participant products from which individual measurement sequences can be reconstructed — would go beyond what participants agreed to. There is no version of this repository that includes them.

Excluded accordingly: raw and cleaned session recordings, per-bout and per-frame measurement tables, per-subject model predictions and cross-validation folds, per-subject plots, cached orientation series, and the trial-to-participant mapping file.

Aggregate results — cohort-level statistics, confusion matrices, interval estimates — are reported in the dissertation.

**What this means if you want to reproduce the results:** you cannot re-run the dissertation numbers, because the inputs are not public. What you can do is read and run the implementation, exercise the pipeline on the synthetic log or on your own recordings, and reuse the fusion, calibration, and classification code on a dataset you have your own ethical approval for. That is the intended use of this release.

If you are a UCL student continuing this project, the recordings are held separately under the original approval; ask the supervisor.

---

## Reading this alongside the dissertation

The dissertation's appendix cites implementations by path and by SHA-256. The release reorganised some of them, so `docs/paper-code-map.md` maps every cited location to where it is here, lists which hashes are unchanged, and states plainly what is not published and why.

`docs/reproduction-notes.md` holds the bench material the appendices point at: the address-assignment sequence and its fault-finding table, the build/flash and start-up acceptance checks, the coordinate-processing skeleton with its reproduction checks, a worked numerical example, the per-run clock mapping parameters, the full hyper-parameter grids, and the design record behind the addressing scheme.

The firmware is the strongest link: the source and `build_opt.h` in `firmware/imu_i3c_xyz_sflp/` hash byte-identical to the values the appendix records, so a reader can verify them directly against the paper.

## Citing

If this code supports published work, please cite the dissertation. `CITATION.cff` carries the machine-readable entry.

## License

MIT License — see [LICENSE](LICENSE).

Bundled third-party components and their licenses are listed in [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).
