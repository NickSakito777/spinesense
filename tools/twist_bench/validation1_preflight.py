from __future__ import annotations

"""SpineSense Validation1 PREFLIGHT data-field check.

Run this on a short still desk capture (or any trial log) BEFORE trusting a capture for analysis, so
field-stopping problems are caught on the spot instead of after the pilot. It is intentionally
light: it only parses + does arithmetic, never runs VQF/fusion, and NEVER hard-fails -- it always
prints a PASS/FAIL table and writes a small json.

Checks (per the 2026-06-26 Validation1 pilot rules):
  raw accel             : every present IMU's gravity magnitude ~ 1000 mg
  raw gyro              : every present IMU has finite, non-constant gyro
  still gyro quality    : (still test only) per-IMU gyro std < threshold (ideal < 0.5 dps)
  5 IMU complete frames : IMU0..IMU4 all present and a high fraction of timestamps have all five
  SFLP quaternion       : NOT ENABLED unless qw/qx/qy/qz columns are present (today: not enabled)

Typical use:
  python validation1_preflight.py --input data/field_check_raw_001.log
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import twist_bench_v0 as v0


EXPECTED_IMUS = ["IMU0", "IMU1", "IMU2", "IMU3", "IMU4"]
GRAVITY_BAND_MG = (900.0, 1100.0)   # ~1 g with generous tolerance; KB baseline was 998-1019 mg
GYRO_GARBAGE_DPS = 200.0            # a still/slow trial should never sit here; flags dead/garbage gyro


def parse_records(text: str) -> list[v0.ImuSample]:
    records = v0.parse_serial_text(text)
    if not records:
        rows = v0.read_dict_rows(text)
        records = v0.parse_long_table_rows(rows)
    return records


def detect_sflp(text: str) -> bool:
    """True if the log carries on-chip SFLP quaternion columns, detected by a header line whose tokens
    include qw and qx. The raw-only firmware has no such header; the SFLP firmware prints
    '... gz_dps qw qx qy qz'."""
    for line in text.splitlines():
        toks = [t.lower() for t in line.split()]
        if "qw" in toks and "qx" in toks:
            return True
    return False


def quaternion_stats(text: str) -> dict[str, dict[str, object]]:
    """Per-IMU SFLP quaternion health from 14-column lines (... gz_dps qw qx qy qz). Validates that the
    on-chip game-rotation-vector is a unit quaternion (|q| ~ 1) and finite -- the automatable half of
    the bench check. 'Continuity under rotation' stays a manual visual check."""
    by_imu: dict[str, list[tuple[float, float, float, float]]] = {}
    for line in text.splitlines():
        parts = line.strip().split()
        if len(parts) < 14 or not parts[1].upper().startswith("IMU"):
            continue
        try:
            quat = (float(parts[10]), float(parts[11]), float(parts[12]), float(parts[13]))
        except ValueError:
            continue
        by_imu.setdefault(parts[1].upper(), []).append(quat)

    stats: dict[str, dict[str, object]] = {}
    for imu, quats in sorted(by_imu.items()):
        arr = np.array(quats, dtype=float)
        finite_rows = arr[np.all(np.isfinite(arr), axis=1)]
        norms = np.linalg.norm(finite_rows, axis=1) if len(finite_rows) else np.array([])
        nan_frac = 1.0 - (len(finite_rows) / len(arr)) if len(arr) else 1.0
        norm_mean = float(np.mean(norms)) if len(norms) else float("nan")
        ok = bool(len(norms) and 0.98 <= norm_mean <= 1.02 and nan_frac < 0.05)
        stats[imu] = {
            "n": len(arr),
            "norm_mean": round(norm_mean, 4) if np.isfinite(norm_mean) else None,
            "norm_min": round(float(np.min(norms)), 4) if len(norms) else None,
            "norm_max": round(float(np.max(norms)), 4) if len(norms) else None,
            "nan_fraction": round(nan_frac, 4),
            "quat_ok": ok,
        }
    return stats


def per_imu_stats(records: list[v0.ImuSample]) -> dict[str, dict[str, object]]:
    by_imu: dict[str, list[v0.ImuSample]] = {}
    for r in records:
        by_imu.setdefault(r.imu.upper(), []).append(r)

    stats: dict[str, dict[str, object]] = {}
    for imu, samples in sorted(by_imu.items()):
        acc = np.array([(s.ax_mg, s.ay_mg, s.az_mg) for s in samples], dtype=float)
        gyr = np.array([(s.gx_dps, s.gy_dps, s.gz_dps) for s in samples], dtype=float)
        grav = np.linalg.norm(acc, axis=1)
        gyro_std_max = float(np.max(gyr.std(axis=0))) if len(gyr) > 1 else float("nan")
        grav_mean = float(np.mean(grav))
        finite = bool(np.all(np.isfinite(acc)) and np.all(np.isfinite(gyr)))
        non_constant = bool(len(gyr) > 1 and np.any(gyr.std(axis=0) > 1e-9))
        stats[imu] = {
            "n_samples": len(samples),
            "gravity_mean_mg": round(grav_mean, 1),
            "gravity_min_mg": round(float(np.min(grav)), 1),
            "gravity_max_mg": round(float(np.max(grav)), 1),
            "gyro_std_max_dps": round(gyro_std_max, 3) if np.isfinite(gyro_std_max) else None,
            "gyro_absmean_dps": round(float(np.mean(np.abs(gyr))), 3),
            "accel_ok": GRAVITY_BAND_MG[0] <= grav_mean <= GRAVITY_BAND_MG[1] and finite,
            "gyro_ok": finite and non_constant and (not np.isfinite(gyro_std_max) or gyro_std_max < GYRO_GARBAGE_DPS),
        }
    return stats


def frame_completeness(records: list[v0.ImuSample], expected: list[str]) -> dict[str, object]:
    by_time: dict[float, set[str]] = {}
    for r in records:
        by_time.setdefault(r.t_s, set()).add(r.imu.upper())
    total = len(by_time)
    complete = sum(1 for imus in by_time.values() if set(expected).issubset(imus))
    times = sorted(by_time)
    rate = None
    if len(times) > 2:
        dts = np.diff(np.asarray(times, dtype=float))
        med = float(np.median(dts))
        rate = round(1.0 / med, 1) if med > 0 else None
    return {
        "total_timestamps": total,
        "complete_frames": complete,
        "completeness_fraction": round(complete / total, 4) if total else 0.0,
        "sample_rate_hz_median": rate,
    }


def verdict(flag: bool) -> str:
    return "PASS" if flag else "FAIL"


def newest_log() -> Path:
    """Newest data/*.log next to this script, so 'Run Python File' in VSCode (no CLI args) checks the
    capture you just took."""
    data_dir = Path(__file__).resolve().parent / "data"
    logs = sorted(data_dir.glob("*.log"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not logs:
        raise SystemExit(f"No --input given and no .log files in {data_dir}")
    return logs[0]


def main() -> int:
    parser = argparse.ArgumentParser(description="Validation1 preflight data-field check (raw log, no fusion).")
    parser.add_argument("--input", type=Path, help="Raw serial log. Default: newest data/*.log (so VSCode Run-with-no-args checks your latest capture).")
    parser.add_argument("--still-gyro-std-max", type=float, default=0.5, help="Still-test gyro std ceiling (dps).")
    parser.add_argument("--min-frame-completeness", type=float, default=0.90, help="Min fraction of all-5-present frames.")
    parser.add_argument("--moving", action="store_true", help="Trial involves motion: skip the still gyro-std check.")
    parser.add_argument("--out-dir", type=Path, help="Output dir for the json (default tools/twist_bench/plots).")
    args = parser.parse_args()
    if args.input is None:
        args.input = newest_log()
        print(f"(no --input given; using newest log: {args.input.name})")
    if not args.input.exists():
        raise SystemExit(f"input not found: {args.input}")
    text = args.input.read_text(encoding="utf-8", errors="replace")
    records = parse_records(text)
    if not records:
        raise SystemExit(f"No IMU records parsed from {args.input} (is this a raw serial log?)")

    imu_stats = per_imu_stats(records)
    present = sorted(imu_stats)
    frames = frame_completeness(records, EXPECTED_IMUS)
    sflp_present = detect_sflp(text)
    quat_stats = quaternion_stats(text) if sflp_present else {}
    sflp_offenders = [imu for imu, q in quat_stats.items() if not q["quat_ok"]]

    raw_accel_pass = all(s["accel_ok"] for s in imu_stats.values())
    raw_gyro_pass = all(s["gyro_ok"] for s in imu_stats.values())
    five_imu_pass = all(imu in imu_stats for imu in EXPECTED_IMUS) and \
        frames["completeness_fraction"] >= args.min_frame_completeness
    still_offenders = [imu for imu, s in imu_stats.items()
                       if s["gyro_std_max_dps"] is not None and s["gyro_std_max_dps"] >= args.still_gyro_std_max]
    still_quality = "skipped (--moving)" if args.moving else verdict(not still_offenders)
    if not sflp_present:
        sflp_status = "NOT ENABLED"
    elif not quat_stats:
        sflp_status = "PRESENT but no quaternion rows parsed (check column format)"
    elif sflp_offenders:
        sflp_status = f"PRESENT, norm BAD ({', '.join(sflp_offenders)})"
    else:
        sflp_status = "PRESENT, norm OK (|q|~1)"

    report = {
        "input": str(args.input),
        "present_imus": present,
        "missing_imus": [imu for imu in EXPECTED_IMUS if imu not in imu_stats],
        "frames": frames,
        "verdicts": {
            "raw_accel": verdict(raw_accel_pass),
            "raw_gyro": verdict(raw_gyro_pass),
            "five_imu_complete_frames": verdict(five_imu_pass),
            "still_gyro_quality": still_quality,
            "sflp_quaternion": sflp_status,
        },
        "per_imu": imu_stats,
        "per_imu_sflp": quat_stats,
        "thresholds": {
            "gravity_band_mg": list(GRAVITY_BAND_MG),
            "still_gyro_std_max_dps": args.still_gyro_std_max,
            "min_frame_completeness": args.min_frame_completeness,
            "quat_norm_band": [0.98, 1.02],
        },
        "note": ("Preflight only: confirms the capture is analyzable (raw accel/gyro present + sane, "
                 "5 IMUs streaming). It does NOT validate accuracy. SFLP 'NOT ENABLED' is fine on the "
                 "raw-only firmware (VQF runs offline from raw alone); with the SFLP firmware, a "
                 "'norm OK' check that |q|~1 is the automatable health test -- continuity-under-rotation "
                 "stays a manual visual check."),
    }

    out_dir = args.out_dir or (Path(__file__).resolve().parent / "plots")
    out_dir.mkdir(parents=True, exist_ok=True)
    out_json = out_dir / f"{args.input.stem}_preflight.json"
    with out_json.open("w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=2)
        fh.write("\n")

    print(f"PREFLIGHT  {args.input.name}")
    print(f"  present IMUs: {', '.join(present) or '(none)'}"
          + (f"   MISSING: {', '.join(report['missing_imus'])}" if report["missing_imus"] else ""))
    print(f"  rate ~{frames['sample_rate_hz_median']} Hz   "
          f"complete 5-IMU frames: {frames['complete_frames']}/{frames['total_timestamps']} "
          f"({frames['completeness_fraction']*100:.1f}%)")
    for imu in present:
        s = imu_stats[imu]
        print(f"    {imu}: grav {s['gravity_mean_mg']} mg, gyro std {s['gyro_std_max_dps']} dps, n={s['n_samples']}")
    print("  --- verdicts ---")
    print(f"  raw accel             : {report['verdicts']['raw_accel']}")
    print(f"  raw gyro              : {report['verdicts']['raw_gyro']}")
    print(f"  5 IMU complete frames : {report['verdicts']['five_imu_complete_frames']}")
    print(f"  still gyro quality    : {report['verdicts']['still_gyro_quality']}"
          + (f"  (offenders: {', '.join(still_offenders)})" if still_offenders and not args.moving else ""))
    print(f"  SFLP quaternion       : {report['verdicts']['sflp_quaternion']}")
    for imu in quat_stats:
        q = quat_stats[imu]
        print(f"    {imu} quat: |q| {q['norm_mean']} ({q['norm_min']}..{q['norm_max']}), nan {q['nan_fraction']}, n={q['n']}")
    print(f"wrote {out_json}")

    # Exit non-zero only on a hard data problem (accel/gyro/5-IMU), so it scripts cleanly; SFLP and
    # still-quality are advisory and never fail the run.
    return 0 if (raw_accel_pass and raw_gyro_pass and five_imu_pass) else 1


if __name__ == "__main__":
    raise SystemExit(main())
