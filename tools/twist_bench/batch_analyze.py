from __future__ import annotations

"""Batch analyzer for SpineSense twist trials.

Runs the existing fusion pipeline + optional closed-loop return-to-neutral correction over
every matching log in data/, then writes multi_trial_summary.{csv,md}. The point is to judge,
across many trials, whether the bench is REPEATABLE -- not to claim absolute accuracy.

Closed-loop here is self-consistency only: it can flatten residual start/end drift, but it does
NOT validate mid-trial ROM, peak twist, or the swing-twist decomposition. Trials whose neutral
holds are not still (or that overlap the tare window) are reported as invalid and NOT corrected.
"""

import argparse
import csv
import json
from datetime import datetime
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import twist_bench_fusion as fusion


def find_phase(markers: dict, *names: str) -> dict | None:
    for ph in markers.get("phases", []):
        if ph.get("name") in names or ph.get("label") in names:
            return ph
    return None


def derive_windows(markers: dict | None, end_still_tail_s: float | None = 4.0) -> dict[str, object]:
    """Map Pair-Smoke-Test marker phases to bias/tare/end-still window overrides."""
    if not markers:
        return {}
    out: dict[str, object] = {}
    neutral = find_phase(markers, "neutral", "HOLD NEUTRAL")
    ret = find_phase(markers, "return_neutral", "RETURN NEUTRAL")
    if neutral is not None:
        out["bias_start_s"] = float(neutral["start_s"])
        out["bias_end_s"] = float(neutral["end_s"])
        out["tare_start_s"] = float(neutral["start_s"])
        out["tare_end_s"] = float(neutral["end_s"])
    if ret is not None:
        ret_start = float(ret["start_s"])
        ret_end = float(ret["end_s"])
        if end_still_tail_s is not None and end_still_tail_s > 0:
            out["end_still_start_s"] = max(ret_start, ret_end - float(end_still_tail_s))
        else:
            out["end_still_start_s"] = ret_start
        out["end_still_end_s"] = ret_end
    return out


def end_still_drift_deg_per_min(t: np.ndarray, twist: np.ndarray, mask: np.ndarray) -> float | None:
    """Residual drift slope while holding the END neutral pose (true value should be flat)."""
    if int(np.count_nonzero(mask)) < 5:
        return None
    slope = float(np.polyfit(t[mask], twist[mask], 1)[0])
    return slope * 60.0


FIELDS = [
    "trial",
    "has_markers",
    "filter",
    "sample_rate_hz",
    "start_gyro_std_dps",
    "end_gyro_std_dps",
    "start_rest_frac",
    "end_rest_frac",
    "raw_return_to_zero_deg",
    "corrected_return_to_zero_deg",
    "candidate_correction_deg",
    "twist_min_deg",
    "twist_max_deg",
    "twist_range_deg",
    "swing_max_deg",
    "end_still_drift_deg_per_min",
    "correction_method",
    "valid",
    "reason",
]


def analyze_one(log_path: Path, common: dict[str, object]) -> dict[str, object]:
    stem = log_path.stem
    markers_path = log_path.with_name(stem + "_markers.json")
    markers = None
    if markers_path.exists():
        try:
            markers = json.loads(markers_path.read_text(encoding="utf-8"))
        except Exception:
            markers = None

    overrides = dict(common)
    overrides.update(derive_windows(markers, common.get("end_still_tail_s")))
    args = fusion.make_args(**overrides)

    row: dict[str, object] = {field: None for field in FIELDS}
    row["trial"] = stem
    row["has_markers"] = markers_path.exists()
    row["filter"] = args.filter
    row["correction_method"] = "n/a"

    try:
        pairs = fusion.load_fusion_pairs(log_path, args.parent, args.child)
        result = fusion.run_pipeline(pairs, args)
        result = fusion.apply_closed_loop(result, pairs, args)
    except SystemExit as exc:
        row["valid"] = False
        row["reason"] = f"pipeline refused to run: {exc}"
        return row
    except Exception as exc:  # noqa: BLE001 - batch must survive a single bad trial
        row["valid"] = False
        row["reason"] = f"exception: {type(exc).__name__}: {exc}"
        return row

    s = result.summary
    cl = s.get("closed_loop")
    if cl:
        start_q = cl.get("start_still_quality", {})
        end_q = cl.get("end_still_quality", {})
    else:
        start_mask = result.tare_mask if result.tare_mask is not None else (result.t_s <= args.tare_seconds)
        end_mask_for_quality = fusion.resolve_end_still_mask(result.t_s, args)
        start_q = fusion.assess_still_quality(
            "start_still", start_mask, result.t_s, pairs.parent_gyr_dps, pairs.child_gyr_dps,
            result.parent_rest, result.child_rest, args,
        )
        end_q = fusion.assess_still_quality(
            "end_still", end_mask_for_quality, result.t_s, pairs.parent_gyr_dps, pairs.child_gyr_dps,
            result.parent_rest, result.child_rest, args,
        )
    end_mask = fusion.resolve_end_still_mask(result.t_s, args)
    drift = end_still_drift_deg_per_min(result.t_s, result.twist_deg, end_mask)

    def r(value: object, digits: int = 2) -> object:
        return round(float(value), digits) if isinstance(value, (int, float)) else value

    row.update(
        {
            "sample_rate_hz": r(s.get("sample_rate_hz_median")),
            "start_gyro_std_dps": start_q.get("gyro_std_max_dps"),
            "end_gyro_std_dps": end_q.get("gyro_std_max_dps"),
            "start_rest_frac": start_q.get("rest_detected_fraction"),
            "end_rest_frac": end_q.get("rest_detected_fraction"),
            "raw_return_to_zero_deg": r(
                cl.get("raw_return_to_zero_deg") if cl else s.get("return_to_zero_estimate_deg")
            ),
            "corrected_return_to_zero_deg": r(
                cl.get("corrected_return_to_zero_deg") if cl else s.get("return_to_zero_estimate_deg")
            ),
            "candidate_correction_deg": r(cl.get("candidate_correction_deg") if cl else None),
            "twist_min_deg": r(s.get("twist_min_deg")),
            "twist_max_deg": r(s.get("twist_max_deg")),
            "twist_range_deg": r(s.get("twist_max_deg", 0.0) - s.get("twist_min_deg", 0.0)),
            "swing_max_deg": r(s.get("swing_max_deg")),
            "end_still_drift_deg_per_min": (round(drift, 2) if drift is not None else None),
            "correction_method": cl.get("correction_method", "none") if cl else "none",
            "valid": bool(cl.get("applied", False)) if cl else False,
            "reason": ("; ".join(cl.get("reasons", [])) or "ok") if cl else "closed-loop not requested",
        }
    )
    return row


def aggregate(rows: list[dict[str, object]]) -> dict[str, object]:
    def col(name: str) -> list[float]:
        return [float(r[name]) for r in rows if isinstance(r.get(name), (int, float))]

    raw_rtz = col("raw_return_to_zero_deg")
    swing = col("swing_max_deg")
    rates = col("sample_rate_hz")
    valid = [r for r in rows if r.get("valid")]
    return {
        "n_trials": len(rows),
        "n_valid": len(valid),
        "raw_rtz_mean": (round(float(np.mean(raw_rtz)), 2) if raw_rtz else None),
        "raw_rtz_std": (round(float(np.std(raw_rtz)), 2) if raw_rtz else None),
        "raw_rtz_absmax": (round(float(np.max(np.abs(raw_rtz))), 2) if raw_rtz else None),
        "swing_max_mean": (round(float(np.mean(swing)), 2) if swing else None),
        "rate_min": (round(float(np.min(rates)), 2) if rates else None),
        "rate_max": (round(float(np.max(rates)), 2) if rates else None),
    }


def write_csv(rows: list[dict[str, object]], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: ("" if row.get(k) is None else row.get(k)) for k in FIELDS})


def md_cell(value: object) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "yes" if value else "NO"
    return str(value)


def write_md(rows: list[dict[str, object]], agg: dict[str, object], args: argparse.Namespace, path: Path) -> None:
    stamp = datetime.now().strftime("%Y-%m-%d %H:%M")
    lines: list[str] = []
    lines.append("# SpineSense multi-trial summary")
    lines.append("")
    lines.append(f"- generated: {stamp}")
    lines.append(f"- data dir: `{args.data_dir}`  pattern: `{args.pattern}`")
    lines.append(f"- filter: `{args.filter}`  closed-loop: `{args.closed_loop}`")
    lines.append(
        f"- gates: still_gyro_std_max={args.still_gyro_std_max} dps, "
        f"min_rest_fraction={args.min_rest_fraction}, min_sample_rate_hz={args.min_sample_rate_hz}"
    )
    lines.append("")
    lines.append("> [!warning] What this does and does NOT prove")
    lines.append("> Closed-loop / return-to-neutral correction only improves **start/end self-consistency**.")
    lines.append("> It does **not** prove the accuracy of mid-trial ROM, peak twist, or the swing-twist")
    lines.append("> decomposition. `valid=yes` means the correction was trustworthy enough to apply on a clean")
    lines.append("> neutral hold; it is **not** a claim of absolute angular accuracy. Absolute validation still")
    lines.append("> requires mocap / optical tracking / mechanical jig.")
    lines.append("")
    lines.append("## Stability across trials (raw, uncorrected)")
    lines.append("")
    lines.append(f"- trials: {agg['n_trials']}  (correction-valid: {agg['n_valid']})")
    lines.append(
        f"- raw return-to-zero: mean {agg['raw_rtz_mean']}deg, std {agg['raw_rtz_std']}deg, "
        f"abs-max {agg['raw_rtz_absmax']}deg  <- spread = bench repeatability"
    )
    lines.append(f"- swing max (mean across trials): {agg['swing_max_mean']}deg  <- large => motion not pure twist")
    lines.append(f"- sample rate range: {agg['rate_min']} .. {agg['rate_max']} Hz")
    lines.append("")
    lines.append("## Per-trial")
    lines.append("")
    header = [
        "trial", "rate Hz", "start std", "end std", "raw RTZ", "corr RTZ",
        "cand corr", "twist range", "swing max", "drift /min", "method", "valid", "reason",
    ]
    keys = [
        "trial", "sample_rate_hz", "start_gyro_std_dps", "end_gyro_std_dps",
        "raw_return_to_zero_deg", "corrected_return_to_zero_deg", "candidate_correction_deg",
        "twist_range_deg", "swing_max_deg", "end_still_drift_deg_per_min",
        "correction_method", "valid", "reason",
    ]
    lines.append("| " + " | ".join(header) + " |")
    lines.append("|" + "|".join(["---"] * len(header)) + "|")
    for row in rows:
        lines.append("| " + " | ".join(md_cell(row.get(k)) for k in keys) + " |")
    lines.append("")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Batch-analyze SpineSense twist trials into a multi-trial summary.")
    parser.add_argument("--data-dir", type=Path, default=here / "data")
    parser.add_argument("--pattern", default="visual_test_*.log", help="Glob for logs. Default visual_test_*.log.")
    parser.add_argument("--also-trial-logs", action="store_true", help="Also include twist_trial_*.log files.")
    parser.add_argument("--out-dir", type=Path, default=here / "plots")
    parser.add_argument("--filter", choices=("vqf", "madgwick"), default="vqf")
    parser.add_argument("--closed-loop", choices=("none", "linear", "piecewise"), default="linear")
    parser.add_argument("--still-gyro-std-max", type=float, default=0.5)
    parser.add_argument("--min-rest-fraction", type=float, default=0.5)
    parser.add_argument("--min-sample-rate-hz", type=float, default=25.0)
    parser.add_argument(
        "--end-still-tail-s", type=float, default=4.0,
        help=(
            "Use only the last N seconds of the RETURN NEUTRAL phase as end-still. "
            "The full return phase includes the movement back to neutral. Use 0 to use the full phase."
        ),
    )
    parser.add_argument("--parent", default="IMU1")
    parser.add_argument("--child", default="IMU2")
    args = parser.parse_args()

    logs: list[Path] = sorted(args.data_dir.glob(args.pattern))
    if args.also_trial_logs:
        logs += sorted(args.data_dir.glob("twist_trial_*.log"))
    logs = [p for p in dict.fromkeys(logs)]  # de-dup, keep order
    if not logs:
        raise SystemExit(f"No logs matched {args.pattern} under {args.data_dir}")

    common = {
        "filter": args.filter,
        "closed_loop": args.closed_loop,
        "still_gyro_std_max": args.still_gyro_std_max,
        "min_rest_fraction": args.min_rest_fraction,
        "min_sample_rate_hz": args.min_sample_rate_hz,
        "end_still_tail_s": args.end_still_tail_s,
        "parent": args.parent,
        "child": args.child,
        "no_v0_baseline": True,
    }

    rows = [analyze_one(log, common) for log in logs]
    agg = aggregate(rows)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    out_csv = args.out_dir / "multi_trial_summary.csv"
    out_md = args.out_dir / "multi_trial_summary.md"
    write_csv(rows, out_csv)
    write_md(rows, agg, args, out_md)

    print(f"analyzed {len(rows)} trial(s); correction-valid: {agg['n_valid']}")
    for row in rows:
        flag = "ok " if row.get("valid") else "INV"
        print(f"  [{flag}] {row['trial']}  rawRTZ={row.get('raw_return_to_zero_deg')}  "
              f"swing={row.get('swing_max_deg')}  method={row.get('correction_method')}  {row.get('reason')}")
    print(f"wrote {out_csv}")
    print(f"wrote {out_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
