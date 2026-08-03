from __future__ import annotations

"""Extract zero-lag IMU/MoCap samples per protocol bout from a reviewed block manifest.

This is the bridge between cleaning/orientation and cleaned_accuracy.py. It supports
the current common readouts (global/local sacrum-to-upper swing and per-bout local
upper-to-sternum twist). Trial-specific ROM-only probes remain separate.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import five_imu_fusion as fiv
import twist_bench_fusion as quat


SIGNAL_BY_LABEL = {
    "flexion": "flex_deg", "extension": "flex_deg",
    "left_bend": "lateral_deg", "right_bend": "lateral_deg",
    "left_twist": "axial_deg", "right_twist": "axial_deg",
}
EXPECTED_LABELS = {
    "B1": "flexion", "B2": "extension", "B3": "left_bend",
    "B4": "right_bend", "B5": "left_twist", "B6": "right_twist",
}


def moving_mean(values: np.ndarray, count: int) -> np.ndarray:
    count = max(3, int(count))
    return np.convolve(values, np.ones(count) / count, mode="same")


def peak_bouts(time_s, signal, window, sign, min_dist_s=6.0, base_s=24.0, fraction=0.35, min_duration_s=0.8):
    dt = float(np.median(np.diff(time_s)))
    delta = signal - moving_mean(signal, int(base_s / dt))
    directed = np.abs(delta) if sign == 0 else sign * delta
    mask = (time_s >= window[0]) & (time_s <= window[1])
    indices = np.where(mask)[0]
    if len(indices) < 3:
        return []
    values = directed.copy(); values[~mask] = -np.inf
    local = np.where((values[1:-1] > values[:-2]) & (values[1:-1] >= values[2:]))[0] + 1
    threshold = max(2.0, 0.35 * float(np.nanmax(directed[mask])))
    candidates = [index for index in local if mask[index] and directed[index] >= threshold]
    selected: list[int] = []
    for index in sorted(candidates, key=lambda item: directed[item], reverse=True):
        if all(abs(float(time_s[index] - time_s[other])) >= min_dist_s for other in selected):
            selected.append(index)
    bouts = []
    for index in sorted(selected):
        edge = fraction * directed[index]
        start = end = index
        while start > indices[0] and directed[start] > edge:
            start -= 1
        while end < indices[-1] and directed[end] > edge:
            end += 1
        if float(time_s[end] - time_s[start]) >= min_duration_s:
            bouts.append((float(time_s[start]), float(time_s[end])))
    return bouts


def load_reference(path: Path):
    with path.open(newline="", encoding="utf-8-sig") as fh:
        rows = list(csv.DictReader(fh))
    required = {"time_s", "flex_deg", "lateral_deg", "axial_deg"}
    if not rows or not required <= set(rows[0]):
        raise SystemExit(f"Reference CSV must contain {sorted(required)}")
    return {key: np.asarray([float(row[key]) for row in rows]) for key in required}


def pre_mask(time_s: np.ndarray, a: float, b: float, bout_start: float) -> np.ndarray:
    mask = (time_s >= a * (bout_start - 1.2) + b) & (time_s <= a * (bout_start - 0.2) + b)
    if int(np.count_nonzero(mask)) < 3:
        mask = time_s <= min(8.0, float(time_s[-1]))
    return mask


def tared_relation_series(result, parent: str, child: str, axis_mode: str, a: float, b: float, bout_start: float):
    q_rel = quat.qmul(quat.qconj(result.sensors[parent].q_segment), result.sensors[child].q_segment)
    q0 = quat.quat_average(q_rel[pre_mask(result.t_s, a, b, bout_start)])
    q_tared = quat.qmul(quat.qconj(q0)[None, :], q_rel)
    twist, swing = quat.swing_twist_deg(q_tared, quat.SEGMENT_TWIST_AXIS)
    return -quat.unwrap_deg(twist) if axis_mode == "twist" else swing


def series_for(result, readout: str, a: float, b: float, bout_start: float):
    if readout == "sacrum_to_upper_yawmasked_swing":
        return result.relations["sacrum_to_upper"].swing_deg
    if readout == "sacrum_to_upper_local_swing":
        return tared_relation_series(result, "sacrum", "upper", "swing", a, b, bout_start)
    if readout == "upper_to_sternum_local_retare_twist":
        return tared_relation_series(result, "upper", "sternum", "twist", a, b, bout_start)
    raise ValueError(f"Unsupported common readout {readout!r}; use a versioned trial-specific probe")


def run_segment(path: Path, config: dict[str, object]):
    placement = config.get("placement_map", {})
    overrides = {str(role): str(imu) for imu, role in placement.items()}
    args = fiv.make_args(
        layout_preset=str(config.get("layout_preset", "t01")),
        filter=str(config.get("filter", "sflp")),
        **overrides,
    )
    return fiv.run_pipeline(path, args)


def resolve(path: str) -> Path:
    value = Path(path)
    return value if value.is_absolute() else Path.cwd() / value


def validate_blocks(config: dict[str, object]) -> None:
    blocks = config.get("blocks", {})
    if not config.get("allow_partial_blocks") and set(blocks) != set(EXPECTED_LABELS):
        raise SystemExit(f"Current protocol requires exactly {list(EXPECTED_LABELS)}")
    windows = []
    for block_id, expected_label in EXPECTED_LABELS.items():
        if block_id not in blocks:
            continue
        block = blocks[block_id]
        if block.get("label") != expected_label:
            raise SystemExit(f"{block_id} label must be {expected_label!r}, got {block.get('label')!r}")
        window = block.get("mocap_window_s")
        if window and None not in window:
            lo, hi = map(float, window)
            if hi <= lo:
                raise SystemExit(f"{block_id} has invalid mocap_window_s {window}")
            windows.append((lo, hi, block_id))
    for (_, previous_hi, previous_id), (next_lo, _, next_id) in zip(windows, windows[1:]):
        if next_lo < previous_hi:
            raise SystemExit(f"Protocol windows overlap: {previous_id} and {next_id}")


def self_test() -> None:
    time_s = np.arange(0.0, 50.0, 0.01)
    signal = sum(20.0 * np.exp(-0.5 * ((time_s - center) / 0.8) ** 2) for center in (10.0, 20.0, 30.0))
    bouts = peak_bouts(time_s, signal, [5.0, 35.0], +1, min_dist_s=5.0)
    assert len(bouts) == 3
    validate_blocks({"allow_partial_blocks": True, "blocks": {"B1": {"label": "flexion", "mocap_window_s": [5, 35]}}})
    print("extract_block_samples self-test: PASS")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--mocap-reference", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--report-json", type=Path)
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.manifest is None or args.mocap_reference is None or args.output is None:
        raise SystemExit("--manifest, --mocap-reference and --output are required (or use --self-test).")
    config = json.loads(args.manifest.read_text(encoding="utf-8"))
    validate_blocks(config)
    reference = load_reference(args.mocap_reference)
    tm = reference["time_s"]

    results = {}
    for segment_id, segment in config.get("segments", {}).items():
        source = resolve(segment["imu_source"])
        if not source.is_file():
            raise SystemExit(f"Missing segment source: {source}")
        results[segment_id] = run_segment(source, config)

    output_rows = []
    report = {"pair": config.get("pair"), "method_version": config.get("method_version"), "blocks": {}}
    for block_id, block in config.get("blocks", {}).items():
        label = block["label"]
        segment_id = block["segment_id"]
        if segment_id not in results:
            raise SystemExit(f"{block_id} references unknown segment {segment_id}")
        segment = config["segments"][segment_id]
        clock = segment.get("clock", {})
        if clock.get("a") is None or clock.get("b") is None or not clock.get("phase_confirmed"):
            raise SystemExit(f"{block_id}: clock must have a,b and phase_confirmed=true")
        a, b = float(clock["a"]), float(clock["b"])
        signal_name = block.get("mocap_signal", SIGNAL_BY_LABEL.get(label))
        if signal_name not in reference:
            raise SystemExit(f"{block_id}: unknown MoCap signal {signal_name!r}")
        signal = reference[signal_name]
        explicit = block.get("bouts")
        if explicit:
            bouts = [(float(lo), float(hi)) for lo, hi in explicit]
            source = "manifest"
        else:
            window = block.get("mocap_window_s")
            if not window or None in window:
                raise SystemExit(f"{block_id}: provide bouts or a complete mocap_window_s")
            bouts = peak_bouts(
                tm, signal, window, int(block.get("expected_sign", 0)),
                float(block.get("min_peak_distance_s", 6.0)),
            )
            source = "deterministic_peak_detector"
        result = results[segment_id]
        abs_mocap = bool(block.get("abs_mocap", "bend" in label or label in {"flexion", "extension"}))
        for rep_index, (lo, hi) in enumerate(bouts, 1):
            series = series_for(result, block["readout"], a, b, lo)
            grid = np.arange(lo, hi, 0.01)
            valid = (a * grid + b >= result.t_s[0]) & (a * grid + b <= result.t_s[-1])
            grid = grid[valid]
            if len(grid) < 5:
                continue
            quiet = np.arange(max(float(tm[0]), lo - 1.2), lo - 0.2, 0.01)
            mocap_zero = float(np.mean(np.interp(quiet, tm, signal))) if len(quiet) >= 5 else 0.0
            imu_zero = 0.0
            if len(quiet) >= 5 and a * quiet[0] + b >= result.t_s[0] and a * quiet[-1] + b <= result.t_s[-1]:
                imu_zero = float(np.mean(np.interp(a * quiet + b, result.t_s, series)))
            mocap_values = np.interp(grid, tm, signal) - mocap_zero
            imu_values = np.interp(a * grid + b, result.t_s, series) - imu_zero
            if abs_mocap:
                mocap_values = np.abs(mocap_values)
            bout_id = f"{block_id}_rep{rep_index:02d}"
            for time_value, imu_value, mocap_value in zip(grid, imu_values, mocap_values):
                output_rows.append({
                    "subject": config.get("pair", "TXX_PXX").split("_")[0],
                    "movement": label,
                    "block_id": block_id,
                    "bout_id": bout_id,
                    "time_s": float(time_value),
                    "imu_deg": float(imu_value),
                    "mocap_deg": float(mocap_value),
                })
        report["blocks"][block_id] = {"label": label, "segment_id": segment_id, "bout_source": source, "bouts": bouts, "n_bouts": len(bouts)}

    if not output_rows:
        raise SystemExit("No aligned block samples were produced")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(output_rows[0]))
        writer.writeheader(); writer.writerows(output_rows)
    report_path = args.report_json or args.output.with_suffix(".json")
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"wrote {len(output_rows)} samples to {args.output} and {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
