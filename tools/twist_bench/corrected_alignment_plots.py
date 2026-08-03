from __future__ import annotations

"""Regenerate IMU/MoCap time-alignment figures from the corrected cohort authority.

The alignment signal intentionally matches the historical sync audit: the IMU
trace is the smoothed mean five-sensor gyro-magnitude envelope and the MoCap
trace is the smoothed median all-marker speed envelope.  The accepted affine
clock transforms are read from the promoted per-trial block manifests; they are
not re-fitted here.  Multi-log trials are plotted per accepted source segment.
"""

import argparse
import csv
import hashlib
import json
import math
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

MAPPING_RUN_DIR = HERE / "runs/mapping_repair_2026-07-13"
MAPPING_RUN_MANIFEST = MAPPING_RUN_DIR / "run_manifest.json"

import dataset_adapter as bd  # noqa: E402
import placement_maps as pm  # noqa: E402
import sync_audit as sa  # noqa: E402
import twist_bench_v0 as v0  # noqa: E402


# Cohort comes from the dataset config, not from a constant here -- a hard-coded
# session list makes the script describe one study rather than one method.
ACTIONS = ["flexion", "extension", "left_bend", "right_bend", "left_twist", "right_twist"]
ACTION_LABELS = {
    "flexion": "Flex",
    "extension": "Ext",
    "left_bend": "Left bend",
    "right_bend": "Right bend",
    "left_twist": "Left twist",
    "right_twist": "Right twist",
}
ACTION_COLORS = {
    "flexion": "#4c78a8",
    "extension": "#72b7b2",
    "left_bend": "#54a24b",
    "right_bend": "#b279a2",
    "left_twist": "#e45756",
    "right_twist": "#f58518",
}
MOCAP_COLOR = "#1f77b4"
IMU_COLOR = "#ff7f0e"
EXPECTED_IMUS = {f"IMU{index}" for index in range(5)}
FULL_IMU_SUPPORT_THRESHOLD = 0.999
MOCAP_SUPPORT_THRESHOLD = 0.50


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        raise ValueError(f"no rows for {path}")
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def canon_label(value: str) -> str:
    return str(value).strip().replace(" ", "_")


def mocap_activity_with_support(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    """Historical marker-speed envelope plus raw finite-marker support per frame."""
    time_s, markers_raw = sa.parse_motive_markers(path, gap_fill=False)
    usable = [values for values in markers_raw.values() if np.isfinite(values).any()]
    if not usable:
        raise ValueError(f"no usable MoCap markers in {path}")
    support = np.mean(
        np.vstack([np.isfinite(values).all(axis=1) for values in usable]),
        axis=0,
    )
    speeds = []
    for values in usable:
        filled = sa.fill_gaps(values)
        velocity = np.gradient(filled, time_s, axis=0)
        speeds.append(np.linalg.norm(velocity, axis=1))
    activity = np.nanmedian(np.vstack(speeds), axis=0)
    return time_s, sa.smooth(time_s, activity), support, len(usable)


def imu_activity_with_support(
    path: Path,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float, float, list[str]]:
    """Historical partial-frame envelope plus explicit five-IMU support."""
    text = path.read_text(encoding="utf-8", errors="replace")
    records = v0.parse_serial_text(text) or v0.parse_long_table_rows(v0.read_dict_rows(text))
    if not records:
        raise ValueError(f"no IMU rows parsed from {path}")
    by_time: dict[float, dict[str, list[float]]] = {}
    observed: set[str] = set()
    for record in records:
        imu = record.imu.strip().upper()
        if imu not in EXPECTED_IMUS:
            continue
        observed.add(imu)
        magnitude = float(np.linalg.norm([record.gx_dps, record.gy_dps, record.gz_dps]))
        by_time.setdefault(float(record.t_s), {}).setdefault(imu, []).append(magnitude)
    if observed != EXPECTED_IMUS:
        raise ValueError(f"{path}: expected {sorted(EXPECTED_IMUS)}, observed {sorted(observed)}")
    times = np.asarray(sorted(by_time), dtype=float)
    activity = np.asarray([
        float(np.mean([np.mean(values) for values in by_time[time].values()]))
        for time in times
    ])
    support = np.asarray([len(by_time[time]) / len(EXPECTED_IMUS) for time in times], dtype=float)
    times -= times[0]
    positive_steps = np.diff(times)
    positive_steps = positive_steps[positive_steps > 0]
    median_step = float(np.median(positive_steps)) if len(positive_steps) else 0.0
    max_group_gap = float(np.max(positive_steps)) if len(positive_steps) else 0.0
    per_imu_max_gaps = []
    for imu in sorted(EXPECTED_IMUS):
        imu_times = np.asarray([time for time in sorted(by_time) if imu in by_time[time]], dtype=float)
        imu_steps = np.diff(imu_times)
        imu_steps = imu_steps[imu_steps > 0]
        per_imu_max_gaps.append(float(np.max(imu_steps)) if len(imu_steps) else math.inf)
    max_per_imu_gap = max(per_imu_max_gaps)
    return (
        times, sa.smooth(times, activity), support, median_step,
        max_group_gap, max_per_imu_gap, sorted(observed),
    )


def nearest_support(
    source_time: np.ndarray,
    source_support: np.ndarray,
    query_time: np.ndarray,
    max_nearest_gap_s: float,
) -> np.ndarray:
    """Nearest-frame support, forced to zero across internal timestamp gaps."""
    right = np.searchsorted(source_time, query_time, side="left")
    right = np.clip(right, 0, len(source_time) - 1)
    left = np.clip(right - 1, 0, len(source_time) - 1)
    use_right = np.abs(source_time[right] - query_time) < np.abs(source_time[left] - query_time)
    nearest = np.where(use_right, right, left)
    distance = np.abs(source_time[nearest] - query_time)
    support = source_support[nearest].astype(float, copy=True)
    support[distance > max_nearest_gap_s] = 0.0
    return support


def historical_window_corr(
    ti: np.ndarray,
    ia: np.ndarray,
    tm: np.ndarray,
    ma: np.ndarray,
    a: float,
    b: float,
    lo: float,
    hi: float,
) -> float | None:
    g0 = max(float(lo), float(tm[0]), float((ti[0] - b) / a))
    g1 = min(float(hi), float(tm[-1]), float((ti[-1] - b) / a))
    if g1 - g0 < 8.0:
        return None
    grid = np.arange(g0, g1, 0.05)
    imu = np.interp(a * grid + b, ti, ia)
    mocap = np.interp(grid, tm, ma)
    if len(grid) < 20 or np.std(imu) < 1e-8 or np.std(mocap) < 1e-8:
        return None
    return float(np.corrcoef(imu, mocap)[0, 1])


def support_aware_window_corr(
    ti: np.ndarray,
    ia: np.ndarray,
    imu_support: np.ndarray,
    max_nearest_gap_s: float,
    tm: np.ndarray,
    ma: np.ndarray,
    marker_support: np.ndarray,
    a: float,
    b: float,
    lo: float,
    hi: float,
    *,
    minimum_joint_fraction: float,
) -> tuple[float | None, float]:
    g0 = max(float(lo), float(tm[0]), float((ti[0] - b) / a))
    g1 = min(float(hi), float(tm[-1]), float((ti[-1] - b) / a))
    if g1 - g0 < 8.0:
        return None, 0.0
    grid = np.arange(g0, g1, 0.05)
    imu_query = a * grid + b
    imu = np.interp(imu_query, ti, ia)
    mocap = np.interp(grid, tm, ma)
    imu_full = nearest_support(ti, imu_support, imu_query, max_nearest_gap_s) >= FULL_IMU_SUPPORT_THRESHOLD
    mocap_visible = np.interp(grid, tm, marker_support) >= MOCAP_SUPPORT_THRESHOLD
    good = imu_full & mocap_visible & np.isfinite(imu) & np.isfinite(mocap)
    joint_fraction = float(np.mean(good)) if len(good) else 0.0
    if joint_fraction < minimum_joint_fraction or int(np.sum(good)) < 160:
        return None, joint_fraction
    imu_good = imu[good]
    mocap_good = mocap[good]
    if np.std(imu_good) < 1e-8 or np.std(mocap_good) < 1e-8:
        return None, joint_fraction
    return float(np.corrcoef(imu_good, mocap_good)[0, 1]), joint_fraction


def aligned_trace(
    ti: np.ndarray,
    ia: np.ndarray,
    tm: np.ndarray,
    ma: np.ndarray,
    marker_support: np.ndarray,
    imu_support: np.ndarray,
    max_nearest_gap_s: float,
    a: float,
    b: float,
    step_s: float = 0.05,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    g0, g1 = sa.overlap(ti, tm, a, b)
    if g1 - g0 < 30.0:
        raise ValueError(f"overlap too short: {g0:.3f}..{g1:.3f}")
    grid = np.arange(g0, g1, step_s)
    imu = np.interp(a * grid + b, ti, sa.zscore(ia))
    mocap = np.interp(grid, tm, sa.zscore(ma))
    mocap_support = np.interp(grid, tm, marker_support)
    mapped_imu_support = nearest_support(ti, imu_support, a * grid + b, max_nearest_gap_s)
    return grid, mocap, imu, mocap_support, mapped_imu_support


def trial_structure(subject: str, validation: dict[str, Any]) -> tuple[list[dict[str, Any]], Path, Path, dict[str, Any]]:
    trial = f"T{subject}_P{subject}"
    manifest_path = bd.manifest_path(subject).resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    validation_subject = validation["subjects"][trial]
    corrected_subject_path = (
        MAPPING_RUN_DIR / "C_corrected_uniform" / "validation" / "subjects" / f"{trial}.json"
    ).resolve()
    standalone_subject = json.loads(corrected_subject_path.read_text(encoding="utf-8"))
    if standalone_subject != validation_subject:
        raise ValueError(f"{trial}: standalone corrected authority differs from cohort authority")
    authority = {block["block"]: block for block in validation_subject["blocks"]}
    overrides = bd.BLOCK_OVERRIDES.get(subject, {})
    raw_segments = bd._segments(manifest)

    if raw_segments:
        segments = {
            name: {
                "segment": name,
                "raw": data["raw"].resolve(),
                "clock": {"a": float(data["clock"]["a"]), "b": float(data["clock"]["b"])},
                "blocks": [],
            }
            for name, data in raw_segments.items()
        }
    else:
        segments = {
            "single": {
                "segment": "single",
                "raw": bd.imu_path(subject).resolve(),
                "clock": {"a": float(manifest["clock"]["a"]), "b": float(manifest["clock"]["b"])},
                "blocks": [],
            }
        }

    seen_blocks: set[str] = set()
    for block_id, block in bd._manifest_blocks(manifest):
        override = overrides.get(block_id, {})
        label = canon_label(override.get("label", block["label"]))
        if label not in ACTIONS:
            continue
        if block_id not in authority:
            raise KeyError(f"{trial}/{block_id} missing from corrected validation authority")
        window = bd._block_window(block, override)
        if raw_segments:
            segment = block.get("segment") or bd._pick_segment(raw_segments, window)
        else:
            segment = "single"
        auth = authority[block_id]
        adopted_clock = segments[segment]["clock"]
        auth_clock = auth.get("clock", {})
        if not (
            math.isclose(float(auth_clock.get("a", math.nan)), adopted_clock["a"], abs_tol=1e-12)
            and math.isclose(float(auth_clock.get("b", math.nan)), adopted_clock["b"], abs_tol=1e-12)
        ):
            raise ValueError(
                f"{trial}/{block_id}: corrected clock {auth_clock} != selected {segment} clock {adopted_clock}"
            )
        segments[segment]["blocks"].append({
            "block": block_id,
            "action": str(auth["label"]),
            "quality": str(auth["quality"]),
            "window_start_s": float(window[0]),
            "window_end_s": float(window[1]),
            "n_detected": int(auth["n_bouts_detected"]),
            "n_scored": int(auth["n_bouts_scored"]),
            "detected_bouts": [[float(lo), float(hi)] for lo, hi in auth["detected_bouts"]],
            "scored_bouts": [[float(lo), float(hi)] for lo, hi in auth["scored_bouts"]],
        })
        seen_blocks.add(block_id)
    if seen_blocks != set(authority):
        raise ValueError(f"{trial}: block mismatch seen={sorted(seen_blocks)} authority={sorted(authority)}")

    output = list(segments.values())
    for segment in output:
        segment["blocks"].sort(key=lambda row: row["window_start_s"])
    output.sort(key=lambda row: min([b["window_start_s"] for b in row["blocks"]] or [math.inf]))
    return output, bd.mocap_path(subject).resolve(), manifest_path, validation_subject


def shade_blocks(ax, blocks: list[dict[str, Any]], lo: float, hi: float, *, labels: bool) -> None:
    for block in blocks:
        left = max(lo, block["window_start_s"])
        right = min(hi, block["window_end_s"])
        if right <= left:
            continue
        action = block["action"]
        ax.axvspan(left, right, color=ACTION_COLORS[action], alpha=0.045, linewidth=0)
        if labels:
            ax.text(
                (left + right) / 2.0,
                0.98,
                f"{block['block']} {ACTION_LABELS[action]}",
                transform=ax.get_xaxis_transform(),
                ha="center",
                va="top",
                fontsize=7,
                color=ACTION_COLORS[action],
            )


def plot_activity(
    ax,
    grid: np.ndarray,
    mocap_z: np.ndarray,
    imu_z: np.ndarray,
    mocap_support: np.ndarray,
    imu_support: np.ndarray,
    *,
    lw: float,
) -> None:
    mocap_good = mocap_support >= MOCAP_SUPPORT_THRESHOLD
    imu_good = imu_support >= FULL_IMU_SUPPORT_THRESHOLD
    ax.plot(grid, np.where(mocap_good, mocap_z, np.nan), color=MOCAP_COLOR, lw=lw)
    ax.plot(grid, np.where(imu_good, imu_z, np.nan), color=IMU_COLOR, lw=lw, alpha=0.88)
    if np.any(~mocap_good):
        ax.fill_between(grid, -2.7, 6.2, where=~mocap_good, color="#777777", alpha=0.06, linewidth=0)
    if np.any(~imu_good):
        ax.fill_between(grid, -2.7, 6.2, where=~imu_good, color="#d62728", alpha=0.035, linewidth=0)


def build_subject(subject: str, validation: dict[str, Any]) -> dict[str, Any]:
    segments, mocap_path, manifest_path, validation_subject = trial_structure(subject, validation)
    tm, ma, marker_support, marker_count = mocap_activity_with_support(mocap_path)
    trial = f"T{subject}_P{subject}"
    placement = pm.resolve_placement(trial_id=trial)
    segment_rows: list[dict[str, Any]] = []
    block_rows: list[dict[str, Any]] = []
    plot_segments: list[dict[str, Any]] = []
    corrected_subject_path = (
        MAPPING_RUN_DIR
        / "C_corrected_uniform"
        / "validation"
        / "subjects"
        / f"{trial}.json"
    ).resolve()
    input_paths = {mocap_path, manifest_path, corrected_subject_path}

    for segment in segments:
        raw = segment["raw"]
        input_paths.add(raw)
        (
            ti, ia, native_imu_support, median_imu_step,
            max_group_timestamp_gap, max_per_imu_timestamp_gap, observed_imus,
        ) = imu_activity_with_support(raw)
        historical_ti, historical_ia = sa.imu_activity(raw)
        if len(historical_ti) != len(ti) or not np.allclose(historical_ti, ti, rtol=0.0, atol=1e-12):
            raise ValueError(f"{raw}: historical and support-aware IMU timestamps differ")
        max_nearest_gap_s = max(0.10, 5.0 * median_imu_step)
        a, b = segment["clock"]["a"], segment["clock"]["b"]
        grid, mocap_z, imu_z, mocap_support_grid, imu_support_grid = aligned_trace(
            ti, ia, tm, ma, marker_support, native_imu_support, max_nearest_gap_s, a, b
        )
        historical_corr = sa.corr_for(historical_ti, historical_ia, tm, ma, a, b)
        support_corr, joint_support_fraction = support_aware_window_corr(
            ti, ia, native_imu_support, max_nearest_gap_s,
            tm, ma, marker_support, a, b, float(grid[0]), float(grid[-1]),
            minimum_joint_fraction=0.10,
        )
        deltas = np.arange(-2.0, 2.0001, 0.05)
        local_corrs = np.asarray([
            support_aware_window_corr(
                ti, ia, native_imu_support, max_nearest_gap_s,
                tm, ma, marker_support, a, b + float(delta), float(grid[0]), float(grid[-1]),
                minimum_joint_fraction=0.10,
            )[0]
            for delta in deltas
        ], dtype=float)
        if np.any(np.isfinite(local_corrs)):
            best_index = int(np.nanargmax(local_corrs))
            best_delta = float(deltas[best_index])
            best_corr = float(local_corrs[best_index])
        else:
            best_delta = None
            best_corr = None

        for block in segment["blocks"]:
            block_deltas = np.arange(-0.5, 0.5001, 0.025)
            historical_block_corr = historical_window_corr(
                historical_ti, historical_ia, tm, ma, a, b,
                block["window_start_s"], block["window_end_s"]
            )
            canonical_scored_fraction = (
                block["n_scored"] / block["n_detected"] if block["n_detected"] else 0.0
            )
            block_grid_mask = (
                (grid >= block["window_start_s"]) & (grid <= block["window_end_s"])
            )
            full_imu_fraction = (
                float(np.mean(imu_support_grid[block_grid_mask] >= FULL_IMU_SUPPORT_THRESHOLD))
                if np.any(block_grid_mask) else 0.0
            )
            _, block_joint_fraction = support_aware_window_corr(
                ti, ia, native_imu_support, max_nearest_gap_s,
                tm, ma, marker_support, a, b,
                block["window_start_s"], block["window_end_s"], minimum_joint_fraction=0.0,
            )
            if canonical_scored_fraction >= 0.50:
                block_corr, _ = support_aware_window_corr(
                    ti, ia, native_imu_support, max_nearest_gap_s,
                    tm, ma, marker_support, a, b,
                    block["window_start_s"], block["window_end_s"], minimum_joint_fraction=0.50,
                )
                correlations = [
                    support_aware_window_corr(
                        ti, ia, native_imu_support, max_nearest_gap_s,
                        tm, ma, marker_support, a, b + float(delta),
                        block["window_start_s"], block["window_end_s"], minimum_joint_fraction=0.50,
                    )[0]
                    for delta in block_deltas
                ]
                finite = [
                    (idx, value) for idx, value in enumerate(correlations)
                    if value is not None and np.isfinite(value)
                ]
                if finite:
                    idx, block_best_corr = max(finite, key=lambda pair: pair[1])
                    block_best_delta = float(block_deltas[idx])
                    metric_status = "available"
                else:
                    block_best_corr = None
                    block_best_delta = None
                    metric_status = "suppressed_insufficient_joint_support"
            else:
                block_corr = None
                block_best_corr = None
                block_best_delta = None
                metric_status = "suppressed_canonical_coverage_below_50pct"
            block["full_imu_support_fraction"] = full_imu_fraction
            block["joint_visible_support_fraction"] = block_joint_fraction
            block_support_mask = (tm >= block["window_start_s"]) & (tm <= block["window_end_s"])
            block_rows.append({
                "subject": f"T{subject}",
                "trial_id": trial,
                "segment": segment["segment"],
                "block": block["block"],
                "action": block["action"],
                "quality": block["quality"],
                "window_start_s": round(block["window_start_s"], 6),
                "window_end_s": round(block["window_end_s"], 6),
                "n_bouts_detected": block["n_detected"],
                "n_bouts_scored": block["n_scored"],
                "canonical_scored_fraction": round(canonical_scored_fraction, 6),
                "full_5imu_support_fraction": round(full_imu_fraction, 6),
                "joint_visible_support_fraction": round(block_joint_fraction, 6),
                "historical_unmasked_corr_at_adopted_clock": (
                    None if historical_block_corr is None else round(float(historical_block_corr), 6)
                ),
                "support_aware_corr_at_adopted_clock": None if block_corr is None else round(float(block_corr), 6),
                "support_aware_metric_status": metric_status,
                "best_support_aware_local_delta_b_s": (
                    None if block_best_delta is None else round(block_best_delta, 6)
                ),
                "best_support_aware_local_corr": (
                    None if block_best_corr is None else round(float(block_best_corr), 6)
                ),
                "mean_raw_marker_support_fraction": (
                    round(float(np.mean(marker_support[block_support_mask])), 6)
                    if np.any(block_support_mask) else None
                ),
            })

        segment_rows.append({
            "subject": f"T{subject}",
            "trial_id": trial,
            "segment": segment["segment"],
            "imu_path": str(raw),
            "mocap_path": str(mocap_path),
            "clock_a": round(a, 8),
            "clock_b_s": round(b, 6),
            "overlap_start_mocap_s": round(float(grid[0]), 6),
            "overlap_end_mocap_s": round(float(grid[-1]), 6),
            "overlap_duration_s": round(float(grid[-1] - grid[0]), 6),
            "imu_activity_samples": len(ti),
            "mocap_activity_samples": len(tm),
            "mocap_marker_count": marker_count,
            "observed_imu_ids": "|".join(observed_imus),
            "native_imu_median_step_s": round(median_imu_step, 6),
            "max_group_timestamp_gap_s": round(max_group_timestamp_gap, 6),
            "max_per_imu_timestamp_gap_s": round(max_per_imu_timestamp_gap, 6),
            "full_5imu_support_fraction": round(
                float(np.mean(imu_support_grid >= FULL_IMU_SUPPORT_THRESHOLD)), 6
            ),
            "joint_visible_support_fraction": round(joint_support_fraction, 6),
            "mean_raw_marker_support_fraction": round(float(np.mean(mocap_support_grid)), 6),
            "minimum_raw_marker_support_fraction": round(float(np.min(mocap_support_grid)), 6),
            "historical_unmasked_corr_at_adopted_clock": round(float(historical_corr), 6),
            "support_aware_corr_at_adopted_clock": (
                None if support_corr is None else round(float(support_corr), 6)
            ),
            "best_support_aware_local_delta_b_s": (
                None if best_delta is None else round(best_delta, 6)
            ),
            "best_support_aware_local_corr": None if best_corr is None else round(best_corr, 6),
            "n_blocks": len(segment["blocks"]),
        })
        plot_segments.append({
            **segment,
            "grid": grid,
            "mocap_z": mocap_z,
            "imu_z": imu_z,
            "marker_support": mocap_support_grid,
            "imu_support": imu_support_grid,
            "deltas": deltas,
            "local_corrs": local_corrs,
            "historical_corr": historical_corr,
            "support_corr": support_corr,
            "joint_support_fraction": joint_support_fraction,
            "best_delta": best_delta,
            "best_corr": best_corr,
        })

    return {
        "subject": f"T{subject}",
        "trial_id": trial,
        "placement_status": placement.status,
        "mapping_sha256": placement.canonical_sha256,
        "role_to_imu": dict(placement.role_to_imu),
        "legacy_manifest_status": validation_subject["legacy_manifest"]["status"],
        "mocap_time": tm,
        "mocap_support": marker_support,
        "corrected_subject_authority": {
            "path": str(corrected_subject_path),
            "sha256": sha256_file(corrected_subject_path),
        },
        "segments": plot_segments,
        "segment_rows": segment_rows,
        "block_rows": block_rows,
        "input_files": [
            {"path": str(path), "sha256": sha256_file(path), "size_bytes": path.stat().st_size}
            for path in sorted(input_paths, key=str)
        ],
    }


def plot_subject_summary(path: Path, payload: dict[str, Any], dpi: int) -> None:
    segments = payload["segments"]
    fig = plt.figure(figsize=(20, max(7.4, 5.1 * len(segments) + 1.4)), constrained_layout=True)
    spec = fig.add_gridspec(
        len(segments) + 1, 2,
        width_ratios=[3.2, 1.0], height_ratios=[0.42] + [1.0] * len(segments),
    )
    coverage = fig.add_subplot(spec[0, :])
    full_lo = float(payload["mocap_time"][0])
    full_hi = float(payload["mocap_time"][-1])
    mocap_good = payload["mocap_support"] >= MOCAP_SUPPORT_THRESHOLD
    coverage.plot(
        payload["mocap_time"], np.where(mocap_good, 1.0, np.nan),
        color=MOCAP_COLOR, lw=5.0, solid_capstyle="butt",
    )
    accepted_ranges = []
    for segment in segments:
        grid = segment["grid"]
        imu_good = segment["imu_support"] >= FULL_IMU_SUPPORT_THRESHOLD
        coverage.plot(grid, np.where(imu_good, 0.0, np.nan), color=IMU_COLOR, lw=5.0, solid_capstyle="butt")
        accepted_ranges.append((float(grid[0]), float(grid[-1])))
        coverage.text(
            (float(grid[0]) + float(grid[-1])) / 2.0, -0.30, segment["segment"],
            ha="center", va="top", fontsize=7, color=IMU_COLOR,
        )
    accepted_ranges.sort()
    cursor = full_lo
    for lo, hi in accepted_ranges + [(full_hi, full_hi)]:
        if lo - cursor > 1.0:
            coverage.axvspan(cursor, lo, color="#777777", alpha=0.12, linewidth=0)
            if lo - cursor > 20.0:
                coverage.text((cursor + lo) / 2.0, 0.5, "no accepted IMU segment", ha="center", va="center", fontsize=7)
        cursor = max(cursor, hi)
    coverage.set_xlim(full_lo, full_hi)
    coverage.set_ylim(-0.48, 1.35)
    coverage.set_yticks([0.0, 1.0], ["full 5-IMU frames", "MoCap ≥50% markers"])
    coverage.set_xlabel("full-trial MoCap time (s)")
    coverage.set_title("Coverage lane: blank/red-tinted IMU intervals are not drawn as valid five-sensor activity", fontsize=10)
    coverage.grid(axis="x", alpha=0.15)

    for row, segment in enumerate(segments):
        left = fig.add_subplot(spec[row + 1, 0])
        right = fig.add_subplot(spec[row + 1, 1])
        grid = segment["grid"]
        plot_activity(
            left, grid, segment["mocap_z"], segment["imu_z"],
            segment["marker_support"], segment["imu_support"], lw=0.9,
        )
        left.set_ylim(-2.7, 6.2)
        left.set_xlim(float(grid[0]), float(grid[-1]))
        left.grid(axis="y", alpha=0.18)
        shade_blocks(left, segment["blocks"], float(grid[0]), float(grid[-1]), labels=True)
        a, b = segment["clock"]["a"], segment["clock"]["b"]
        visible_text = "NA" if segment["support_corr"] is None else f"{segment['support_corr']:.3f}"
        left.set_title(
            f"{payload['subject']} · {segment['segment']} · t_IMU={a:.6f}·t_MoCap{b:+.3f}s · "
            f"r_hist={segment['historical_corr']:.3f} · r_visible={visible_text} · "
            f"joint support={segment['joint_support_fraction']:.1%}"
        )
        left.set_xlabel("MoCap time (s)")
        left.set_ylabel("z-scored activity")
        left.legend(handles=[
            Line2D([], [], color=MOCAP_COLOR, lw=1.5, label="MoCap marker-speed activity"),
            Line2D([], [], color=IMU_COLOR, lw=1.5, label="full-frame five-IMU gyro activity"),
            Patch(facecolor="#d62728", alpha=0.12, label="IMU support insufficient (trace hidden)"),
        ], loc="upper right", frameon=False)

        right.plot(segment["deltas"], segment["local_corrs"], color="#4c4c4c", lw=1.7)
        right.axvline(0.0, color="#111111", lw=1.2, label="adopted clock")
        if segment["best_delta"] is not None:
            right.axvline(segment["best_delta"], color="#b22222", lw=1.2, linestyle="--", label="best ±2 s")
            right.scatter([segment["best_delta"]], [segment["best_corr"]], color="#b22222", s=28, zorder=3)
            sensitivity_title = f"best Δb={segment['best_delta']:+.2f}s, r_visible={segment['best_corr']:.3f}"
        else:
            sensitivity_title = "insufficient joint support"
        right.set_title(f"Support-aware local offset sensitivity\n{sensitivity_title}")
        right.set_xlabel("Δb around adopted clock (s)")
        right.set_ylabel("visible-range correlation")
        right.grid(alpha=0.2)
        right.legend(loc="best", frameon=False, fontsize=8)

    fig.suptitle(
        f"{payload['trial_id']} corrected-run IMU–MoCap time-alignment audit (support-aware v2)",
        fontsize=16,
    )
    fig.savefig(path.with_suffix(".png"), dpi=dpi)
    fig.savefig(path.with_suffix(".svg"))
    plt.close(fig)


def plot_subject_detail(path: Path, payload: dict[str, Any], chunk_s: float, dpi: int) -> None:
    chunks: list[tuple[dict[str, Any], float, float]] = []
    for segment in payload["segments"]:
        start, end = float(segment["grid"][0]), float(segment["grid"][-1])
        for index in range(max(1, int(math.ceil((end - start) / chunk_s)))):
            lo = start + index * chunk_s
            hi = min(end, lo + chunk_s)
            chunks.append((segment, lo, hi))
    fig, axes = plt.subplots(
        len(chunks), 1,
        figsize=(22, max(3.0, 2.15 * len(chunks))),
        squeeze=False,
        constrained_layout=True,
    )
    for axis, (segment, lo, hi) in zip(axes[:, 0], chunks):
        grid = segment["grid"]
        mask = (grid >= lo) & (grid <= hi)
        plot_activity(
            axis,
            grid[mask], segment["mocap_z"][mask], segment["imu_z"][mask], segment["marker_support"][mask],
            segment["imu_support"][mask],
            lw=0.9,
        )
        axis.set_xlim(lo, hi)
        axis.set_ylim(-2.7, 6.2)
        axis.grid(alpha=0.16)
        shade_blocks(axis, segment["blocks"], lo, hi, labels=True)
        axis.set_ylabel(f"{segment['segment']}\n{lo:.0f}–{hi:.0f}s", fontsize=8)
    axes[-1, 0].set_xlabel("MoCap time (s)")
    handles = [
        Line2D([], [], color=MOCAP_COLOR, lw=1.5, label="MoCap marker-speed activity"),
        Line2D([], [], color=IMU_COLOR, lw=1.5, label="full-frame five-IMU activity mapped to MoCap time"),
        Patch(facecolor="#d62728", alpha=0.12, label="IMU support insufficient (trace hidden)"),
    ]
    fig.legend(handles=handles, loc="upper right", frameon=False)
    fig.suptitle(f"{payload['trial_id']} corrected-run detailed time alignment ({chunk_s:.0f}s lanes)", fontsize=16)
    fig.savefig(path.with_suffix(".png"), dpi=dpi)
    fig.savefig(path.with_suffix(".svg"))
    plt.close(fig)


def plot_subject_actions(path: Path, payload: dict[str, Any], dpi: int) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(22, 12.5), sharey=True, constrained_layout=True)
    for axis, action in zip(axes.ravel(), ACTIONS):
        matching = []
        for segment in payload["segments"]:
            for block in segment["blocks"]:
                if block["action"] == action:
                    matching.append((segment, block))
        detected_total = sum(block["n_detected"] for _, block in matching)
        scored_total = sum(block["n_scored"] for _, block in matching)
        qualities = sorted({block["quality"] for _, block in matching})
        full_support_count = 0
        support_count = 0
        for segment, block in matching:
            grid = segment["grid"]
            mask = (grid >= block["window_start_s"]) & (grid <= block["window_end_s"])
            if np.any(mask):
                support_count += int(np.sum(mask))
                full_support_count += int(np.sum(segment["imu_support"][mask] >= FULL_IMU_SUPPORT_THRESHOLD))
                plot_activity(
                    axis,
                    grid[mask], segment["mocap_z"][mask], segment["imu_z"][mask], segment["marker_support"][mask],
                    segment["imu_support"][mask],
                    lw=1.0,
                )
            scored_keys = {(round(lo, 3), round(hi, 3)) for lo, hi in block["scored_bouts"]}
            for lo, hi in block["detected_bouts"]:
                accepted = (round(lo, 3), round(hi, 3)) in scored_keys
                axis.axvspan(lo, hi, color="#2ca02c" if accepted else "#d62728", alpha=0.035, linewidth=0)
        if matching:
            lo = min(block["window_start_s"] for _, block in matching)
            hi = max(block["window_end_s"] for _, block in matching)
            axis.set_xlim(lo, hi)
        axis.set_ylim(-2.7, 6.2)
        axis.grid(alpha=0.16)
        axis.set_title(
            f"{ACTION_LABELS[action]} · scored/detected {scored_total}/{detected_total}\n"
            f"quality: {', '.join(qualities) if qualities else 'unavailable'} · "
            f"full-5 IMU support: {(full_support_count / support_count if support_count else 0.0):.1%}"
        )
        axis.set_xlabel("MoCap time (s)")
        axis.set_ylabel("z-scored activity")
    handles = [
        Line2D([], [], color=MOCAP_COLOR, lw=1.6, label="MoCap marker-speed activity"),
        Line2D([], [], color=IMU_COLOR, lw=1.6, label="full-frame five-IMU gyro activity"),
        Line2D([], [], color="#2ca02c", lw=7, alpha=0.25, label="scored bout"),
        Line2D([], [], color="#d62728", lw=7, alpha=0.25, label="coverage-excluded detected bout"),
    ]
    fig.legend(handles=handles, loc="outside lower center", ncol=4, frameon=False)
    fig.suptitle(f"{payload['trial_id']} corrected-run six-action alignment detail (support-aware v2)", fontsize=17)
    fig.savefig(path.with_suffix(".png"), dpi=dpi)
    fig.savefig(path.with_suffix(".svg"))
    plt.close(fig)


def plot_cohort(path: Path, payloads: list[dict[str, Any]], dpi: int) -> None:
    fig, axes = plt.subplots(5, 3, figsize=(22, 26), squeeze=False, constrained_layout=True)
    flat = axes.ravel()
    for axis, payload in zip(flat, payloads):
        historical_correlations = []
        visible_correlations = []
        for segment in payload["segments"]:
            grid = segment["grid"]
            stride = max(1, int(math.ceil(len(grid) / 5000)))
            plot_activity(
                axis,
                grid[::stride], segment["mocap_z"][::stride], segment["imu_z"][::stride], segment["marker_support"][::stride],
                segment["imu_support"][::stride],
                lw=0.65,
            )
            historical_correlations.append(segment["historical_corr"])
            if segment["support_corr"] is not None:
                visible_correlations.append(segment["support_corr"])
        visible_text = "NA" if not visible_correlations else f"{np.median(visible_correlations):.3f}"
        axis.set_title(
            f"{payload['subject']} · {len(payload['segments'])} segment(s) · "
            f"median r_hist={np.median(historical_correlations):.3f} · r_visible={visible_text}"
        )
        axis.set_ylim(-2.7, 6.2)
        axis.grid(alpha=0.15)
        axis.set_xlabel("MoCap time (s)")
        axis.set_ylabel("z activity")
    for axis in flat[len(payloads):]:
        axis.axis("off")
    handles = [
        Line2D([], [], color=MOCAP_COLOR, lw=1.7, label="MoCap marker-speed activity"),
        Line2D([], [], color=IMU_COLOR, lw=1.7, label="full-frame five-IMU activity mapped with adopted clock"),
        Patch(facecolor="#d62728", alpha=0.12, label="IMU support insufficient (trace hidden)"),
    ]
    fig.legend(handles=handles, loc="outside lower center", ncol=3, frameon=False)
    fig.suptitle("Corrected cohort: support-aware IMU–MoCap time-alignment overview (v2)", fontsize=19)
    fig.savefig(path.with_suffix(".png"), dpi=dpi)
    fig.savefig(path.with_suffix(".svg"))
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--subjects", nargs="*", default=bd.sessions())
    parser.add_argument(
        "--validation",
        type=Path,
        default=HERE / "runs/mapping_repair_2026-07-13/C_corrected_uniform/validation/cohort_validation.json",
    )
    parser.add_argument("--out-dir", type=Path, default=HERE / "runs/corrected_alignment_plots_2026-07-14_final")
    parser.add_argument("--summary-dpi", type=int, default=220)
    parser.add_argument("--detail-dpi", type=int, default=180)
    parser.add_argument("--chunk-s", type=float, default=120.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    subjects = [f"{int(subject):02d}" for subject in args.subjects]
    if sorted(subjects) != sorted(bd.sessions()):
        raise SystemExit(f"this frozen cohort run requires exactly {bd.sessions()}, got {subjects}")
    frozen_run = json.loads(MAPPING_RUN_MANIFEST.read_text(encoding="utf-8"))
    if frozen_run.get("status") != "canonical_frozen":
        raise SystemExit(f"mapping run is not canonical_frozen: {frozen_run.get('status')}")
    out_dir = args.out_dir.resolve()
    if out_dir.exists() and any(out_dir.iterdir()):
        raise SystemExit(f"refusing to overwrite non-empty output directory: {out_dir}")
    out_dir.mkdir(parents=True, exist_ok=True)
    plot_dir = out_dir / "plots"
    plot_dir.mkdir()
    validation_path = args.validation.resolve()
    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    if validation.get("run_type") != "corrected_uniform_validation":
        raise SystemExit(f"wrong validation authority: {validation.get('run_type')}")
    validation_hash = sha256_file(validation_path)
    frozen_validation_hash = frozen_run["C_corrected_uniform_validation"]["cohort_validation_sha256"]
    if validation_hash != frozen_validation_hash:
        raise SystemExit(f"corrected validation hash is not frozen authority: {validation_hash}")
    registry_hash = sha256_file(pm.DEFAULT_CONFIG_PATH.resolve())
    if not (
        registry_hash == frozen_run["mapping_registry"]["sha256"]
        == validation.get("mapping_registry_sha256")
    ):
        raise SystemExit("mapping registry hash differs across registry, frozen run, and corrected validation")

    payloads = []
    for subject in subjects:
        payload = build_subject(subject, validation)
        payloads.append(payload)
        plot_subject_summary(plot_dir / f"T{subject}_alignment_summary", payload, args.summary_dpi)
        plot_subject_detail(plot_dir / f"T{subject}_alignment_detail", payload, args.chunk_s, args.detail_dpi)
        plot_subject_actions(plot_dir / f"T{subject}_alignment_six_action", payload, args.summary_dpi)
    plot_cohort(plot_dir / "cohort_alignment_overview", payloads, args.summary_dpi)

    segment_rows = [row for payload in payloads for row in payload["segment_rows"]]
    block_rows = [row for payload in payloads for row in payload["block_rows"]]
    write_csv(out_dir / "segment_alignment_metrics.csv", segment_rows)
    write_csv(out_dir / "block_alignment_metrics.csv", block_rows)
    (out_dir / "alignment_metrics.json").write_text(
        json.dumps({"segments": segment_rows, "blocks": block_rows}, indent=2) + "\n",
        encoding="utf-8",
    )

    outputs = {}
    for path in sorted(out_dir.rglob("*")):
        if path.is_file() and path.name != "manifest.json":
            outputs[str(path.relative_to(out_dir))] = {
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
    manifest = {
        "schema_version": 2,
        "run_type": "corrected_alignment_plots_v2",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "subjects": [f"T{subject}" for subject in subjects],
        "unavailable_trials": bd.unavailable_sessions(),
        "mapping_registry": {
            "path": str(pm.DEFAULT_CONFIG_PATH.resolve()),
            "sha256": sha256_file(pm.DEFAULT_CONFIG_PATH.resolve()),
        },
        "validation_authority": {
            "path": str(validation_path),
            "sha256": sha256_file(validation_path),
        },
        "canonical_mapping_run": {
            "path": str(MAPPING_RUN_MANIFEST.resolve()),
            "sha256": sha256_file(MAPPING_RUN_MANIFEST.resolve()),
        },
        "generator": {
            "path": str(Path(__file__).resolve()),
            "sha256": sha256_file(Path(__file__).resolve()),
        },
        "calculation_dependencies": {
            str(Path(module.__file__).resolve()): sha256_file(Path(module.__file__).resolve())
            for module in (sa, v0, bd, pm)
        },
        "definition": {
            "clock_formula": "t_imu = a * t_mocap + b",
            "clock_policy": "use adopted per-trial/per-segment transforms; never refit from the plot",
            "historical_imu_activity": "smoothed mean gyro-vector magnitude across whichever physical IMU0-4 rows exist at each timestamp; retained only to reproduce the legacy unmasked sync statistic",
            "displayed_imu_activity": "the historical envelope is shown only where the nearest raw timestamp contains all five physical IMUs and is within max(0.10 s, 5 native median steps); unsupported intervals are NaN, never interpolated as valid coverage",
            "mocap_activity": "smoothed median speed across all available MoCap markers",
            "normalization": "each activity envelope z-scored before plotting",
            "mocap_support": "MoCap trace hidden and lane greyed where fewer than 50% of raw markers are finite",
            "imu_support": "IMU trace hidden/red-tinted unless all IMU0-4 rows are present at the nearest native timestamp; internal timestamp gaps beyond the nearest-frame threshold force zero support",
            "historical_unmasked_corr": "legacy activity correlation with partial IMU frames and gap-filled MoCap retained as a reproduction field, not the visible-data evidence metric",
            "support_aware_corr": "correlation only at full-five-IMU frames with at least 50% raw MoCap marker support",
            "summary_local_sweep": "support-aware whole-overlap correlation for delta-b in [-2,+2] s around adopted clock",
            "block_local_sweep": "support-aware block correlation for delta-b in [-0.5,+0.5] s; suppressed when canonical scored/detected coverage is below 50% or joint visible support is below 50%",
            "detail_chunk_s": args.chunk_s,
            "mapping_note": "time alignment is anatomical-mapping agnostic, but this run is bound to corrected cohort provenance",
        },
        "counts": {
            "subjects": len(payloads),
            "segments": len(segment_rows),
            "blocks": len(block_rows),
            "summary_figures": len(payloads),
            "detail_figures": len(payloads),
            "six_action_figures": len(payloads),
            "cohort_figures": 1,
        },
        "trials": {
            payload["trial_id"]: {
                "placement_status": payload["placement_status"],
                "mapping_sha256": payload["mapping_sha256"],
                "role_to_imu": payload["role_to_imu"],
                "legacy_manifest_status": payload["legacy_manifest_status"],
                "corrected_subject_authority": payload["corrected_subject_authority"],
                "segments": [row["segment"] for row in payload["segment_rows"]],
                "input_files": payload["input_files"],
            }
            for payload in payloads
        },
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "numpy": np.__version__,
        },
        "outputs": outputs,
        "claim_boundaries": [
            "activity-envelope overlay audits clock phase only; it is not an angular-accuracy plot",
            "historical unmasked correlation may include partial IMU frames and gap-filled MoCap; use support-aware correlation for visible-data evidence",
            "high correlation does not validate anatomical placement or IMU/MoCap angle scale",
            "repetitive movement can create correlation aliases; adopted clocks remain the frozen authority",
            "T01 and T07 remain unavailable rather than being imputed",
        ],
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest["counts"], indent=2))
    print(f"wrote {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
