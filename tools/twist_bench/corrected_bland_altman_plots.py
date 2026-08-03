from __future__ import annotations

"""Generate corrected-cohort bout-level Bland–Altman plots.

The paired measurements exactly mirror the frozen corrected-uniform validation:
each scored bout is locally re-tared, sampled on the same 200-point MoCap grid,
and compared either at native IMU scale or after true leave-one-repetition-out
calibration.  The Bland–Altman point is the synchronized MoCap-peak sample,
not independently selected peaks and not 200 autocorrelated points per bout.
"""

import argparse
import csv
import hashlib
import json
import math
import platform
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import corrected_validation_rebuild as cvr  # noqa: E402
import five_imu_fusion as fiv  # noqa: E402
import dataset_adapter as mlb  # type: ignore  # noqa: E402
import placement_maps as pm  # noqa: E402
import session_recipe as t06  # noqa: E402
import signed_diagnostic as sd  # noqa: E402
import sync_audit as sa  # noqa: E402


# Cohort comes from the dataset config, not from a constant here -- a hard-coded
# session list makes the script describe one study rather than one method.
ACTIONS = ["flexion", "extension", "left_bend", "right_bend", "left_twist", "right_twist"]
ACTION_LABELS = {
    "flexion": "Flexion",
    "extension": "Extension",
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
QUALITY_MARKERS = {"clean": "o", "low_conf": "s"}
MAPPING_RUN = HERE / "runs/mapping_repair_2026-07-13"
VALIDATION_PATH = MAPPING_RUN / "C_corrected_uniform/validation/cohort_validation.json"


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


def canonical_marker_support(
    subject: str,
    auth_subject: dict[str, Any],
) -> dict[tuple[str, int], dict[str, float | bool]]:
    """Independent raw-marker support for the canonical MoCap target windows."""
    time_s, markers = sa.parse_motive_markers(mlb.mocap_path(subject), gap_fill=False)
    prefix = "Trunk:"
    sacral_name = next((name for name in ("S2", "S1") if prefix + name in markers), None)
    if sacral_name is None:
        raise KeyError(f"T{subject}: no S2/S1 marker")
    common = ["L3", sacral_name, "LPSIS_B", "RPSIS_B", "LPSIS_F", "RPSIS_F"]
    lookup: dict[tuple[str, int], dict[str, float | bool]] = {}
    for block in auth_subject["blocks"]:
        is_twist = block["label"] in {"left_twist", "right_twist"}
        required = common + (["JN", "XP"] if is_twist else ["C7 (2)", "T2"])
        valid = np.ones(len(time_s), dtype=bool)
        for marker in required:
            key = prefix + marker
            if key not in markers:
                raise KeyError(f"T{subject}: missing required canonical marker {key}")
            valid &= np.isfinite(markers[key]).all(axis=1)
        for index, (lo, hi) in enumerate(block["scored_bouts"]):
            neutral_mask = (time_s >= float(lo) - 1.2) & (time_s <= float(lo) - 0.2)
            movement_mask = (time_s >= float(lo)) & (time_s <= float(hi))
            neutral = float(np.mean(valid[neutral_mask])) if np.any(neutral_mask) else 0.0
            movement = float(np.mean(valid[movement_mask])) if np.any(movement_mask) else 0.0
            lookup[(block["block"], index)] = {
                "neutral": neutral,
                "movement": movement,
                "accepted": min(neutral, movement) >= 0.50,
            }
    return lookup


def paired_bout_arrays(
    *,
    res: Any,
    a: float,
    b: float,
    tm: np.ndarray,
    signal: np.ndarray,
    bouts: list[tuple[float, float]],
    is_twist: bool,
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    xs: list[np.ndarray] = []
    ys: list[np.ndarray] = []
    for lo, hi in bouts:
        series = t06.local_twist(res, a, b, lo) if is_twist else t06.local_swing(res, a, b, lo)
        grid = np.linspace(float(lo), float(hi), 200, endpoint=False)
        pre = np.linspace(float(lo - 1.2), float(lo - 0.2), 100, endpoint=False)
        mocap_zero = float(np.mean(np.interp(pre, tm, signal)))
        imu_zero = float(np.mean(np.interp(a * pre + b, res.t_s, series)))
        y = np.interp(grid, tm, signal) - mocap_zero
        x = np.interp(a * grid + b, res.t_s, series) - imu_zero
        if not is_twist:
            y = np.abs(y)
        if not (np.all(np.isfinite(x)) and np.all(np.isfinite(y))):
            raise ValueError(f"non-finite paired array for bout {lo:.3f}..{hi:.3f}")
        xs.append(x)
        ys.append(y)
    return xs, ys


def reconstruct_pairs(validation: dict[str, Any]) -> tuple[list[dict[str, Any]], set[Path]]:
    rows: list[dict[str, Any]] = []
    input_paths: set[Path] = {VALIDATION_PATH.resolve()}
    for subject in mlb.sessions():
        trial = f"T{subject}_P{subject}"
        tm, flex, lat, axial = sd.mocap_signed(mlb.mocap_path(subject))
        signals = {"flex": flex, "lat": lat, "axial": axial}
        auth_subject = validation["subjects"][trial]
        marker_lookup = canonical_marker_support(subject, auth_subject)
        auth_blocks = {block["block"]: block for block in auth_subject["blocks"]}
        subject_path = MAPPING_RUN / "C_corrected_uniform/validation/subjects" / f"{trial}.json"
        if json.loads(subject_path.read_text(encoding="utf-8")) != auth_subject:
            raise ValueError(f"{trial}: standalone subject authority differs from cohort authority")
        input_paths.update({mlb.mocap_path(subject).resolve(), mlb.manifest_path(subject).resolve(), subject_path.resolve()})
        seen: set[str] = set()

        for block_id, block, res, a, b, detected_bouts in mlb.subject_blocks(subject, tm, signals):
            if block_id not in auth_blocks:
                raise KeyError(f"{trial}/{block_id} missing from corrected authority")
            auth = auth_blocks[block_id]
            label = str(auth["label"])
            if label not in ACTIONS:
                continue
            if not (
                math.isclose(float(auth["clock"]["a"]), float(a), abs_tol=1e-12)
                and math.isclose(float(auth["clock"]["b"]), float(b), abs_tol=1e-12)
            ):
                raise ValueError(f"{trial}/{block_id}: clock differs from corrected authority")
            auth_detected = np.asarray(auth["detected_bouts"], dtype=float)
            detected = np.asarray(detected_bouts, dtype=float)
            if auth_detected.shape != detected.shape or not np.allclose(auth_detected, detected, atol=1e-6):
                raise ValueError(f"{trial}/{block_id}: detected bouts differ from frozen authority")
            scored_bouts = [(float(lo), float(hi)) for lo, hi in auth["scored_bouts"]]
            override = mlb.BLOCK_OVERRIDES.get(subject, {}).get(block_id, {})
            signal_key = override.get(
                "sig", block.get("mocap_signal", block.get("primary_signal", mlb.SIG_BY_LABEL[label]))
            )
            xs, ys = paired_bout_arrays(
                res=res, a=float(a), b=float(b), tm=tm, signal=signals[signal_key],
                bouts=scored_bouts, is_twist=label in {"left_twist", "right_twist"},
            )
            if len(xs) != int(auth["n_bouts_scored"]):
                raise ValueError(f"{trial}/{block_id}: reconstructed scored bout count differs")

            fold_predictions: list[np.ndarray | None] = [None] * len(xs)
            fold_gains: list[float | None] = [None] * len(xs)
            fold_intercepts: list[float | None] = [None] * len(xs)
            fold_rmses: list[float] = []
            if len(xs) >= 2:
                for index in range(len(xs)):
                    train_x = np.concatenate([xs[i] for i in range(len(xs)) if i != index])
                    train_y = np.concatenate([ys[i] for i in range(len(ys)) if i != index])
                    gain, intercept = np.polyfit(train_x, train_y, 1)
                    prediction = gain * xs[index] + intercept
                    fold_predictions[index] = prediction
                    fold_gains[index] = float(gain)
                    fold_intercepts[index] = float(intercept)
                    fold_rmses.append(float(np.sqrt(np.mean(np.square(ys[index] - prediction)))))
                expected_rmses = auth["corrected_metric_full"].get("per_bout_heldout_rmse_deg", [])
                if len(expected_rmses) != len(fold_rmses) or not np.allclose(
                    fold_rmses, np.asarray(expected_rmses, dtype=float), atol=5.1e-5
                ):
                    raise ValueError(f"{trial}/{block_id}: LORO folds do not reproduce frozen validation")

            raw_input = Path(str(res.summary.get("input", "")))
            if raw_input.is_file():
                input_paths.add(raw_input.resolve())
            for index, ((lo, hi), x, y) in enumerate(zip(scored_bouts, xs, ys)):
                peak_index = int(np.argmax(np.abs(y)))
                mocap_peak = float(y[peak_index])
                native_peak = float(x[peak_index])
                loro_prediction = fold_predictions[index]
                loro_peak = None if loro_prediction is None else float(loro_prediction[peak_index])
                support = marker_lookup.get((block_id, index))
                if support is None:
                    raise KeyError(f"{trial}/{block_id}/{index}: missing raw marker support record")
                rows.append({
                    "subject": f"T{subject}",
                    "trial_id": trial,
                    "block": block_id,
                    "action": label,
                    "quality": str(auth["quality"]),
                    "bout_index": index,
                    "bout_start_mocap_s": round(lo, 6),
                    "bout_end_mocap_s": round(hi, 6),
                    "clock_a": round(float(a), 8),
                    "clock_b_s": round(float(b), 6),
                    "mocap_peak_deg": round(mocap_peak, 9),
                    "imu_native_at_mocap_peak_deg": round(native_peak, 9),
                    "native_ba_mean_deg": round((native_peak + mocap_peak) / 2.0, 9),
                    "native_ba_difference_deg": round(native_peak - mocap_peak, 9),
                    "loro_gain": None if fold_gains[index] is None else round(float(fold_gains[index]), 9),
                    "loro_intercept_deg": (
                        None if fold_intercepts[index] is None else round(float(fold_intercepts[index]), 9)
                    ),
                    "imu_loro_at_mocap_peak_deg": None if loro_peak is None else round(loro_peak, 9),
                    "loro_ba_mean_deg": (
                        None if loro_peak is None else round((loro_peak + mocap_peak) / 2.0, 9)
                    ),
                    "loro_ba_difference_deg": (
                        None if loro_peak is None else round(loro_peak - mocap_peak, 9)
                    ),
                    "raw_marker_neutral_support_fraction": round(float(support["neutral"]), 6),
                    "raw_marker_movement_support_fraction": round(float(support["movement"]), 6),
                    "raw_marker_support_qc": "pass" if support["accepted"] else "fail",
                })
            seen.add(block_id)
        if seen != set(auth_blocks):
            raise ValueError(f"{trial}: block mismatch seen={sorted(seen)} auth={sorted(auth_blocks)}")
    if len(rows) != 1299:
        raise ValueError(f"expected 1299 scored bout pairs, got {len(rows)}")
    if sum(row["imu_loro_at_mocap_peak_deg"] is not None for row in rows) != 1296:
        raise ValueError("expected 1296 true-LORO bout pairs")
    if sum(row["raw_marker_support_qc"] == "pass" for row in rows) != 1185:
        raise ValueError("expected 1185 raw-marker-support sensitivity bouts")
    return rows, input_paths


def mode_values(row: dict[str, Any], mode: str, *, absolute: bool = False) -> tuple[float, float] | None:
    mocap = float(row["mocap_peak_deg"])
    if mode == "native":
        imu = float(row["imu_native_at_mocap_peak_deg"])
    else:
        value = row["imu_loro_at_mocap_peak_deg"]
        if value is None:
            return None
        imu = float(value)
    if absolute:
        return abs(mocap), abs(imu)
    return mocap, imu


def repeated_loa(pairs: list[tuple[str, float, float]]) -> dict[str, float | int | None]:
    grouped: dict[str, list[float]] = defaultdict(list)
    all_x: list[float] = []
    all_y: list[float] = []
    all_mean: list[float] = []
    for subject, mocap, imu in pairs:
        grouped[subject].append(imu - mocap)
        all_x.append(mocap)
        all_y.append(imu)
        all_mean.append((mocap + imu) / 2.0)
    subject_arrays = [np.asarray(values, dtype=float) for values in grouped.values()]
    if not subject_arrays:
        return {"n_subjects": 0, "n_bouts": 0, "bias_deg": None, "lower_loa_deg": None, "upper_loa_deg": None}
    subject_means = np.asarray([float(np.mean(values)) for values in subject_arrays])
    bias = float(np.mean(subject_means))
    within_ss = float(sum(np.sum(np.square(values - np.mean(values))) for values in subject_arrays))
    within_df = sum(len(values) - 1 for values in subject_arrays)
    within_variance = within_ss / within_df if within_df > 0 else 0.0
    mean_variance = float(np.var(subject_means, ddof=1)) if len(subject_means) > 1 else 0.0
    corrections = [float(np.var(values, ddof=1)) / len(values) for values in subject_arrays if len(values) > 1]
    between_variance = max(0.0, mean_variance - (float(np.mean(corrections)) if corrections else 0.0))
    total_sd = math.sqrt(within_variance + between_variance)
    x = np.asarray(all_x)
    y = np.asarray(all_y)
    d = y - x
    centered_mean = []
    centered_difference = []
    for subject, values in grouped.items():
        indices = [i for i, pair in enumerate(pairs) if pair[0] == subject]
        means = np.asarray([all_mean[i] for i in indices])
        diffs = np.asarray(values)
        centered_mean.extend(means - np.mean(means))
        centered_difference.extend(diffs - np.mean(diffs))
    slope = None
    if len(centered_mean) >= 3 and np.std(centered_mean) > 1e-9:
        slope = float(np.polyfit(np.asarray(centered_mean), np.asarray(centered_difference), 1)[0])
    covariance = float(np.cov(x, y, ddof=1)[0, 1]) if len(x) > 1 else 0.0
    ccc_denom = float(np.var(x, ddof=1) + np.var(y, ddof=1) + (np.mean(x) - np.mean(y)) ** 2) if len(x) > 1 else 0.0
    return {
        "n_subjects": len(grouped),
        "n_bouts": len(pairs),
        "bias_deg": bias,
        "within_subject_sd_deg": math.sqrt(within_variance),
        "between_subject_sd_deg": math.sqrt(between_variance),
        "total_sd_deg": total_sd,
        "lower_loa_deg": bias - 1.96 * total_sd,
        "upper_loa_deg": bias + 1.96 * total_sd,
        "mae_deg": float(np.mean(np.abs(d))),
        "rmse_deg": float(np.sqrt(np.mean(np.square(d)))),
        "ccc": (2.0 * covariance / ccc_denom) if ccc_denom > 1e-12 else None,
        "proportional_bias_within_subject_slope": slope,
    }


def bootstrap_ci(
    pairs: list[tuple[str, float, float]], *, seed: int, n_bootstrap: int
) -> dict[str, float | None]:
    grouped: dict[str, list[tuple[float, float]]] = defaultdict(list)
    for subject, mocap, imu in pairs:
        grouped[subject].append((mocap, imu))
    subjects = sorted(grouped)
    if len(subjects) < 2 or n_bootstrap <= 0:
        return {key: None for key in ("bias_ci_low", "bias_ci_high", "lower_loa_ci_low", "lower_loa_ci_high", "upper_loa_ci_low", "upper_loa_ci_high")}
    rng = np.random.default_rng(seed)
    estimates = []
    for _ in range(n_bootstrap):
        sampled = rng.choice(subjects, size=len(subjects), replace=True)
        bootstrap_pairs = [
            (f"cluster_{draw}", mocap, imu)
            for draw, subject in enumerate(sampled)
            for mocap, imu in grouped[str(subject)]
        ]
        stat = repeated_loa(bootstrap_pairs)
        estimates.append([stat["bias_deg"], stat["lower_loa_deg"], stat["upper_loa_deg"]])
    values = np.asarray(estimates, dtype=float)
    intervals = np.percentile(values, [2.5, 97.5], axis=0)
    return {
        "bias_ci_low": float(intervals[0, 0]),
        "bias_ci_high": float(intervals[1, 0]),
        "lower_loa_ci_low": float(intervals[0, 1]),
        "lower_loa_ci_high": float(intervals[1, 1]),
        "upper_loa_ci_low": float(intervals[0, 2]),
        "upper_loa_ci_high": float(intervals[1, 2]),
    }


def build_stats(rows: list[dict[str, Any]], n_bootstrap: int) -> list[dict[str, Any]]:
    sets = {
        "all_scored": lambda row: True,
        "raw_marker_support_qc": lambda row: row["raw_marker_support_qc"] == "pass",
        "clean_plus_low_conf_sensitivity": lambda row: row["quality"] in {"clean", "low_conf"},
    }
    output = []
    for mode in ("native", "loro"):
        for set_name, predicate in sets.items():
            for action in ACTIONS + ["overall_absolute_amplitude"]:
                absolute = action == "overall_absolute_amplitude"
                pairs = []
                for row in rows:
                    if not predicate(row) or (not absolute and row["action"] != action):
                        continue
                    values = mode_values(row, mode, absolute=absolute)
                    if values is None:
                        continue
                    mocap, imu = values
                    pairs.append((row["subject"], mocap, imu))
                stat = repeated_loa(pairs)
                seed = int(hashlib.sha256(f"{mode}|{set_name}|{action}".encode()).hexdigest()[:8], 16)
                ci = bootstrap_ci(pairs, seed=seed, n_bootstrap=n_bootstrap)
                output.append({
                    "mode": mode,
                    "analysis_set": set_name,
                    "action": action,
                    **{key: (round(value, 9) if isinstance(value, float) else value) for key, value in stat.items()},
                    **{key: (round(value, 9) if isinstance(value, float) else value) for key, value in ci.items()},
                    "bootstrap_clusters": int(stat["n_subjects"] or 0),
                    "bootstrap_replicates": n_bootstrap,
                    "loa_method": "subject-balanced random-intercept variance decomposition",
                })
    return output


def subject_colors() -> dict[str, Any]:
    cmap = plt.get_cmap("tab20")
    return {f"T{subject}": cmap(index) for index, subject in enumerate(mlb.sessions())}


def quality_marker(quality: str) -> str:
    return QUALITY_MARKERS.get(quality, "X")


def save_figure(fig, path: Path, dpi: int) -> None:
    fig.savefig(path.with_suffix(".png"), dpi=dpi)
    fig.savefig(path.with_suffix(".svg"))
    plt.close(fig)


def plot_cohort_actions(
    path: Path,
    rows: list[dict[str, Any]],
    stats: list[dict[str, Any]],
    *,
    mode: str,
    analysis_set: str,
    dpi: int,
) -> None:
    colors = subject_colors()
    if analysis_set == "all_scored":
        predicate = lambda row: True
    elif analysis_set == "raw_marker_support_qc":
        predicate = lambda row: row["raw_marker_support_qc"] == "pass"
    elif analysis_set == "clean_plus_low_conf_sensitivity":
        predicate = lambda row: row["quality"] in {"clean", "low_conf"}
    else:
        raise ValueError(f"unknown analysis set: {analysis_set}")
    lookup = {(row["mode"], row["analysis_set"], row["action"]): row for row in stats}
    fig, axes = plt.subplots(2, 3, figsize=(22, 13), constrained_layout=True)
    for axis, action in zip(axes.ravel(), ACTIONS):
        panel = []
        for row in rows:
            if row["action"] != action or not predicate(row):
                continue
            values = mode_values(row, mode)
            if values is None:
                continue
            mocap, imu = values
            mean = (mocap + imu) / 2.0
            difference = imu - mocap
            panel.append((row, mean, difference))
            axis.scatter(
                mean, difference, s=24, marker=quality_marker(row["quality"]),
                color=colors[row["subject"]], alpha=0.78, linewidths=0.35, edgecolors="#333333",
            )
        stat = lookup[(mode, analysis_set, action)]
        if stat["bias_deg"] is not None:
            axis.axhline(float(stat["bias_deg"]), color="#b22222", lw=1.5)
            axis.axhline(float(stat["lower_loa_deg"]), color="#333333", lw=1.2, linestyle="--")
            axis.axhline(float(stat["upper_loa_deg"]), color="#333333", lw=1.2, linestyle="--")
        axis.axhline(0.0, color="#777777", lw=0.8, alpha=0.5)
        axis.set_title(
            f"{ACTION_LABELS[action]} · {stat['n_subjects']} subjects / {stat['n_bouts']} bouts\n"
            f"bias={stat['bias_deg']:.2f}° · LoA [{stat['lower_loa_deg']:.2f}, {stat['upper_loa_deg']:.2f}]°"
        )
        axis.set_xlabel("Mean of MoCap and IMU (deg)")
        axis.set_ylabel("IMU − MoCap (deg)")
        axis.grid(alpha=0.16)
    subject_handles = [
        Line2D([], [], color=colors[f"T{s}"], marker="o", linestyle="", label=f"T{s}") for s in mlb.sessions()
    ]
    quality_handles = [
        Line2D([], [], color="#555555", marker="o", linestyle="", label="clean"),
        Line2D([], [], color="#555555", marker="s", linestyle="", label="low_conf"),
        Line2D([], [], color="#555555", marker="X", linestyle="", label="invalid quality (still scored)"),
    ]
    fig.legend(handles=subject_handles + quality_handles, loc="outside lower center", ncol=8, frameon=False)
    mode_title = "native/raw IMU (no gain correction)" if mode == "native" else "true leave-one-bout-out calibrated IMU"
    set_title = {
        "all_scored": "all 13-person coverage-scored bouts",
        "raw_marker_support_qc": "window-support sensitivity subset (raw canonical markers)",
        "clean_plus_low_conf_sensitivity": "quality sensitivity subset (clean + low_conf)",
    }[analysis_set]
    fig.suptitle(f"Corrected cohort Bland–Altman: {mode_title}\n{set_title}", fontsize=17)
    save_figure(fig, path, dpi)


def plot_subject_actions(path: Path, rows: list[dict[str, Any]], subject: str, dpi: int) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(22, 12.5), constrained_layout=True)
    for axis, action in zip(axes.ravel(), ACTIONS):
        panel = []
        for row in rows:
            if row["subject"] != subject or row["action"] != action:
                continue
            values = mode_values(row, "loro")
            if values is None:
                continue
            mocap, imu = values
            panel.append((mocap + imu, imu - mocap, row))
        if panel:
            means = np.asarray([item[0] / 2.0 for item in panel])
            differences = np.asarray([item[1] for item in panel])
            for quality in ("clean", "low_conf", "invalid_quality", "invalid_insufficient_repetitions"):
                subset = [item for item in panel if item[2]["quality"] == quality]
                if not subset:
                    continue
                axis.scatter(
                    [item[0] / 2.0 for item in subset], [item[1] for item in subset],
                    s=28, color=ACTION_COLORS[action], alpha=0.82,
                    marker=quality_marker(quality), edgecolors="#333333", linewidths=0.35,
                    label=quality,
                )
            bias = float(np.mean(differences))
            sd = float(np.std(differences, ddof=1)) if len(differences) > 1 else 0.0
            axis.axhline(bias, color="#b22222", lw=1.4)
            axis.axhline(bias - 1.96 * sd, color="#333333", lw=1.1, linestyle="--")
            axis.axhline(bias + 1.96 * sd, color="#333333", lw=1.1, linestyle="--")
            quality_text = ", ".join(sorted({item[2]["quality"] for item in panel}))
            title = f"{ACTION_LABELS[action]} · n={len(panel)} · quality={quality_text}\nbias={bias:.2f}° · descriptive LoA [{bias-1.96*sd:.2f}, {bias+1.96*sd:.2f}]°"
        else:
            title = f"{ACTION_LABELS[action]} · LORO unavailable"
            axis.text(0.5, 0.5, "No ≥2-bout calibrated comparison", transform=axis.transAxes, ha="center", va="center")
        axis.axhline(0.0, color="#777777", lw=0.8, alpha=0.5)
        axis.set_title(title)
        axis.set_xlabel("Mean of MoCap and LORO IMU (deg)")
        axis.set_ylabel("LORO IMU − MoCap (deg)")
        axis.grid(alpha=0.16)
    fig.suptitle(
        f"{subject}: true leave-one-bout-out peak Bland–Altman\n"
        "dynamic local-tared angle change; not absolute posture or thoracic curvature",
        fontsize=17,
    )
    quality_handles = [
        Line2D([], [], color="#555555", marker=quality_marker(quality), linestyle="", label=quality)
        for quality in ("clean", "low_conf", "invalid_quality", "invalid_insufficient_repetitions")
        if any(row["subject"] == subject and row["quality"] == quality for row in rows)
    ]
    if quality_handles:
        fig.legend(handles=quality_handles, loc="outside lower center", ncol=len(quality_handles), frameon=False)
    save_figure(fig, path, dpi)


def plot_subject_overview(path: Path, rows: list[dict[str, Any]], dpi: int) -> None:
    fig, axes = plt.subplots(5, 3, figsize=(22, 28), constrained_layout=True)
    for axis, subject in zip(axes.ravel(), [f"T{s}" for s in mlb.sessions()]):
        panel = []
        for row in rows:
            if row["subject"] != subject:
                continue
            values = mode_values(row, "loro", absolute=True)
            if values is None:
                continue
            mocap, imu = values
            panel.append((row, (mocap + imu) / 2.0, imu - mocap))
        for action in ACTIONS:
            subset = [item for item in panel if item[0]["action"] == action]
            if subset:
                axis.scatter(
                    [item[1] for item in subset], [item[2] for item in subset],
                    s=20, color=ACTION_COLORS[action], alpha=0.76, label=ACTION_LABELS[action],
                )
        differences = np.asarray([item[2] for item in panel])
        if len(differences):
            bias = float(np.mean(differences))
            sd = float(np.std(differences, ddof=1)) if len(differences) > 1 else 0.0
            axis.axhline(bias, color="#b22222", lw=1.3)
            axis.axhline(bias - 1.96 * sd, color="#333333", lw=1.0, linestyle="--")
            axis.axhline(bias + 1.96 * sd, color="#333333", lw=1.0, linestyle="--")
            axis.set_title(f"{subject} · n={len(panel)} · bias={bias:.2f}° · descriptive within-subject band [{bias-1.96*sd:.2f}, {bias+1.96*sd:.2f}]°")
        axis.axhline(0.0, color="#777777", lw=0.7, alpha=0.5)
        axis.set_xlabel("Mean absolute peak amplitude (deg)")
        axis.set_ylabel("|LORO IMU| − |MoCap| (deg)")
        axis.grid(alpha=0.15)
    for axis in axes.ravel()[len(mlb.sessions()):]:
        axis.axis("off")
    handles = [Line2D([], [], color=ACTION_COLORS[action], marker="o", linestyle="", label=ACTION_LABELS[action]) for action in ACTIONS]
    fig.legend(handles=handles, loc="outside lower center", ncol=6, frameon=False)
    fig.suptitle(
        "All 13 subjects: LORO Bland–Altman absolute peak-amplitude overview\n"
        "secondary cross-action view; primary inference remains action-specific",
        fontsize=18,
    )
    save_figure(fig, path, dpi)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=HERE / "runs/corrected_bland_altman_2026-07-14_final")
    parser.add_argument("--bootstrap", type=int, default=10000)
    parser.add_argument("--dpi", type=int, default=220)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    out_dir = args.out_dir.resolve()
    if out_dir.exists() and any(out_dir.iterdir()):
        raise SystemExit(f"refusing to overwrite non-empty output directory: {out_dir}")
    frozen = json.loads((MAPPING_RUN / "run_manifest.json").read_text(encoding="utf-8"))
    if frozen.get("status") != "canonical_frozen":
        raise SystemExit("mapping repair run is not canonical_frozen")
    validation_hash = sha256_file(VALIDATION_PATH)
    if validation_hash != frozen["C_corrected_uniform_validation"]["cohort_validation_sha256"]:
        raise SystemExit("corrected validation is not the frozen authority")
    registry_hash = sha256_file(pm.DEFAULT_CONFIG_PATH.resolve())
    if registry_hash != frozen["mapping_registry"]["sha256"]:
        raise SystemExit("placement registry is not the frozen authority")
    validation = json.loads(VALIDATION_PATH.read_text(encoding="utf-8"))
    out_dir.mkdir(parents=True)
    plot_dir = out_dir / "plots"
    plot_dir.mkdir()

    pairs, input_paths = reconstruct_pairs(validation)
    stats = build_stats(pairs, args.bootstrap)
    write_csv(out_dir / "paired_bout_peaks.csv", pairs)
    write_csv(out_dir / "bland_altman_stats.csv", stats)
    (out_dir / "bland_altman_stats.json").write_text(json.dumps(stats, indent=2) + "\n", encoding="utf-8")

    plot_cohort_actions(
        plot_dir / "cohort_native_peak_bland_altman", pairs, stats,
        mode="native", analysis_set="all_scored", dpi=args.dpi,
    )
    plot_cohort_actions(
        plot_dir / "cohort_loro_peak_bland_altman", pairs, stats,
        mode="loro", analysis_set="all_scored", dpi=args.dpi,
    )
    plot_cohort_actions(
        plot_dir / "cohort_loro_window_support_bland_altman", pairs, stats,
        mode="loro", analysis_set="raw_marker_support_qc", dpi=args.dpi,
    )
    plot_cohort_actions(
        plot_dir / "cohort_loro_quality_accepted_bland_altman", pairs, stats,
        mode="loro", analysis_set="clean_plus_low_conf_sensitivity", dpi=args.dpi,
    )
    plot_subject_overview(plot_dir / "all_subjects_loro_absolute_peak_overview", pairs, args.dpi)
    for subject in [f"T{s}" for s in mlb.sessions()]:
        plot_subject_actions(plot_dir / f"{subject}_loro_peak_bland_altman", pairs, subject, args.dpi)

    outputs = {}
    for path in sorted(out_dir.rglob("*")):
        if path.is_file() and path.name != "manifest.json":
            outputs[str(path.relative_to(out_dir))] = {"sha256": sha256_file(path), "size_bytes": path.stat().st_size}
    dependency_paths = [Path(module.__file__).resolve() for module in (cvr, fiv, mlb, pm, t06, sd, sa)]
    for dependency_name in ("twist_bench_fusion.py", "validation3_cluster_orientation.py", "validation_t01.py"):
        dependency_path = HERE / dependency_name
        if dependency_path.is_file() and dependency_path not in dependency_paths:
            dependency_paths.append(dependency_path)
    manifest = {
        "schema_version": 1,
        "run_type": "corrected_bland_altman_bout_peak_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "subjects": [f"T{s}" for s in mlb.sessions()],
        "unavailable_trials": mlb.unavailable_sessions(),
        "frozen_mapping_run": {"path": str((MAPPING_RUN / "run_manifest.json").resolve()), "sha256": sha256_file(MAPPING_RUN / "run_manifest.json")},
        "validation_authority": {"path": str(VALIDATION_PATH.resolve()), "sha256": validation_hash},
        "mapping_registry": {"path": str(pm.DEFAULT_CONFIG_PATH.resolve()), "sha256": registry_hash},
        "generator": {"path": str(Path(__file__).resolve()), "sha256": sha256_file(Path(__file__).resolve())},
        "calculation_dependencies": {str(path): sha256_file(path) for path in dependency_paths},
        "definition": {
            "comparison_unit": "one synchronized MoCap-peak pair per coverage-scored bout",
            "mocap_peak_selection": "index of maximum absolute locally-zeroed MoCap target on the frozen 200-point bout grid; IMU is evaluated at the same index",
            "bend": "MoCap absolute flexion/lateral excursion vs local-tared yaw-masked IMU1 sacrum to IMU4 upper swing magnitude",
            "twist": "MoCap signed sacrum-to-sternum axial excursion vs local-tared IMU1 sacrum to IMU0 sternum signed twist",
            "native_difference": "native IMU minus MoCap",
            "loro_difference": "true leave-one-bout-out calibrated IMU prediction minus MoCap",
            "calibration_boundary": "within subject and action block; every evaluated bout is excluded from its own gain/intercept fit",
            "loa": "subject-balanced random-intercept variance decomposition: bias +/- 1.96*sqrt(between-subject variance + within-subject variance)",
            "confidence_intervals": f"95% percentile intervals from {args.bootstrap} subject-cluster bootstrap replicates",
            "marker_support_sensitivity": "window-support sensitivity, independently recomputed from gap_fill=False canonical target markers; requires >=50% all-required-marker frame support in both neutral and movement windows; this is not a peak-frame QC gate",
            "quality_sensitivity": "secondary subset restricted to clean and low_conf validation quality labels; primary all-scored result remains unfiltered",
            "bend_marker_set": "L3, S2-or-S1, LPSIS_B, RPSIS_B, C7 (2), T2, LPSIS_F, RPSIS_F",
            "twist_marker_set": "JN, XP, L3, S2-or-S1, LPSIS_B, RPSIS_B, LPSIS_F, RPSIS_F",
        },
        "counts": {
            "scored_bout_pairs_native": len(pairs),
            "scored_bout_pairs_loro": sum(row["imu_loro_at_mocap_peak_deg"] is not None for row in pairs),
            "marker_support_qc_native": sum(row["raw_marker_support_qc"] == "pass" for row in pairs),
            "stats_rows": len(stats),
            "plot_bases": 18,
        },
        "parameters": {
            "bootstrap_replicates": args.bootstrap,
            "dpi": args.dpi,
            "peak_grid_points": 200,
            "neutral_grid_points": 100,
            "marker_support_threshold": 0.50,
        },
        "input_files": [
            {"path": str(path), "sha256": sha256_file(path), "size_bytes": path.stat().st_size}
            for path in sorted(input_paths, key=str)
        ],
        "outputs": outputs,
        "environment": {"python": sys.version, "platform": platform.platform(), "numpy": np.__version__, "matplotlib": matplotlib.__version__},
        "claim_boundaries": [
            "angles are dynamic neutral-referenced movement changes, not absolute posture, thoracic curvature, vertebral angle, or Cobb angle",
            "MoCap is a surface-marker-derived reference, not a direct joint or vertebral measurement",
            "LORO agreement is within-session and within-action calibration performance, not calibration-free or cross-subject deployment performance",
            "quality labels are shown but all coverage-scored bouts remain in the primary analysis to avoid outcome-dependent filtering",
            "T01 and T07 remain unavailable rather than imputed",
        ],
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest["counts"], indent=2))
    print(f"wrote {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
