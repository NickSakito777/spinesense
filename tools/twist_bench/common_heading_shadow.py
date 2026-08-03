from __future__ import annotations

"""T08 shadow probe for multi-IMU common-heading calibration.

This script is deliberately isolated from ``data_clean`` and the canonical
pipeline.  It tests whether a neutral-pose (T0) heading reset can remove the
independent world-heading gauges of magnetometer-free IMUs before relative
quaternions are formed.

Four arms are compared on the same held-out T08 bouts:

1. canonical gravity-up alignment + relation-first local re-tare;
2. qmt-style neutral common-heading reset to the sacrum;
3. per-sensor neutral-first motion (an algebraic gauge-invariance control);
4. two-motion functional sensor-to-segment calibration followed by the
   common-heading reset.

The probe also contains two synthetic gates.  One injects independent left
world-yaw gauges.  The other checks rigid co-motion for differently mounted
sensors and demonstrates why heading reset is not a substitute for a proper
right-side sensor-to-segment calibration.
"""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import functional_frame_shadow as ffs  # noqa: E402
import signed_diagnostic as sd  # noqa: E402
import twist_bench_fusion as pf  # noqa: E402
import dataset_adapter as mlb  # type: ignore  # noqa: E402


EPS = 1e-10
GAUGE_GATE_TOL_DEG = 1e-4
ROLES = ("sacrum", "lower", "mid", "upper", "sternum")
HEADLINE_RELATIONS = {
    "bend": ("sacrum", "upper"),
    "twist": ("sacrum", "sternum"),
}
GAUGE_RELATIONS = {
    "sacrum_to_lower": ("sacrum", "lower"),
    "lower_to_mid": ("lower", "mid"),
    "mid_to_upper": ("mid", "upper"),
    "sacrum_to_upper": ("sacrum", "upper"),
    "sacrum_to_sternum": ("sacrum", "sternum"),
    "upper_to_sternum": ("upper", "sternum"),
}
METHODS = (
    "canonical_relation_first",
    "t0_common_heading",
    "neutral_first_identity",
    "functional_then_t0_heading",
)
FROZEN_QUALITY_PATH = (
    HERE
    / "runs/mapping_repair_2026-07-13/C_corrected_uniform/validation/block_quality_v1.json"
)


def wrap_rad(angle: float) -> float:
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def quat_axis_angle(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    half = 0.5 * float(angle_rad)
    return np.concatenate([[math.cos(half)], math.sin(half) * axis])


def quat_z(angle_rad: float) -> np.ndarray:
    return quat_axis_angle(np.array([0.0, 0.0, 1.0]), angle_rad)


def qrel(q_parent: np.ndarray, q_child: np.ndarray) -> np.ndarray:
    return pf.qmul(pf.qconj(q_parent), q_child)


def quat_angle_deg(q: np.ndarray) -> np.ndarray:
    q = pf.qnormalize(np.asarray(q, dtype=float))
    return np.degrees(2.0 * np.arccos(np.clip(np.abs(q[..., 0]), -1.0, 1.0)))


def quat_series_distance_deg(a: np.ndarray, b: np.ndarray) -> dict[str, float]:
    delta = qrel(pf.qnormalize(a), pf.qnormalize(b))
    angle = quat_angle_deg(delta)
    return {
        "rms_deg": float(np.sqrt(np.mean(angle**2))),
        "max_deg": float(np.max(angle)),
    }


def average_quat(q: np.ndarray, mask: np.ndarray) -> np.ndarray:
    if int(np.count_nonzero(mask)) < 3:
        raise ValueError("neutral mask has fewer than three samples")
    return pf.quat_average(np.asarray(q)[mask])


def qmt_heading_delta(q_base: np.ndarray, q_target: np.ndarray) -> float:
    """Return qmt.resetHeading's left world-z correction for one reset.

    Formula reproduced from the MIT-licensed qmt implementation:
    https://qmt.readthedocs.io/en/latest/_modules/qmt/functions/reset.html
    Both inputs use this project convention: [w,x,y,z], local-to-world.
    """

    qb = pf.qnormalize(np.asarray(q_base, dtype=float))
    qt = pf.qnormalize(np.asarray(q_target, dtype=float))
    a = float(np.dot(qb, qt))
    b = float(qb[3] * qt[0] + qb[2] * qt[1] - qb[1] * qt[2] - qb[0] * qt[3])
    return wrap_rad(2.0 * math.atan2(b, a))


def heading_component_deg(q: np.ndarray) -> float:
    twist, _ = pf.swing_twist_deg(np.asarray(q, dtype=float), pf.SEGMENT_TWIST_AXIS)
    return float(np.asarray(twist))


def apply_t0_common_heading(
    segments: dict[str, np.ndarray],
    neutral: np.ndarray,
    *,
    base: str = "sacrum",
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """Left-multiply constant world-z corrections so T0 headings match base."""

    q0 = {role: average_quat(q, neutral) for role, q in segments.items()}
    qb = q0[base]
    indices = np.flatnonzero(neutral)
    split = max(1, len(indices) // 2)
    first = np.zeros_like(neutral)
    second = np.zeros_like(neutral)
    first[indices[:split]] = True
    second[indices[split:]] = True
    if int(np.count_nonzero(second)) < 3:
        second = first.copy()

    corrected: dict[str, np.ndarray] = {}
    diagnostics: dict[str, Any] = {}
    for role, q in segments.items():
        delta = 0.0 if role == base else qmt_heading_delta(qb, q0[role])
        corrected[role] = pf.qmul(quat_z(delta), q)

        before = qrel(qb, q0[role])
        after_q0 = average_quat(corrected[role], neutral)
        after = qrel(qb, after_q0)

        if role == base:
            stability = 0.0
        else:
            delta_first = qmt_heading_delta(
                average_quat(segments[base], first), average_quat(q, first)
            )
            delta_second = qmt_heading_delta(
                average_quat(segments[base], second), average_quat(q, second)
            )
            stability = math.degrees(abs(wrap_rad(delta_second - delta_first)))

        diagnostics[role] = {
            "left_heading_correction_deg": math.degrees(delta),
            "neutral_relative_heading_before_deg": heading_component_deg(before),
            "neutral_relative_heading_after_deg": heading_component_deg(after),
            "neutral_full_residual_before_deg": float(quat_angle_deg(before)),
            "neutral_full_residual_after_deg": float(quat_angle_deg(after)),
            "split_half_heading_delta_difference_deg": stability,
        }
    return corrected, diagnostics


def neutral_first_segments(
    segments: dict[str, np.ndarray], neutral: np.ndarray
) -> dict[str, np.ndarray]:
    """Gauge-invariant per-sensor motion, expressed in each sensor's T0 frame."""

    return {
        role: pf.qmul(pf.qconj(average_quat(q, neutral))[None, :], q)
        for role, q in segments.items()
    }


def arm_relations(segments: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    out = {
        name: qrel(segments[parent], segments[child])
        for name, (parent, child) in HEADLINE_RELATIONS.items()
    }
    out.update({
        name: qrel(segments[parent], segments[child])
        for name, (parent, child) in GAUGE_RELATIONS.items()
    })
    return out


def build_arm_relations(
    canonical_segments: dict[str, np.ndarray],
    functional_segments: dict[str, np.ndarray],
    neutral: np.ndarray,
) -> tuple[dict[str, dict[str, np.ndarray]], dict[str, Any]]:
    heading_segments, heading_qc = apply_t0_common_heading(canonical_segments, neutral)
    neutral_first = neutral_first_segments(canonical_segments, neutral)
    functional_heading, functional_heading_qc = apply_t0_common_heading(functional_segments, neutral)
    arms = {
        "canonical_relation_first": arm_relations(canonical_segments),
        "t0_common_heading": arm_relations(heading_segments),
        "neutral_first_identity": arm_relations(neutral_first),
        "functional_then_t0_heading": arm_relations(functional_heading),
    }
    return arms, {
        "gravity_only_t0_heading": heading_qc,
        "functional_t0_heading": functional_heading_qc,
    }


def inject_left_heading(
    segments: dict[str, np.ndarray], offsets_deg: dict[str, float]
) -> dict[str, np.ndarray]:
    return {
        role: pf.qmul(quat_z(math.radians(offsets_deg[role])), q)
        for role, q in segments.items()
    }


def final_readout_difference(
    baseline_q: np.ndarray,
    injected_q: np.ndarray,
    t: np.ndarray,
    block_rows: list[tuple[Any, ...]],
    *,
    kind: str,
    n_calib: int,
) -> dict[str, float]:
    """Compare the locally re-tared bend/twist signal actually used for scoring."""

    differences: list[np.ndarray] = []
    for block_id, block, _, a, b, bouts_raw in block_rows:
        label = mlb._canon_label(
            mlb.BLOCK_OVERRIDES.get("08", {}).get(block_id, {}).get("label", block["label"])
        )
        row_kind = "twist" if "twist" in label else "bend"
        if row_kind != kind:
            continue
        bouts = [(float(lo), float(hi)) for lo, hi in bouts_raw]
        if block_id in {"B1", "B3"}:
            bouts = bouts[n_calib:]
        for lo, hi in bouts:
            base_twist, base_swing = ffs.local_components(baseline_q, t, a, b, lo)
            inj_twist, inj_swing = ffs.local_components(injected_q, t, a, b, lo)
            base_series = base_swing if kind == "bend" else base_twist
            inj_series = inj_swing if kind == "bend" else inj_twist
            grid = np.linspace(lo, hi, 200, endpoint=False)
            pre_grid = np.linspace(lo - 1.2, lo - 0.2, 100, endpoint=False)
            base_zero = float(np.mean(np.interp(a * pre_grid + b, t, base_series)))
            inj_zero = float(np.mean(np.interp(a * pre_grid + b, t, inj_series)))
            base_out = np.interp(a * grid + b, t, base_series) - base_zero
            inj_out = np.interp(a * grid + b, t, inj_series) - inj_zero
            differences.append(inj_out - base_out)
    if not differences:
        raise ValueError(f"no {kind} bouts for final-readout gauge gate")
    values = np.concatenate(differences)
    return {
        "rms_deg": float(np.sqrt(np.mean(values**2))),
        "max_abs_deg": float(np.max(np.abs(values))),
    }


def gauge_injection_gate(
    canonical_segments: dict[str, np.ndarray],
    functional_segments: dict[str, np.ndarray],
    neutral: np.ndarray,
    baseline: dict[str, dict[str, np.ndarray]],
    t: np.ndarray,
    block_rows: list[tuple[Any, ...]],
    n_calib: int,
) -> dict[str, Any]:
    patterns = {
        "separated": {
            "sacrum": 0.0,
            "lower": 20.0,
            "mid": -15.0,
            "upper": 30.0,
            "sternum": -25.0,
        },
        "alternating": {
            "sacrum": -17.0,
            "lower": 33.0,
            "mid": -41.0,
            "upper": 12.0,
            "sternum": 46.0,
        },
    }
    rows: dict[str, Any] = {}
    worst_by_method = {method: 0.0 for method in METHODS}
    for pattern, offsets in patterns.items():
        injected_canonical = inject_left_heading(canonical_segments, offsets)
        injected_functional = inject_left_heading(functional_segments, offsets)
        injected_arms, _ = build_arm_relations(injected_canonical, injected_functional, neutral)
        rows[pattern] = {"offsets_deg": offsets, "methods": {}}
        for method in METHODS:
            rows[pattern]["methods"][method] = {
                "raw_relative_quaternion_series": {},
                "final_locally_tared_readout": {},
            }
            for relation in GAUGE_RELATIONS:
                distance = quat_series_distance_deg(
                    baseline[method][relation], injected_arms[method][relation]
                )
                rows[pattern]["methods"][method]["raw_relative_quaternion_series"][relation] = distance
                worst_by_method[method] = max(worst_by_method[method], distance["max_deg"])
            for relation in HEADLINE_RELATIONS:
                rows[pattern]["methods"][method]["final_locally_tared_readout"][relation] = (
                    final_readout_difference(
                        baseline[method][relation],
                        injected_arms[method][relation],
                        t,
                        block_rows,
                        kind=relation,
                        n_calib=n_calib,
                    )
                )

    common_offsets = {role: 30.0 for role in ROLES}
    common_canonical = arm_relations(inject_left_heading(canonical_segments, common_offsets))
    common_left_sanity = {
        relation: quat_series_distance_deg(
            baseline["canonical_relation_first"][relation], common_canonical[relation]
        )
        for relation in GAUGE_RELATIONS
    }
    return {
        "model": "constant independent left multiplication by world-z yaw per sensor",
        "scope_note": (
            "Raw quaternion-series distance tests the relation algebra before local re-tare. "
            "Final-readout distance separately tests the per-bout locally re-tared swing/twist output."
        ),
        "patterns": rows,
        "worst_max_error_deg_by_method": worst_by_method,
        "pass_by_method": {
            method: value <= GAUGE_GATE_TOL_DEG for method, value in worst_by_method.items()
        },
        "common_left_30deg_sanity": common_left_sanity,
        "tolerance_deg": GAUGE_GATE_TOL_DEG,
    }


def locally_tare(q: np.ndarray, neutral: np.ndarray) -> np.ndarray:
    q0 = average_quat(q, neutral)
    return pf.qmul(pf.qconj(q0)[None, :], q)


def rigid_comotion_gate() -> dict[str, Any]:
    """Synthetic two-sensor gate with independent gauges and right mounts."""

    n = 401
    neutral = np.zeros(n, dtype=bool)
    neutral[:31] = True
    phase = np.linspace(0.0, 1.0, n - 31)
    q_motion = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (n, 1))
    for i, u in enumerate(phase, start=31):
        qx = quat_axis_angle(np.array([1.0, 0.0, 0.0]), math.radians(45.0) * math.sin(math.pi * u))
        qy = quat_axis_angle(np.array([0.0, 1.0, 0.0]), math.radians(25.0) * math.sin(2.0 * math.pi * u))
        q_motion[i] = pf.qmul(qx, qy)

    gauges = {
        "sacrum": quat_z(math.radians(28.0)),
        "upper": quat_z(math.radians(-21.0)),
    }
    mounts = {
        "sacrum": quat_z(math.radians(24.0)),
        "upper": quat_z(math.radians(-32.0)),
    }
    observed = {
        role: pf.qmul(pf.qmul(gauges[role], q_motion), mounts[role])
        for role in ("sacrum", "upper")
    }

    heading_only, _ = apply_t0_common_heading(observed, neutral)
    neutral_first = neutral_first_segments(observed, neutral)
    known_mount_corrected = {
        role: pf.qmul(q, pf.qconj(mounts[role])) for role, q in observed.items()
    }
    full_two_stage, _ = apply_t0_common_heading(known_mount_corrected, neutral)
    candidates = {
        "canonical_relation_first": qrel(observed["sacrum"], observed["upper"]),
        "t0_common_heading": qrel(heading_only["sacrum"], heading_only["upper"]),
        "neutral_first_identity": qrel(neutral_first["sacrum"], neutral_first["upper"]),
        "known_mount_then_t0_heading": qrel(full_two_stage["sacrum"], full_two_stage["upper"]),
    }

    rows: dict[str, Any] = {}
    for method, relation in candidates.items():
        false_motion = quat_angle_deg(locally_tare(relation, neutral))
        rows[method] = {
            "false_motion_rms_deg": float(np.sqrt(np.mean(false_motion**2))),
            "false_motion_max_deg": float(np.max(false_motion)),
            "pass": float(np.max(false_motion)) <= GAUGE_GATE_TOL_DEG,
        }
    return {
        "model": "same physical segment motion; different constant world-yaw gauges and different sensor mounting headings",
        "gauge_yaw_deg": {"sacrum": 28.0, "upper": -21.0},
        "mount_heading_deg": {"sacrum": 24.0, "upper": -32.0},
        "methods": rows,
        "interpretation": (
            "A neutral heading reset removes left reference-frame yaw only after the right sensor-to-segment "
            "mounting transforms are known. T0 alone cannot separate those two constant yaw terms."
        ),
        "tolerance_deg": GAUGE_GATE_TOL_DEG,
    }


def score_blocks(
    arms: dict[str, dict[str, np.ndarray]],
    block_rows: list[tuple[Any, ...]],
    signals: dict[str, np.ndarray],
    tm: np.ndarray,
    axial: np.ndarray,
    n_calib: int,
) -> dict[str, Any]:
    rows: dict[str, Any] = {}
    for block_id, block, res, a, b, bouts_raw in block_rows:
        label = mlb._canon_label(
            mlb.BLOCK_OVERRIDES.get("08", {}).get(block_id, {}).get("label", block["label"])
        )
        if label not in mlb.LABELS:
            continue
        bouts = [(float(lo), float(hi)) for lo, hi in bouts_raw]
        excluded: list[list[float]] = []
        if block_id in {"B1", "B3"}:
            excluded = [[lo, hi] for lo, hi in bouts[:n_calib]]
            bouts = bouts[n_calib:]
        kind = "twist" if "twist" in label else "bend"
        signal_key = mlb.SIG_BY_LABEL[label]
        row: dict[str, Any] = {
            "label": label,
            "kind": kind,
            "signal": signal_key,
            "n_test_bouts": len(bouts),
            "calibration_bouts_excluded": excluded,
            "methods": {},
        }
        for method in METHODS:
            row["methods"][method] = ffs.score_relation(
                arms[method][kind], res.t_s, tm, signals[signal_key], bouts, a, b, kind
            )
        if kind == "bend":
            row["bend_to_twist_cross_talk"] = {
                method: ffs.score_relation(
                    arms[method]["twist"], res.t_s, tm, axial, bouts, a, b, "twist"
                )
                for method in METHODS
            }
        rows[block_id] = row
    return rows


def aggregate_scores(
    blocks: dict[str, Any], block_ids: list[str] | None = None
) -> dict[str, Any]:
    selected = blocks if block_ids is None else {block_id: blocks[block_id] for block_id in block_ids}
    if not selected:
        raise ValueError("aggregate requires at least one block")
    out: dict[str, Any] = {}
    for method in METHODS:
        held = np.array([
            row["methods"][method]["heldout_rmse_deg"] for row in selected.values()
        ], dtype=float)
        raw = np.array([
            row["methods"][method]["raw_rmse_deg"] for row in selected.values()
        ], dtype=float)
        corr = np.array([
            row["methods"][method]["pooled_r"] for row in selected.values()
        ], dtype=float)
        xtalk = np.array([
            row["bend_to_twist_cross_talk"][method]["imu_rms_deg"]
            for row in selected.values() if row["kind"] == "bend"
        ], dtype=float)
        worst_id = max(
            selected,
            key=lambda block_id: selected[block_id]["methods"][method]["heldout_rmse_deg"],
        )
        out[method] = {
            "n_blocks": len(selected),
            "block_ids": list(selected),
            "median_heldout_rmse_deg": float(np.median(held)),
            "median_raw_rmse_deg": float(np.median(raw)),
            "median_pooled_r": float(np.median(corr)),
            "median_bend_to_twist_imu_rms_deg": float(np.median(xtalk)),
            "worst_block": worst_id,
            "worst_heldout_rmse_deg": float(np.max(held)),
        }
    canonical = out["canonical_relation_first"]
    for method in METHODS[1:]:
        paired_held = np.array([
            row["methods"][method]["heldout_rmse_deg"]
            - row["methods"]["canonical_relation_first"]["heldout_rmse_deg"]
            for row in selected.values()
        ], dtype=float)
        paired_raw = np.array([
            row["methods"][method]["raw_rmse_deg"]
            - row["methods"]["canonical_relation_first"]["raw_rmse_deg"]
            for row in selected.values()
        ], dtype=float)
        out[method]["delta_vs_canonical"] = {
            "warning": "delta_of_medians_is_not_the_paired_block_effect",
            "median_heldout_rmse_deg": (
                out[method]["median_heldout_rmse_deg"] - canonical["median_heldout_rmse_deg"]
            ),
            "median_raw_rmse_deg": (
                out[method]["median_raw_rmse_deg"] - canonical["median_raw_rmse_deg"]
            ),
            "median_bend_to_twist_imu_rms_deg": (
                out[method]["median_bend_to_twist_imu_rms_deg"]
                - canonical["median_bend_to_twist_imu_rms_deg"]
            ),
        }
        out[method]["paired_block_delta_vs_canonical"] = {
            "heldout_rmse_deg": {
                "median": float(np.median(paired_held)),
                "mean": float(np.mean(paired_held)),
                "per_block": {
                    block_id: float(value) for block_id, value in zip(selected, paired_held)
                },
            },
            "raw_rmse_deg": {
                "median": float(np.median(paired_raw)),
                "mean": float(np.mean(paired_raw)),
                "per_block": {
                    block_id: float(value) for block_id, value in zip(selected, paired_raw)
                },
            },
        }
    return out


def neutral_motion_qc(res: Any, neutral: np.ndarray) -> dict[str, Any]:
    return {
        role: {
            "gyro_rms_dps": float(np.sqrt(np.mean(sensor.gyr_cal_dps[neutral] ** 2))),
            "raw_gyro_mean_norm_dps": float(np.linalg.norm(np.mean(sensor.gyr_raw_dps[neutral], axis=0))),
            "max_axis_gyro_std_dps": float(np.max(np.std(sensor.gyr_cal_dps[neutral], axis=0))),
        }
        for role, sensor in res.sensors.items()
    }


def select_quiet_preprotocol_t0(
    res: Any,
    *,
    protocol_start_imu_s: float,
    duration_s: float,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Select a fixed-duration T0 using IMU stillness only, before B1 starts."""

    t = res.t_s
    search_start = max(float(t[0]) + 1.0, float(res.summary["tare_window_s"][1]) + 0.5)
    search_end = protocol_start_imu_s - 1.0
    starts = np.arange(search_start, search_end - duration_s, 0.25)
    candidates: list[tuple[float, float, float, dict[str, float]]] = []
    for start in starts:
        stop = float(start + duration_s)
        mask = (t >= start) & (t <= stop)
        if int(np.count_nonzero(mask)) < 20:
            continue
        role_scores = {
            role: float(np.max(np.std(sensor.gyr_raw_dps[mask], axis=0)))
            for role, sensor in res.sensors.items()
        }
        candidates.append((max(role_scores.values()), float(start), stop, role_scores))
    if not candidates:
        raise ValueError("no valid quiet pre-protocol T0 candidates")
    score, start, stop, role_scores = min(candidates, key=lambda row: (row[0], row[1]))
    selected = (t >= start) & (t <= stop)
    return selected, {
        "selection_rule": "minimum across-window maximum of per-role max-axis raw-gyro std",
        "search_interval_imu_s": [search_start, search_end],
        "duration_s": duration_s,
        "candidate_count": len(candidates),
        "selected_window_imu_s": [float(t[selected][0]), float(t[selected][-1])],
        "objective_max_axis_std_dps": score,
        "per_role_max_axis_std_dps": role_scores,
        "objective_uses_mocap_amplitude_or_fit": False,
        "search_boundary_provenance": "frozen B1 protocol start mapped through the frozen MoCap-to-IMU clock",
        "pose_warning": "low gyro variation does not prove the intended neutral pose; std alone can miss smooth constant rotation",
    }


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_frozen_quality(subject: str) -> tuple[dict[str, Any], dict[str, Any]]:
    if not FROZEN_QUALITY_PATH.exists():
        raise FileNotFoundError(f"missing frozen quality authority: {FROZEN_QUALITY_PATH}")
    payload = json.loads(FROZEN_QUALITY_PATH.read_text(encoding="utf-8"))
    rows = {
        row["block"]: {
            "quality": row["quality"],
            "quality_reasons": row.get("quality_reasons", []),
            "n_expected": row.get("n_expected"),
            "n_scored": row.get("n_scored"),
        }
        for row in payload["rows"]
        if row["subject"] == f"T{subject}"
    }
    if not rows:
        raise ValueError(f"no frozen quality rows for T{subject}")
    provenance = {
        "path": str(FROZEN_QUALITY_PATH.relative_to(HERE)),
        "sha256": sha256_file(FROZEN_QUALITY_PATH),
        "schema_version": payload.get("schema_version"),
        "quality_policy_version": payload.get("quality_policy_version"),
    }
    return rows, provenance


def test_bout_provenance(
    block_rows: list[tuple[Any, ...]], n_calib: int
) -> dict[str, Any]:
    rows: dict[str, Any] = {}
    detected_total = 0
    test_total = 0
    for block_id, _, _, _, _, bouts_raw in block_rows:
        detected = [[float(lo), float(hi)] for lo, hi in bouts_raw]
        excluded = detected[:n_calib] if block_id in {"B1", "B3"} else []
        retained = detected[n_calib:] if block_id in {"B1", "B3"} else detected
        rows[block_id] = {
            "detected_bouts_mocap_s": detected,
            "calibration_excluded_mocap_s": excluded,
            "retained_test_bouts_mocap_s": retained,
            "n_detected": len(detected),
            "n_test": len(retained),
        }
        detected_total += len(detected)
        test_total += len(retained)
    return {
        "detected_total": detected_total,
        "test_total": test_total,
        "blocks": rows,
    }


def round_json(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): round_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [round_json(item) for item in value]
    if isinstance(value, np.generic):
        return round_json(value.item())
    if isinstance(value, float):
        if not np.isfinite(value):
            return None
        return round(value, 8)
    return value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--subject", default="08")
    parser.add_argument("--calibration-reps", type=int, default=3)
    parser.add_argument(
        "--test-exclude-reps",
        type=int,
        default=None,
        help="First N B1/B3 bouts excluded from every arm's test set; defaults to calibration-reps.",
    )
    parser.add_argument(
        "--t0-mode",
        choices=("canonical", "quiet-preprotocol"),
        default="canonical",
    )
    parser.add_argument("--t0-seconds", type=float, default=5.0)
    parser.add_argument("--out-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    subject = f"{int(str(args.subject).lstrip('Tt')):02d}"
    if subject != "08":
        raise SystemExit("This first common-heading shadow is intentionally locked to T08.")
    if args.calibration_reps < 2:
        raise SystemExit("--calibration-reps must be at least 2")
    n_test_exclude = (
        args.calibration_reps if args.test_exclude_reps is None else args.test_exclude_reps
    )
    if n_test_exclude < args.calibration_reps:
        raise SystemExit("--test-exclude-reps must be >= --calibration-reps to prevent calibration leakage")

    out_dir = args.out_dir.expanduser().resolve()
    data_clean = (HERE / "data_clean").resolve()
    try:
        out_dir.relative_to(data_clean)
        raise SystemExit(f"refusing output inside data_clean: {out_dir}")
    except ValueError:
        pass
    if out_dir.exists() and any(out_dir.iterdir()):
        raise SystemExit(f"--out-dir must be new or empty: {out_dir}")
    out_dir.mkdir(parents=True, exist_ok=True)

    tm, flex, lat, axial = sd.mocap_signed(mlb.mocap_path(subject))
    signals = {"flex": flex, "lat": lat, "axial": axial}
    block_rows = list(mlb.subject_blocks(subject, tm, signals))
    if not block_rows:
        raise SystemExit("no T08 blocks loaded")
    res = block_rows[0][2]
    if any(row[2] is not res for row in block_rows):
        raise SystemExit("T08 shadow requires one shared single-log fusion result")
    frozen_quality, frozen_quality_provenance = load_frozen_quality(subject)

    by_id = {row[0]: row for row in block_rows}
    if not {"B1", "B3"}.issubset(by_id):
        raise SystemExit("B1 and B3 are required for the functional calibration arm")
    a, b = float(by_id["B1"][3]), float(by_id["B1"][4])
    if any(abs(float(row[3]) - a) > 1e-12 or abs(float(row[4]) - b) > 1e-12 for row in block_rows):
        raise SystemExit("T08 shadow requires one common clock map")

    canonical_t0_lo, canonical_t0_hi = (float(x) for x in res.summary["tare_window_s"])
    if args.t0_mode == "canonical":
        neutral = (res.t_s >= canonical_t0_lo) & (res.t_s <= canonical_t0_hi)
        t0_selection = {
            "selection_rule": "reuse canonical tare window",
            "selected_window_imu_s": [canonical_t0_lo, canonical_t0_hi],
            "objective_uses_mocap_amplitude_or_fit": False,
        }
    else:
        starts = []
        for block_id, block, _, aa, bb, _ in block_rows:
            override = mlb.BLOCK_OVERRIDES.get(subject, {}).get(block_id, {})
            starts.append(float(aa) * float(mlb._block_window(block, override)[0]) + float(bb))
        neutral, t0_selection = select_quiet_preprotocol_t0(
            res,
            protocol_start_imu_s=min(starts),
            duration_s=float(args.t0_seconds),
        )
    t0_lo = float(res.t_s[neutral][0])
    t0_hi = float(res.t_s[neutral][-1])
    if int(np.count_nonzero(neutral)) < 20:
        raise SystemExit("T0 neutral window has fewer than 20 samples")

    flex_bouts = [(float(lo), float(hi)) for lo, hi in by_id["B1"][5]]
    lateral_bouts = [(float(lo), float(hi)) for lo, hi in by_id["B3"][5]]
    n_calib = args.calibration_reps
    if len(flex_bouts) <= n_test_exclude or len(lateral_bouts) <= n_test_exclude:
        raise SystemExit("not enough held-out B1/B3 bouts after functional calibration")
    calib_flex = flex_bouts[:n_calib]
    calib_lateral = lateral_bouts[:n_calib]

    canonical_segments = {role: res.sensors[role].q_segment for role in ROLES}
    functional_frames: dict[str, np.ndarray] = {}
    functional_frame_qc: dict[str, Any] = {}
    for role in ROLES:
        frames, qc = ffs.fit_frame(
            res.sensors[role], res.t_s, calib_flex, calib_lateral, a, b
        )
        functional_frames[role] = frames["two_motion_hybrid"]
        functional_frame_qc[role] = {
            "candidate_spread_deg": qc["candidate_spread_deg"],
            "hybrid_superior_to_gravity_deg": qc["hybrid_superior_to_gravity_deg"],
            "flexion_pca_dominance": qc["flexion_pca"]["dominance"],
            "lateral_pca_dominance": qc["lateral_pca"]["dominance"],
        }
    functional_segments = {
        role: pf.qmul(res.sensors[role].q_filter, functional_frames[role])
        for role in ROLES
    }

    arms, heading_qc = build_arm_relations(canonical_segments, functional_segments, neutral)
    parity = {
        relation: quat_series_distance_deg(
            arms["canonical_relation_first"][relation], res.relations[relation].q_rel
        )
        for relation in GAUGE_RELATIONS
    }
    if max(row["max_deg"] for row in parity.values()) > GAUGE_GATE_TOL_DEG:
        raise RuntimeError(f"canonical relation reconstruction mismatch: {parity}")

    blocks = score_blocks(arms, block_rows, signals, tm, axial, n_test_exclude)
    for block_id, row in blocks.items():
        row["frozen_corrected_uniform_quality"] = frozen_quality.get(block_id)
    aggregate = aggregate_scores(blocks)
    quality_valid_ids = [
        block_id
        for block_id in blocks
        if frozen_quality.get(block_id, {}).get("quality") in {"clean", "low_conf"}
    ]
    aggregate_quality_valid = aggregate_scores(blocks, quality_valid_ids)
    gauge_gate = gauge_injection_gate(
        canonical_segments,
        functional_segments,
        neutral,
        arms,
        res.t_s,
        block_rows,
        n_test_exclude,
    )
    comotion_gate = rigid_comotion_gate()

    heading_only_safe = (
        gauge_gate["pass_by_method"]["t0_common_heading"]
        and comotion_gate["methods"]["t0_common_heading"]["pass"]
    )
    ideal_two_stage_algebra_pass = (
        gauge_gate["pass_by_method"]["t0_common_heading"]
        and comotion_gate["methods"]["known_mount_then_t0_heading"]["pass"]
    )
    result = {
        "status": "shadow_sample_not_canonical",
        "subject": subject,
        "trial_id": "T93_P93",
        "question": "Does a T0 common-heading reset fix independent 6D-IMU heading gauges before relative orientation?",
        "scope": {
            "angle_scores": "two headline relations using sacrum, upper, and sternum",
            "raw_gauge_gate": "six relations covering all five sensors",
            "final_readout_gauge_gate": "headline bend and twist outputs after per-bout local re-tare",
        },
        "verdict_logic": {
            "heading_reset_only_passes_both_hard_gates": heading_only_safe,
            "ideal_known_mount_two_stage_algebra_pass": ideal_two_stage_algebra_pass,
            "empirical_functional_frame_passes_rigid_comotion_gate": "not_tested_without_mount_ground_truth",
            "promotion": "DO_NOT_CHANGE_MAINLINE",
            "reason": (
                "T0 heading reset is invariant to a constant world-z gauge but cannot by itself distinguish reference heading "
                "from sensor mounting heading; treat it as a shadow until sensor-to-segment calibration "
                "and repeated-donning/cohort validation exist."
            ),
        },
        "methods": {
            "canonical_relation_first": "gravity-up q_segment; q_parent^-1*q_child; per-bout local relation tare",
            "t0_common_heading": "qmt-style constant left world-z heading reset to sacrum during T0, then relation",
            "neutral_first_identity": "q_i(T0)^-1*q_i(t) per sensor, then parent^-1*child; gauge control assuming aligned T0 frames",
            "functional_then_t0_heading": (
                "right-multiply a two-motion segment-to-sensor calibration quaternion "
                "(functional axes expressed in sensor coordinates; inverse of the mounting transform), "
                "then apply qmt-style T0 heading reset"
            ),
        },
        "assumption_boundary": {
            "t0_common_heading": "all target segment headings are aligned at T0 after sensor-to-segment calibration",
            "neutral_first_identity": "neutral joint orientation J0 is identity in a shared segment convention",
            "functional_then_t0_heading": "B1/B3 PCA axes consistently approximate shared functional segment axes",
        },
        "calibration": {
            "t0_mode": args.t0_mode,
            "t0_window_imu_s": [t0_lo, t0_hi],
            "t0_samples": int(np.count_nonzero(neutral)),
            "t0_selection": t0_selection,
            "functional_reps_per_motion": n_calib,
            "test_excluded_reps_per_B1_B3": n_test_exclude,
            "functional_flexion_bouts_mocap_s": calib_flex,
            "functional_lateral_bouts_mocap_s": calib_lateral,
            "clock": {"a": a, "b": b},
            "neutral_motion_qc": neutral_motion_qc(res, neutral),
            "functional_frame_qc": functional_frame_qc,
            "heading_reset_qc": heading_qc,
        },
        "canonical_parity": parity,
        "block_comparison": blocks,
        "aggregate": {
            "exploratory_all_six_blocks": aggregate,
            "frozen_quality_valid_five_blocks_excluding_B2": aggregate_quality_valid,
            "warning": (
                "These are equal-weight block summaries, not participant-level estimates. "
                "Read paired_block_delta_vs_canonical; a delta of medians is not the paired effect."
            ),
        },
        "hard_gates": {
            "real_t08_independent_left_heading_injection": gauge_gate,
            "synthetic_rigid_comotion_different_mounts": comotion_gate,
        },
        "placement_provenance": res.summary.get("placement_provenance"),
        "frozen_quality_provenance": frozen_quality_provenance,
        "test_bout_provenance": test_bout_provenance(block_rows, n_test_exclude),
        "file_provenance": {
            "runner": {"path": str(Path(__file__).resolve()), "sha256": sha256_file(Path(__file__).resolve())},
            "imu": {"path": str(mlb.imu_path(subject)), "sha256": sha256_file(mlb.imu_path(subject))},
            "mocap": {"path": str(mlb.mocap_path(subject)), "sha256": sha256_file(mlb.mocap_path(subject))},
            "manifest": {"path": str(mlb.manifest_path(subject)), "sha256": sha256_file(mlb.manifest_path(subject))},
            "arguments": {
                "subject": subject,
                "calibration_reps": n_calib,
                "test_exclude_reps": n_test_exclude,
                "t0_mode": args.t0_mode,
                "t0_seconds": float(args.t0_seconds),
            },
        },
        "leakage_boundary": (
            f"T0 mode is {args.t0_mode}. For quiet-preprotocol mode, the stillness objective uses only "
            "raw gyro variation, but its search boundary is the frozen B1 protocol start mapped through "
            f"the MoCap-to-IMU clock. Functional PCA uses the first {n_calib} B1/B3 bouts and no MoCap "
            "angle or final score, but retrospective MoCap peak-detector boundaries identify those motions. "
            f"All arms exclude the first {n_test_exclude} B1/B3 bouts from testing. "
            "The retained test bouts are recorded explicitly in test_bout_provenance."
        ),
        "limitations": [
            "single subject and one donning; not promotion evidence",
            "T08 SFLP quaternions are magnetometer-free and their internal initialization is not externally synchronized",
            "neutral pose alone cannot identify left reference heading separately from right mounting heading",
            "functional frames are movement-defined rather than landmark-defined anatomical frames",
            "MoCap supplies retrospective protocol segmentation and the comparison reference",
            "the exploratory all-six-block aggregate includes T08 B2, which is invalid under the frozen corrected-uniform quality gate",
            "the heading reset only cancels a bout-constant world-z gauge; it does not correct time-varying differential heading drift",
            "the rigid co-motion PASS uses oracle known mounting yaw and does not prove that the empirical functional frame recovered it",
        ],
        "primary_implementation_reference": "https://qmt.readthedocs.io/en/latest/_modules/qmt/functions/reset.html",
    }

    json_path = out_dir / "T08_common_heading_shadow.json"
    json_path.write_text(
        json.dumps(round_json(result), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    print(f"wrote {json_path}")
    print("method                              held_med raw_med r_med xtalk_rms worst")
    for method in METHODS:
        row = aggregate[method]
        print(
            f"{method:34s} "
            f"{row['median_heldout_rmse_deg']:8.3f} "
            f"{row['median_raw_rmse_deg']:7.3f} "
            f"{row['median_pooled_r']:5.3f} "
            f"{row['median_bend_to_twist_imu_rms_deg']:9.3f} "
            f"{row['worst_block']}={row['worst_heldout_rmse_deg']:.3f}"
        )
    print("gauge gate:", gauge_gate["pass_by_method"])
    print("rigid co-motion:", {
        key: value["false_motion_max_deg"]
        for key, value in comotion_gate["methods"].items()
    })
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
