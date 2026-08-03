from __future__ import annotations

"""Shadow-only functional sensor-to-segment calibration probe.

This probe never modifies ``data_clean`` or the frozen canonical run.  It fits a
gravity/PCA/cross-product functional frame from a small, explicitly separated
set of flexion and lateral-bend bouts, freezes that frame, and compares it with
the current gravity-only alignment on the remaining bouts.

The fitted frame is a *functional* frame.  It is not called an anatomical frame
because no bony landmarks are used to define it.
"""

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import five_imu_fusion as fiv  # noqa: E402
import placement_maps as pm  # noqa: E402
import session_recipe as t06  # noqa: E402
import signed_diagnostic as sd  # noqa: E402
import twist_bench_fusion as pf  # noqa: E402
import dataset_adapter as mlb  # type: ignore  # noqa: E402


EPS = 1e-10


def unit(v: np.ndarray, name: str) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    norm = float(np.linalg.norm(v))
    if not np.isfinite(norm) or norm < EPS:
        raise ValueError(f"{name} is degenerate")
    return v / norm


def matrix_to_quat(matrix: np.ndarray) -> np.ndarray:
    """Convert a proper 3x3 rotation matrix to [w,x,y,z]."""
    m = np.asarray(matrix, dtype=float)
    if m.shape != (3, 3):
        raise ValueError("rotation matrix must be 3x3")
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = np.array([
            0.25 * s,
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
        ])
    else:
        i = int(np.argmax(np.diag(m)))
        if i == 0:
            s = math.sqrt(max(EPS, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) * 2.0
            q = np.array([
                (m[2, 1] - m[1, 2]) / s,
                0.25 * s,
                (m[0, 1] + m[1, 0]) / s,
                (m[0, 2] + m[2, 0]) / s,
            ])
        elif i == 1:
            s = math.sqrt(max(EPS, 1.0 + m[1, 1] - m[0, 0] - m[2, 2])) * 2.0
            q = np.array([
                (m[0, 2] - m[2, 0]) / s,
                (m[0, 1] + m[1, 0]) / s,
                0.25 * s,
                (m[1, 2] + m[2, 1]) / s,
            ])
        else:
            s = math.sqrt(max(EPS, 1.0 + m[2, 2] - m[0, 0] - m[1, 1])) * 2.0
            q = np.array([
                (m[1, 0] - m[0, 1]) / s,
                (m[0, 2] + m[2, 0]) / s,
                (m[1, 2] + m[2, 1]) / s,
                0.25 * s,
            ])
    q = unit(q, "matrix quaternion")
    return -q if q[0] < 0.0 else q


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    basis = np.eye(3)
    return np.column_stack([pf.quat_rotate(q, axis) for axis in basis])


def angle_deg(a: np.ndarray, b: np.ndarray, *, unsigned: bool = False) -> float:
    dot = float(np.dot(unit(a, "angle a"), unit(b, "angle b")))
    if unsigned:
        dot = abs(dot)
    return math.degrees(math.acos(float(np.clip(dot, -1.0, 1.0))))


def quat_distance_deg(a: np.ndarray, b: np.ndarray) -> float:
    return math.degrees(2.0 * math.acos(float(np.clip(abs(np.dot(unit(a, "qa"), unit(b, "qb"))), -1.0, 1.0))))


def mapped_mask(t: np.ndarray, bouts: list[tuple[float, float]], a: float, b: float) -> np.ndarray:
    mask = np.zeros(len(t), dtype=bool)
    for lo, hi in bouts:
        mask |= (t >= a * lo + b) & (t <= a * hi + b)
    return mask


def outbound_mask(t: np.ndarray, bouts: list[tuple[float, float]], a: float, b: float) -> np.ndarray:
    mask = np.zeros(len(t), dtype=bool)
    for lo, hi in bouts:
        mid = lo + 0.50 * (hi - lo)
        mask |= (t >= a * lo + b) & (t <= a * mid + b)
    return mask


def pca_axis(
    gyro: np.ndarray,
    t: np.ndarray,
    bouts: list[tuple[float, float]],
    a: float,
    b: float,
    name: str,
) -> tuple[np.ndarray, dict[str, float]]:
    mask = mapped_mask(t, bouts, a, b)
    values = np.asarray(gyro[mask], dtype=float)
    if len(values) < 20:
        raise ValueError(f"{name}: fewer than 20 calibration samples")
    values = values - values.mean(axis=0)
    covariance = values.T @ values / max(1, len(values) - 1)
    eigvals, eigvecs = np.linalg.eigh(covariance)
    order = np.argsort(eigvals)[::-1]
    eigvals = np.maximum(eigvals[order], 0.0)
    axis = unit(eigvecs[:, order[0]], f"{name} PCA axis")

    outbound = outbound_mask(t, bouts, a, b)
    projection = np.asarray(gyro[outbound], dtype=float) @ axis
    moving = np.abs(projection) >= np.percentile(np.abs(projection), 60.0)
    sign_score = float(np.mean(projection[moving])) if np.any(moving) else float(np.mean(projection))
    if sign_score < 0.0:
        axis = -axis
        sign_score = -sign_score
    total = float(np.sum(eigvals))
    return axis, {
        "samples": int(len(values)),
        "dominance": float(eigvals[0] / total) if total > 0.0 else 0.0,
        "lambda1_over_lambda2": float(eigvals[0] / max(eigvals[1], EPS)),
        "outbound_sign_score_dps": sign_score,
    }


def frame_from_xz(x_raw: np.ndarray, z_raw: np.ndarray) -> np.ndarray:
    z = unit(z_raw, "xz z")
    x = unit(x_raw - z * float(np.dot(x_raw, z)), "xz x")
    y = unit(np.cross(z, x), "xz y")
    x = unit(np.cross(y, z), "xz x reorthogonalized")
    return np.column_stack([x, y, z])


def frame_from_yz(y_raw: np.ndarray, z_raw: np.ndarray) -> np.ndarray:
    z = unit(z_raw, "yz z")
    y = unit(y_raw - z * float(np.dot(y_raw, z)), "yz y")
    x = unit(np.cross(y, z), "yz x")
    y = unit(np.cross(z, x), "yz y reorthogonalized")
    return np.column_stack([x, y, z])


def frame_from_xy(x_raw: np.ndarray, y_raw: np.ndarray, z_hint: np.ndarray) -> np.ndarray:
    x = unit(x_raw, "xy x")
    y = unit(y_raw - x * float(np.dot(y_raw, x)), "xy y")
    z = unit(np.cross(x, y), "xy z")
    if float(np.dot(z, z_hint)) < 0.0:
        y = -y
        z = -z
    y = unit(np.cross(z, x), "xy y reorthogonalized")
    return np.column_stack([x, y, z])


def frame_from_svd(x_raw: np.ndarray, y_raw: np.ndarray, z_raw: np.ndarray) -> np.ndarray:
    raw = np.column_stack([unit(x_raw, "svd x"), unit(y_raw, "svd y"), unit(z_raw, "svd z")])
    u, _, vt = np.linalg.svd(raw)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(u @ vt)
    return u @ correction @ vt


def fit_frame(
    sensor: fiv.SensorState,
    t: np.ndarray,
    flex_bouts: list[tuple[float, float]],
    lateral_bouts: list[tuple[float, float]],
    a: float,
    b: float,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    x_axis, flex_qc = pca_axis(sensor.gyr_cal_dps, t, flex_bouts, a, b, f"{sensor.role} flexion")
    y_axis, lat_qc = pca_axis(sensor.gyr_cal_dps, t, lateral_bouts, a, b, f"{sensor.role} lateral")
    z_axis = unit(sensor.up_sensor, f"{sensor.role} gravity")

    # Make the two movement-axis signs compatible with a right-handed frame whose +z
    # follows gravity.  The outbound phase resolves the remaining simultaneous 180°
    # ambiguity; no MoCap angle or final score is consulted.
    if float(np.dot(np.cross(x_axis, y_axis), z_axis)) < 0.0:
        y_axis = -y_axis

    matrices = {
        "pca_xy_cross": frame_from_xy(x_axis, y_axis, z_axis),
        "flexion_plus_gravity": frame_from_xz(x_axis, z_axis),
        "lateral_plus_gravity": frame_from_yz(y_axis, z_axis),
        "three_axis_svd": frame_from_svd(x_axis, y_axis, z_axis),
    }
    quats = np.stack([matrix_to_quat(matrix) for matrix in matrices.values()])
    q_mean = pf.quat_average(quats)
    q_simple = matrix_to_quat(matrices["flexion_plus_gravity"])
    final = quat_to_matrix(q_mean)
    candidate_spread = max(quat_distance_deg(q_mean, q) for q in quats)
    qc = {
        "imu": sensor.imu,
        "flexion_pca": flex_qc,
        "lateral_pca": lat_qc,
        "raw_axis_angles_deg": {
            "flexion_to_lateral_unsigned": angle_deg(x_axis, y_axis, unsigned=True),
            "flexion_to_gravity_unsigned": angle_deg(x_axis, z_axis, unsigned=True),
            "lateral_to_gravity_unsigned": angle_deg(y_axis, z_axis, unsigned=True),
        },
        "candidate_spread_deg": candidate_spread,
        "simple_to_hybrid_frame_distance_deg": quat_distance_deg(q_simple, q_mean),
        "hybrid_superior_to_gravity_deg": angle_deg(final[:, 2], z_axis),
        "final_det": float(np.linalg.det(final)),
        "final_orthogonality_fro": float(np.linalg.norm(final.T @ final - np.eye(3))),
        "functional_axes_in_sensor_xyz": {
            "x_flexion_axis": final[:, 0].tolist(),
            "y_lateral_axis": final[:, 1].tolist(),
            "z_superior_axis": final[:, 2].tolist(),
        },
    }
    return {"simple_gravity_pca_cross": q_simple, "two_motion_hybrid": q_mean}, qc


def qrel(q_parent: np.ndarray, q_child: np.ndarray) -> np.ndarray:
    return pf.qmul(pf.qconj(q_parent), q_child)


def pre_mask_or_fail(t: np.ndarray, a: float, b: float, lo: float) -> np.ndarray:
    mask = (t >= a * (lo - 1.2) + b) & (t <= a * (lo - 0.2) + b)
    if int(np.count_nonzero(mask)) < 3:
        raise ValueError(f"no valid pre-bout neutral for bout starting {lo:.3f}s")
    return mask


def local_components(
    q_rel: np.ndarray,
    t: np.ndarray,
    a: float,
    b: float,
    lo: float,
) -> tuple[np.ndarray, np.ndarray]:
    pre = pre_mask_or_fail(t, a, b, lo)
    q0 = pf.quat_average(q_rel[pre])
    q_tared = pf.qmul(pf.qconj(q0)[None, :], q_rel)
    twist, swing = pf.swing_twist_deg(q_tared, pf.SEGMENT_TWIST_AXIS)
    return -pf.unwrap_deg(twist), swing


def local_bend_plane_components(
    q_rel: np.ndarray,
    t: np.ndarray,
    a: float,
    b: float,
    lo: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return flexion-like and lateral-like angles in the chosen child frame.

    The functional frame defines x as the flexion rotation axis, y as the
    lateral-bend rotation axis and z as superior.  Rotating z by the locally
    tared relative quaternion gives the two signed tilt components below.
    """
    pre = pre_mask_or_fail(t, a, b, lo)
    q0 = pf.quat_average(q_rel[pre])
    q_tared = pf.qmul(pf.qconj(q0)[None, :], q_rel)
    direction = pf.quat_rotate(q_tared, np.array([0.0, 0.0, 1.0]))
    flexion = -np.degrees(np.arctan2(direction[:, 1], direction[:, 2]))
    lateral = np.degrees(np.arctan2(direction[:, 0], direction[:, 2]))
    return flexion, lateral


def collect_bout_arrays(
    q_rel: np.ndarray,
    t: np.ndarray,
    tm: np.ndarray,
    signal: np.ndarray,
    bouts: list[tuple[float, float]],
    a: float,
    b: float,
    kind: str,
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    xs: list[np.ndarray] = []
    ys: list[np.ndarray] = []
    for lo, hi in bouts:
        twist, swing = local_components(q_rel, t, a, b, lo)
        series = swing if kind == "bend" else twist
        grid = np.linspace(lo, hi, 200, endpoint=False)
        pre_grid = np.linspace(lo - 1.2, lo - 0.2, 100, endpoint=False)
        mocap_zero = float(np.mean(np.interp(pre_grid, tm, signal)))
        imu_zero = float(np.mean(np.interp(a * pre_grid + b, t, series)))
        x = np.interp(a * grid + b, t, series) - imu_zero
        y = np.interp(grid, tm, signal) - mocap_zero
        if kind == "bend":
            y = np.abs(y)
        if np.all(np.isfinite(x)) and np.all(np.isfinite(y)):
            xs.append(x)
            ys.append(y)
    return xs, ys


def score_arrays(xs: list[np.ndarray], ys: list[np.ndarray]) -> dict[str, Any]:
    if not xs:
        return {"n_bouts": 0}
    x_all = np.concatenate(xs)
    y_all = np.concatenate(ys)
    r = float(np.corrcoef(x_all, y_all)[0, 1]) if np.std(x_all) > EPS and np.std(y_all) > EPS else float("nan")
    gain, intercept = np.polyfit(x_all, y_all, 1)
    raw_rmse = float(np.sqrt(np.mean((y_all - x_all) ** 2)))
    calibrated_rmse = float(np.sqrt(np.mean((y_all - (gain * x_all + intercept)) ** 2)))
    fold_rmse: list[float] = []
    if len(xs) >= 2:
        for held in range(len(xs)):
            train_x = np.concatenate([x for i, x in enumerate(xs) if i != held])
            train_y = np.concatenate([y for i, y in enumerate(ys) if i != held])
            fold_gain, fold_intercept = np.polyfit(train_x, train_y, 1)
            prediction = fold_gain * xs[held] + fold_intercept
            fold_rmse.append(float(np.sqrt(np.mean((ys[held] - prediction) ** 2))))
    return {
        "n_bouts": len(xs),
        "pooled_r": r,
        "gain": float(gain),
        "raw_rmse_deg": raw_rmse,
        "calibrated_rmse_deg": calibrated_rmse,
        "heldout_rmse_deg": float(np.sqrt(np.mean(np.square(fold_rmse)))) if fold_rmse else None,
        "imu_rms_deg": float(np.sqrt(np.mean(x_all**2))),
        "imu_peak_abs_deg": float(np.max(np.abs(x_all))),
        "mocap_rms_deg": float(np.sqrt(np.mean(y_all**2))),
    }


def score_relation(
    q_rel: np.ndarray,
    t: np.ndarray,
    tm: np.ndarray,
    signal: np.ndarray,
    bouts: list[tuple[float, float]],
    a: float,
    b: float,
    kind: str,
) -> dict[str, Any]:
    return score_arrays(*collect_bout_arrays(q_rel, t, tm, signal, bouts, a, b, kind))


def score_bend_plane(
    q_rel: np.ndarray,
    t: np.ndarray,
    tm: np.ndarray,
    signal: np.ndarray,
    bouts: list[tuple[float, float]],
    a: float,
    b: float,
    desired_plane: str,
) -> dict[str, Any]:
    desired_rows: list[np.ndarray] = []
    undesired_rows: list[np.ndarray] = []
    mocap_rows: list[np.ndarray] = []
    for lo, hi in bouts:
        flexion, lateral = local_bend_plane_components(q_rel, t, a, b, lo)
        desired = flexion if desired_plane == "flexion" else lateral
        undesired = lateral if desired_plane == "flexion" else flexion
        grid = np.linspace(lo, hi, 200, endpoint=False)
        pre_grid = np.linspace(lo - 1.2, lo - 0.2, 100, endpoint=False)
        desired_zero = float(np.mean(np.interp(a * pre_grid + b, t, desired)))
        undesired_zero = float(np.mean(np.interp(a * pre_grid + b, t, undesired)))
        mocap_zero = float(np.mean(np.interp(pre_grid, tm, signal)))
        desired_rows.append(np.abs(np.interp(a * grid + b, t, desired) - desired_zero))
        undesired_rows.append(np.abs(np.interp(a * grid + b, t, undesired) - undesired_zero))
        mocap_rows.append(np.abs(np.interp(grid, tm, signal) - mocap_zero))
    score = score_arrays(desired_rows, mocap_rows)
    if desired_rows:
        desired_all = np.concatenate(desired_rows)
        undesired_all = np.concatenate(undesired_rows)
        desired_energy = float(np.sum(desired_all**2))
        undesired_energy = float(np.sum(undesired_all**2))
        score["desired_plane"] = desired_plane
        score["desired_energy_fraction"] = desired_energy / max(desired_energy + undesired_energy, EPS)
        score["undesired_rms_deg"] = float(np.sqrt(np.mean(undesired_all**2)))
    return score


def round_json(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): round_json(v) for k, v in value.items()}
    if isinstance(value, list):
        return [round_json(v) for v in value]
    if isinstance(value, tuple):
        return [round_json(v) for v in value]
    if isinstance(value, np.generic):
        return round_json(value.item())
    if isinstance(value, float):
        if not np.isfinite(value):
            return None
        return round(value, 6)
    return value


def plot_summary(path: Path, result: dict[str, Any]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    blocks = result["block_comparison"]
    labels = list(blocks)
    canonical_rmse = [blocks[k]["canonical"].get("heldout_rmse_deg", np.nan) for k in labels]
    simple_rmse = [blocks[k]["simple_gravity_pca_cross"].get("heldout_rmse_deg", np.nan) for k in labels]
    hybrid_rmse = [blocks[k]["two_motion_hybrid"].get("heldout_rmse_deg", np.nan) for k in labels]
    bend_labels = [k for k in labels if blocks[k]["kind"] == "bend"]
    canonical_xtalk = [blocks[k]["bend_to_twist_cross_talk"]["canonical"]["raw_rmse_deg"] for k in bend_labels]
    simple_xtalk = [blocks[k]["bend_to_twist_cross_talk"]["simple_gravity_pca_cross"]["raw_rmse_deg"] for k in bend_labels]
    hybrid_xtalk = [blocks[k]["bend_to_twist_cross_talk"]["two_motion_hybrid"]["raw_rmse_deg"] for k in bend_labels]

    fig, axes = plt.subplots(2, 2, figsize=(14, 10), constrained_layout=True)
    x = np.arange(len(labels))
    width = 0.26
    axes[0, 0].bar(x - width, canonical_rmse, width, label="canonical gravity-only")
    axes[0, 0].bar(x, simple_rmse, width, label="simple gravity+PCA+cross")
    axes[0, 0].bar(x + width, hybrid_rmse, width, label="two-motion hybrid")
    axes[0, 0].set_xticks(x, labels)
    axes[0, 0].set_ylabel("true LORO RMSE (deg)")
    axes[0, 0].set_title("Held-out angle score")
    axes[0, 0].legend()
    axes[0, 0].grid(axis="y", alpha=0.25)

    xb = np.arange(len(bend_labels))
    axes[0, 1].bar(xb - width, canonical_xtalk, width, label="canonical gravity-only")
    axes[0, 1].bar(xb, simple_xtalk, width, label="simple gravity+PCA+cross")
    axes[0, 1].bar(xb + width, hybrid_xtalk, width, label="two-motion hybrid")
    axes[0, 1].set_xticks(xb, bend_labels)
    axes[0, 1].set_ylabel("native twist-vs-MoCap axial RMSE (deg)")
    axes[0, 1].set_title("Bend-to-twist cross-talk")
    axes[0, 1].legend()
    axes[0, 1].grid(axis="y", alpha=0.25)

    roles = list(result["frame_qc"])
    flex_dom = [result["frame_qc"][role]["flexion_pca"]["dominance"] for role in roles]
    lat_dom = [result["frame_qc"][role]["lateral_pca"]["dominance"] for role in roles]
    xr = np.arange(len(roles))
    axes[1, 0].bar(xr - width / 2, flex_dom, width, label="flexion PCA")
    axes[1, 0].bar(xr + width / 2, lat_dom, width, label="lateral PCA")
    axes[1, 0].axhline(0.8, color="black", ls="--", lw=1, label="proposed QC=0.8")
    axes[1, 0].set_xticks(xr, roles, rotation=25)
    axes[1, 0].set_ylim(0, 1)
    axes[1, 0].set_ylabel("PC1 energy fraction")
    axes[1, 0].set_title("Calibration excitation quality")
    axes[1, 0].legend()
    axes[1, 0].grid(axis="y", alpha=0.25)

    spread = [result["frame_qc"][role]["candidate_spread_deg"] for role in roles]
    axes[1, 1].bar(xr, spread, color="tab:purple")
    axes[1, 1].axhline(10.0, color="black", ls="--", lw=1, label="proposed repeatability target")
    axes[1, 1].set_xticks(xr, roles, rotation=25)
    axes[1, 1].set_ylabel("candidate-frame spread (deg)")
    axes[1, 1].set_title("Cross-product construction agreement")
    axes[1, 1].legend()
    axes[1, 1].grid(axis="y", alpha=0.25)

    fig.suptitle(
        f"T{result['subject']} functional-frame shadow probe | "
        f"{result['calibration']['n_reps_per_motion']}+{result['calibration']['n_reps_per_motion']} calibration bouts"
    )
    fig.savefig(path, dpi=150)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--subject", default="08", help="Single-log subject id; this sample is validated for T08.")
    parser.add_argument("--calibration-reps", type=int, default=3, help="First N B1 and B3 bouts used only for calibration.")
    parser.add_argument("--out-dir", type=Path, required=True, help="New/empty shadow output directory.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    subject = f"{int(str(args.subject).lstrip('Tt')):02d}"
    if subject != "08":
        raise SystemExit("The first shadow sample is intentionally locked to clean single-log subject T08.")
    if args.calibration_reps < 2:
        raise SystemExit("--calibration-reps must be at least 2")
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
        raise SystemExit("no subject blocks loaded")
    res = block_rows[0][2]
    if any(row[2] is not res for row in block_rows):
        raise SystemExit("sample requires one shared single-log result")
    by_id = {row[0]: row for row in block_rows}
    if "B1" not in by_id or "B3" not in by_id:
        raise SystemExit("B1 flexion and B3 lateral calibration blocks are required")
    a, b = float(by_id["B1"][3]), float(by_id["B1"][4])
    if any(abs(float(row[3]) - a) > 1e-12 or abs(float(row[4]) - b) > 1e-12 for row in block_rows):
        raise SystemExit("sample requires one shared clock mapping")

    flex_bouts = [(float(lo), float(hi)) for lo, hi in by_id["B1"][5]]
    lateral_bouts = [(float(lo), float(hi)) for lo, hi in by_id["B3"][5]]
    n_calib = args.calibration_reps
    if len(flex_bouts) <= n_calib or len(lateral_bouts) <= n_calib:
        raise SystemExit("not enough held-out B1/B3 bouts after calibration split")
    calib_flex = flex_bouts[:n_calib]
    calib_lateral = lateral_bouts[:n_calib]

    frame_quats: dict[str, dict[str, np.ndarray]] = {}
    frame_qc: dict[str, Any] = {}
    for role, sensor in res.sensors.items():
        q_frames, qc = fit_frame(sensor, res.t_s, calib_flex, calib_lateral, a, b)
        frame_quats[role] = q_frames
        frame_qc[role] = qc

    functional_segment = {
        method: {
            role: pf.qmul(sensor.q_filter, frame_quats[role][method])
            for role, sensor in res.sensors.items()
        }
        for method in ("simple_gravity_pca_cross", "two_motion_hybrid")
    }
    q_functional_bend = {
        method: qrel(segments["sacrum"], segments["upper"])
        for method, segments in functional_segment.items()
    }
    q_functional_twist = {
        method: qrel(segments["sacrum"], segments["sternum"])
        for method, segments in functional_segment.items()
    }
    q_canonical_bend = res.relations["sacrum_to_upper"].q_rel
    q_canonical_twist = res.relations["sacrum_to_sternum"].q_rel

    block_comparison: dict[str, Any] = {}
    for block_id, block, _, aa, bb, bouts_raw in block_rows:
        label = mlb._canon_label(mlb.BLOCK_OVERRIDES.get(subject, {}).get(block_id, {}).get("label", block["label"]))
        if label not in mlb.LABELS:
            continue
        bouts = [(float(lo), float(hi)) for lo, hi in bouts_raw]
        calibration_excluded: list[list[float]] = []
        if block_id == "B1":
            calibration_excluded = [[lo, hi] for lo, hi in bouts[:n_calib]]
            bouts = bouts[n_calib:]
        elif block_id == "B3":
            calibration_excluded = [[lo, hi] for lo, hi in bouts[:n_calib]]
            bouts = bouts[n_calib:]
        signal_key = mlb.SIG_BY_LABEL[label]
        kind = "twist" if "twist" in label else "bend"
        canonical_q = q_canonical_twist if kind == "twist" else q_canonical_bend
        row: dict[str, Any] = {
            "label": label,
            "kind": kind,
            "signal": signal_key,
            "n_test_bouts": len(bouts),
            "calibration_bouts_excluded": calibration_excluded,
            "canonical": score_relation(canonical_q, res.t_s, tm, signals[signal_key], bouts, aa, bb, kind),
        }
        for method in ("simple_gravity_pca_cross", "two_motion_hybrid"):
            functional_q = q_functional_twist[method] if kind == "twist" else q_functional_bend[method]
            row[method] = score_relation(functional_q, res.t_s, tm, signals[signal_key], bouts, aa, bb, kind)
        if kind == "bend":
            desired_plane = "flexion" if label in {"flexion", "extension"} else "lateral"
            row["plane_specific_bend"] = {
                "canonical": score_bend_plane(
                    q_canonical_bend, res.t_s, tm, signals[signal_key], bouts, aa, bb, desired_plane
                )
            }
            for method in ("simple_gravity_pca_cross", "two_motion_hybrid"):
                row["plane_specific_bend"][method] = score_bend_plane(
                    q_functional_bend[method], res.t_s, tm, signals[signal_key], bouts, aa, bb, desired_plane
                )
            row["bend_to_twist_cross_talk"] = {
                "reference": "native locally-zeroed sacrum-to-sternum twist versus MoCap axial during bend bouts",
                "canonical": score_relation(q_canonical_twist, res.t_s, tm, axial, bouts, aa, bb, "twist"),
            }
            for method in ("simple_gravity_pca_cross", "two_motion_hybrid"):
                row["bend_to_twist_cross_talk"][method] = score_relation(
                    q_functional_twist[method], res.t_s, tm, axial, bouts, aa, bb, "twist"
                )
        block_comparison[block_id] = row

    result = {
        "status": "shadow_feasibility_sample_not_canonical",
        "subject": subject,
        "trial_id": f"T{subject}_P{subject}",
        "methods": {
            "canonical": "neutral gravity up-axis alignment only",
            "simple_gravity_pca_cross": "neutral gravity + flexion gyro PCA axis + cross-product orthonormalization",
            "two_motion_hybrid": "flexion/lateral gyro PCA axes + neutral gravity + cross-product/SVD candidate quaternion average",
        },
        "claim_boundary": "gravity-aligned functional frame; not a landmark-defined anatomical frame",
        "leakage_boundary": "PCA uses only first B1/B3 calibration bouts and no MoCap amplitudes/correlations; remaining bouts are test-only. MoCap still supplies protocol bout boundaries in this retrospective sample.",
        "calibration": {
            "n_reps_per_motion": n_calib,
            "flexion_bouts": calib_flex,
            "lateral_bouts": calib_lateral,
            "clock": {"a": a, "b": b},
        },
        "placement_provenance": res.summary.get("placement_provenance"),
        "frame_qc": frame_qc,
        "block_comparison": block_comparison,
        "limitations": [
            "single clean subject; not a cohort result",
            "calibration movements were not collected as a prospective independent calibration protocol",
            "MoCap-defined bout boundaries are reused for retrospective segmentation",
            "no repeated donning or mixed-motion trial is available",
            "functional axes are movement-defined and are not externally validated anatomical axes",
        ],
    }
    json_path = out_dir / f"T{subject}_functional_frame_shadow.json"
    plot_path = out_dir / f"T{subject}_functional_frame_shadow.png"
    json_path.write_text(json.dumps(round_json(result), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    plot_summary(plot_path, result)

    print(f"wrote {json_path}")
    print(f"wrote {plot_path}")
    print("block  canonical_r/held  simple_r/held  hybrid_r/held  xtalk_raw canonical->simple->hybrid")
    for block_id, row in block_comparison.items():
        c = row["canonical"]
        s = row["simple_gravity_pca_cross"]
        h = row["two_motion_hybrid"]
        xtalk = row.get("bend_to_twist_cross_talk")
        xtalk_text = "-"
        if xtalk:
            xtalk_text = (
                f"{xtalk['canonical']['raw_rmse_deg']:.2f}->"
                f"{xtalk['simple_gravity_pca_cross']['raw_rmse_deg']:.2f}->"
                f"{xtalk['two_motion_hybrid']['raw_rmse_deg']:.2f}"
            )
        print(
            f"{block_id:>3s}  {c.get('pooled_r', float('nan')): .3f}/{c.get('heldout_rmse_deg', float('nan')):5.2f}  "
            f"{s.get('pooled_r', float('nan')): .3f}/{s.get('heldout_rmse_deg', float('nan')):5.2f}  "
            f"{h.get('pooled_r', float('nan')): .3f}/{h.get('heldout_rmse_deg', float('nan')):5.2f}  {xtalk_text}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
