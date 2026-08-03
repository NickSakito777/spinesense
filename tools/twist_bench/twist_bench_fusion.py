from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import twist_bench_v0 as v0

G_MPS2 = 9.80665
SEGMENT_TWIST_AXIS = np.array([0.0, 0.0, 1.0])
NAMED_UP = {
    "+x": np.array([1.0, 0.0, 0.0]),
    "-x": np.array([-1.0, 0.0, 0.0]),
    "+y": np.array([0.0, 1.0, 0.0]),
    "-y": np.array([0.0, -1.0, 0.0]),
    "+z": np.array([0.0, 0.0, 1.0]),
    "-z": np.array([0.0, 0.0, -1.0]),
}


@dataclass(frozen=True)
class FusionPair:
    t_s: np.ndarray          # (N,)
    parent_acc_mg: np.ndarray  # (N, 3)
    parent_gyr_dps: np.ndarray
    child_acc_mg: np.ndarray
    child_gyr_dps: np.ndarray


# ---------------------------------------------------------------------------
# quaternion utilities, [w, x, y, z] convention, v_world = R(q) v_sensor
# ---------------------------------------------------------------------------

def qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bw, bx, by, bz = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        axis=-1,
    )


def qconj(q: np.ndarray) -> np.ndarray:
    out = q.copy()
    out[..., 1:] = -out[..., 1:]
    return out


def qnormalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    qv = np.zeros(q.shape[:-1] + (4,))
    qv[..., 1:] = v
    return qmul(qmul(q, qv), qconj(q))[..., 1:]


def quat_from_axis_angle(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    half = 0.5 * angle_rad
    return np.concatenate([[math.cos(half)], math.sin(half) * axis])


def quat_align_z_to(up: np.ndarray) -> np.ndarray:
    """Minimal rotation q with R(q) ez = up; maps segment vectors to sensor vectors."""
    up = up / np.linalg.norm(up)
    ez = np.array([0.0, 0.0, 1.0])
    c = float(np.dot(ez, up))
    if c > 1.0 - 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if c < -1.0 + 1e-12:
        return np.array([0.0, 1.0, 0.0, 0.0])  # 180 deg about x
    axis = np.cross(ez, up)
    angle = math.acos(max(-1.0, min(1.0, c)))
    return quat_from_axis_angle(axis, angle)


def quat_average(qs: np.ndarray) -> np.ndarray:
    qs = qs.copy()
    flip = np.sum(qs * qs[0], axis=-1) < 0.0
    qs[flip] = -qs[flip]
    m = qs.T @ qs
    eigvals, eigvecs = np.linalg.eigh(m)
    q = eigvecs[:, -1]
    if q[0] < 0:
        q = -q
    return q / np.linalg.norm(q)


def swing_twist_deg(q: np.ndarray, axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Decompose q = q_swing * q_twist (twist applied first, about `axis` in the child frame).

    Returns (twist_deg, swing_deg), twist wrapped to [-180, 180), swing >= 0.
    """
    q = qnormalize(q)
    proj = q[..., 1:] @ axis
    twist = 2.0 * np.arctan2(proj, q[..., 0])
    twist = (twist + np.pi) % (2.0 * np.pi) - np.pi

    q_twist = np.zeros_like(q)
    q_twist[..., 0] = q[..., 0]
    q_twist[..., 1:] = proj[..., None] * axis
    norms = np.linalg.norm(q_twist, axis=-1, keepdims=True)
    degenerate = norms[..., 0] < 1e-9
    q_twist = np.where(degenerate[..., None], np.array([1.0, 0.0, 0.0, 0.0]), q_twist / np.maximum(norms, 1e-12))

    q_swing = qmul(q, qconj(q_twist))
    swing = 2.0 * np.arccos(np.clip(np.abs(q_swing[..., 0]), -1.0, 1.0))
    return np.degrees(twist), np.degrees(swing)


def estimate_twist_axis(
    q: np.ndarray,
    calib_mask: np.ndarray | None = None,
    seed: np.ndarray = SEGMENT_TWIST_AXIS,
) -> np.ndarray:
    """Functional sensor-to-segment calibration of the longitudinal TWIST axis.

    Returns the unit axis (in the same tared/parent frame as `q`) that MINIMISES the twist RMS
    over a calibration movement that should contain no axial twist -- e.g. a pure flexion or
    lateral bend. That axis is the longitudinal direction the bend does NOT excite, i.e. the
    anatomical twist axis the fixed [0,0,1] choice only matches at neutral.

    Why it matters: swing_twist about a FIXED axis gives twist = 2*atan2(tan(theta/2)*n_z, 1), so a
    residual out-of-plane component n_z (left by gravity-only alignment, mounting heading, etc.) is
    multiplied by tan(theta/2) and explodes at large flexion -- the flexion->twist cross-talk.
    Decomposing about the functionally-identified axis removes that n_z by construction. On the
    2026-06-26 P01 pilot this cut sacrum->sternum flexion cross-talk ~4.3x (RMS 16.4->3.8 deg,
    peak 41->10 deg) while preserving genuine upright axial twist. Falls back to `seed` on thin data.
    """
    q = qnormalize(q)
    if calib_mask is not None:
        q = q[np.asarray(calib_mask, dtype=bool)]
    seed = np.asarray(seed, dtype=float)
    seed = seed / np.linalg.norm(seed)
    if len(q) < 10:
        return seed

    # The bend must actually move this relation, else the twist axis is under-determined and the fit
    # lands on noise. Fall back to the fixed seed if the calibration swing is too small (e.g. an
    # adjacent pair the bend barely flexed). 90th-pct total rotation < 15 deg -> not enough leverage.
    swing_deg = 2.0 * np.degrees(np.arccos(np.clip(np.abs(q[:, 0]), -1.0, 1.0)))
    if float(np.percentile(swing_deg, 90)) < 15.0:
        return seed

    qw = q[:, 0]
    qxyz = q[:, 1:]

    def twist_rms(axis: np.ndarray) -> float:
        axis = axis / np.linalg.norm(axis)
        tw = 2.0 * np.arctan2(qxyz @ axis, qw)
        tw = (tw + np.pi) % (2.0 * np.pi) - np.pi
        return float(np.sqrt(np.mean(tw * tw)))

    # Global coarse search over the unit sphere, then a shrinking local tangent-plane refine.
    # Minimises the EXACT twist RMS (not a small-angle proxy), so it is faithful at large swing.
    best_axis, best_val = seed, twist_rms(seed)
    for phi in np.linspace(0.0, np.pi, 37):
        sp, cp = math.sin(phi), math.cos(phi)
        for th in np.linspace(0.0, 2.0 * np.pi, 73):
            cand = np.array([sp * math.cos(th), sp * math.sin(th), cp])
            val = twist_rms(cand)
            if val < best_val:
                best_val, best_axis = val, cand
    step = math.radians(5.0)
    for _ in range(6):
        base = best_axis / np.linalg.norm(best_axis)
        ref = np.array([0.0, 0.0, 1.0]) if abs(base[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        t1 = np.cross(base, ref)
        t1 /= np.linalg.norm(t1)
        t2 = np.cross(base, t1)
        improved = False
        for du in (-step, 0.0, step):
            for dv in (-step, 0.0, step):
                cand = base + du * t1 + dv * t2
                val = twist_rms(cand)
                if val < best_val:
                    best_val, best_axis, improved = val, cand / np.linalg.norm(cand), True
        if not improved:
            step *= 0.5
    axis = best_axis / np.linalg.norm(best_axis)
    if np.dot(axis, seed) < 0.0:  # stable hemisphere: align with the neutral longitudinal seed
        axis = -axis
    return axis


def unwrap_deg(angles_deg: np.ndarray) -> np.ndarray:
    return np.degrees(np.unwrap(np.radians(angles_deg)))


# ---------------------------------------------------------------------------
# data loading: reuse twist_bench_v0 parsing, keep accel + gyro
# ---------------------------------------------------------------------------

def load_fusion_pairs(path: Path, parent: str, child: str) -> FusionPair:
    text = path.read_text(encoding="utf-8", errors="replace")
    records = v0.parse_serial_text(text)
    if not records:
        rows = v0.read_dict_rows(text)
        records = v0.parse_long_table_rows(rows)
    if not records:
        raise SystemExit(f"No IMU records parsed from {path}")

    parent_key, child_key = parent.upper(), child.upper()
    by_time: dict[float, dict[str, v0.ImuSample]] = {}
    for record in records:
        by_time.setdefault(record.t_s, {})[record.imu.upper()] = record

    rows_t, pa, pg, ca, cg = [], [], [], [], []
    for t_s in sorted(by_time):
        group = by_time[t_s]
        if parent_key in group and child_key in group:
            p, c = group[parent_key], group[child_key]
            rows_t.append(t_s)
            pa.append((p.ax_mg, p.ay_mg, p.az_mg))
            pg.append((p.gx_dps, p.gy_dps, p.gz_dps))
            ca.append((c.ax_mg, c.ay_mg, c.az_mg))
            cg.append((c.gx_dps, c.gy_dps, c.gz_dps))
    if len(rows_t) < 10:
        raise SystemExit(f"Only {len(rows_t)} exact parent/child pairs found; need more data.")

    t = np.asarray(rows_t)
    t = t - t[0]
    return FusionPair(
        t_s=t,
        parent_acc_mg=np.asarray(pa),
        parent_gyr_dps=np.asarray(pg),
        child_acc_mg=np.asarray(ca),
        child_gyr_dps=np.asarray(cg),
    )


# ---------------------------------------------------------------------------
# per-IMU 6D fusion
# ---------------------------------------------------------------------------

def run_vqf(gyr_dps: np.ndarray, acc_mg: np.ndarray, ts_s: float) -> tuple[np.ndarray, dict[str, object], np.ndarray]:
    from vqf import offlineVQF

    gyr = np.ascontiguousarray(np.radians(gyr_dps), dtype=np.float64)
    acc = np.ascontiguousarray(acc_mg * (G_MPS2 / 1000.0), dtype=np.float64)
    out = offlineVQF(gyr, acc, None, float(ts_s))
    quat = np.asarray(out["quat6D"])
    bias_dps = np.degrees(np.asarray(out["bias"]))
    rest = np.asarray(out["restDetected"], dtype=float)  # per-sample 0/1, used by closed-loop window gates
    info = {
        "residual_bias_final_dps": [round(float(b), 4) for b in bias_dps[-1]],
        "rest_detected_fraction": round(float(np.mean(rest)), 3),
    }
    return quat, info, rest


def run_madgwick(gyr_dps: np.ndarray, acc_mg: np.ndarray, t_s: np.ndarray, beta: float = 0.1) -> tuple[np.ndarray, dict[str, object], None]:
    gyr = np.radians(gyr_dps)
    acc = acc_mg.astype(float)
    n = len(t_s)
    quat = np.zeros((n, 4))
    q = init_quat_from_acc(acc[0])
    quat[0] = q
    for i in range(1, n):
        dt = max(1e-4, float(t_s[i] - t_s[i - 1]))
        q = madgwick_update(q, gyr[i], acc[i], dt, beta)
        quat[i] = q
    return quat, {"beta": beta}, None  # Madgwick has no rest detector; closed-loop rest gate is skipped


def init_quat_from_acc(acc: np.ndarray) -> np.ndarray:
    up = acc / max(1e-9, np.linalg.norm(acc))
    return qconj(quat_align_z_to(up))


def madgwick_update(q: np.ndarray, gyr_rads: np.ndarray, acc: np.ndarray, dt: float, beta: float) -> np.ndarray:
    w, x, y, z = q
    norm = np.linalg.norm(acc)
    q_dot = 0.5 * qmul(q, np.array([0.0, *gyr_rads]))
    if norm > 1e-9:
        ax, ay, az = acc / norm
        f = np.array(
            [
                2.0 * (x * z - w * y) - ax,
                2.0 * (w * x + y * z) - ay,
                2.0 * (0.5 - x * x - y * y) - az,
            ]
        )
        jt = np.array(
            [
                [-2.0 * y, 2.0 * z, -2.0 * w, 2.0 * x],
                [2.0 * x, 2.0 * w, 2.0 * z, 2.0 * y],
                [0.0, -4.0 * x, -4.0 * y, 0.0],
            ]
        )
        grad = jt.T @ f
        gnorm = np.linalg.norm(grad)
        if gnorm > 1e-12:
            q_dot = q_dot - beta * grad / gnorm
    q = q + q_dot * dt
    return q / np.linalg.norm(q)


# ---------------------------------------------------------------------------
# pipeline
# ---------------------------------------------------------------------------

@dataclass
class FusionResult:
    t_s: np.ndarray
    q_parent: np.ndarray
    q_child: np.ndarray
    q_rel: np.ndarray
    q_display: np.ndarray
    twist_deg: np.ndarray
    swing_deg: np.ndarray
    twist_v0_deg: np.ndarray | None
    summary: dict[str, object]
    # Inputs carried for optional closed-loop post-processing (never overwrite the raw filter output above).
    parent_rest: np.ndarray | None = None
    child_rest: np.ndarray | None = None
    still_mask: np.ndarray | None = None
    tare_mask: np.ndarray | None = None
    # Closed-loop post-processing products (None unless --closed-loop linear|piecewise applied).
    raw_fusion_twist_deg: np.ndarray | None = None
    closed_loop_twist_deg: np.ndarray | None = None
    correction_deg: np.ndarray | None = None


def resolve_up(name: str, acc_mg: np.ndarray, still: np.ndarray) -> np.ndarray:
    if name != "auto":
        return NAMED_UP[name]
    mean_acc = acc_mg[still].mean(axis=0)
    n = np.linalg.norm(mean_acc)
    if n < 100.0:
        raise SystemExit("Still-window accel mean too small to estimate gravity; check the still window.")
    return mean_acc / n


def run_pipeline(pairs: FusionPair, args: argparse.Namespace) -> FusionResult:
    t = pairs.t_s
    dts = np.diff(t)
    ts_med = float(np.median(dts))
    if args.bias_start_s is not None or args.bias_end_s is not None:
        b0 = args.bias_start_s if args.bias_start_s is not None else 0.0
        b1 = args.bias_end_s if args.bias_end_s is not None else float(t[-1])
        still = (t >= b0) & (t <= b1)
    else:
        still = t <= args.bias_seconds

    warnings: list[str] = []
    if still.sum() < 5:
        raise SystemExit("Bias window has fewer than 5 samples; increase --bias-seconds.")
    for label, gyr in (("parent", pairs.parent_gyr_dps), ("child", pairs.child_gyr_dps)):
        std = float(np.max(gyr[still].std(axis=0)))
        if std > 0.5:
            warnings.append(f"{label} gyro std in bias window is {std:.2f} dps; window may contain motion.")
    long_gaps = int(np.sum(dts > 0.5))
    if long_gaps:
        warnings.append(f"{long_gaps} sample gaps > 0.5 s; fixed-Ts fusion will misintegrate across them.")

    parent_bias = pairs.parent_gyr_dps[still].mean(axis=0)
    child_bias = pairs.child_gyr_dps[still].mean(axis=0)
    parent_gyr = pairs.parent_gyr_dps - parent_bias
    child_gyr = pairs.child_gyr_dps - child_bias

    if args.filter == "vqf":
        q_parent, parent_info, parent_rest = run_vqf(parent_gyr, pairs.parent_acc_mg, ts_med)
        q_child, child_info, child_rest = run_vqf(child_gyr, pairs.child_acc_mg, ts_med)
    else:
        q_parent, parent_info, parent_rest = run_madgwick(parent_gyr, pairs.parent_acc_mg, t)
        q_child, child_info, child_rest = run_madgwick(child_gyr, pairs.child_acc_mg, t)

    parent_up = resolve_up(args.parent_up, pairs.parent_acc_mg, still)
    child_up = resolve_up(args.child_up, pairs.child_acc_mg, still)
    q_align_parent = quat_align_z_to(parent_up)
    q_align_child = quat_align_z_to(child_up)

    q_seg_parent = qmul(q_parent, q_align_parent)
    q_seg_child = qmul(q_child, q_align_child)
    q_rel = qmul(qconj(q_seg_parent), q_seg_child)

    if args.tare_start_s is not None or args.tare_end_s is not None:
        t0 = args.tare_start_s if args.tare_start_s is not None else 0.0
        t1 = args.tare_end_s if args.tare_end_s is not None else float(t[-1])
        tare = (t >= t0) & (t <= t1)
    else:
        tare = t <= args.tare_seconds
    if tare.sum() < 3:
        raise SystemExit("Tare window has fewer than 3 samples.")
    return_window = t >= (t[-1] - args.return_window_seconds)
    if np.any(tare & return_window):
        warnings.append(
            "Tare window overlaps the return-to-zero window; "
            "return-to-zero will be artificially near zero."
        )
    q_rel0 = quat_average(q_rel[tare])
    q_display = qmul(qconj(q_rel0)[None, :], q_rel)

    twist_deg, swing_deg = swing_twist_deg(q_display, SEGMENT_TWIST_AXIS)
    twist_deg = unwrap_deg(twist_deg)

    twist_v0 = None
    if not args.no_v0_baseline:
        twist_v0 = integrate_v0_baseline(t, parent_gyr, child_gyr, parent_up)

    summary = build_summary(
        args=args,
        t=t,
        ts_med=ts_med,
        dts=dts,
        twist_deg=twist_deg,
        swing_deg=swing_deg,
        twist_v0=twist_v0,
        parent_bias=parent_bias,
        child_bias=child_bias,
        parent_up=parent_up,
        child_up=child_up,
        parent_info=parent_info,
        child_info=child_info,
        still=still,
        tare=tare,
        warnings=warnings,
    )
    return FusionResult(
        t_s=t,
        q_parent=q_parent,
        q_child=q_child,
        q_rel=q_rel,
        q_display=q_display,
        twist_deg=twist_deg,
        swing_deg=swing_deg,
        twist_v0_deg=twist_v0,
        summary=summary,
        parent_rest=parent_rest,
        child_rest=child_rest,
        still_mask=still,
        tare_mask=tare,
    )


def integrate_v0_baseline(
    t: np.ndarray, parent_gyr_dps: np.ndarray, child_gyr_dps: np.ndarray, parent_up: np.ndarray
) -> np.ndarray:
    """v0 reference: bias-corrected relative gyro projected on the measured up axis."""
    rel = child_gyr_dps - parent_gyr_dps
    rel_axis = rel @ parent_up
    dt = np.diff(t, prepend=t[0])
    return np.cumsum(rel_axis * dt)


def window_metrics(
    t: np.ndarray, twist_deg: np.ndarray, swing_deg: np.ndarray, tare: np.ndarray, args: argparse.Namespace
) -> dict[str, object]:
    out: dict[str, object] = {}
    last = t >= (t[-1] - args.return_window_seconds)
    out["return_to_zero_estimate_deg"] = float(np.mean(twist_deg[last]))
    detrended = twist_deg[tare] - np.mean(twist_deg[tare])
    out["tare_window_twist_rms_deg"] = float(np.sqrt(np.mean(detrended**2)))
    out["swing_max_deg"] = float(np.max(swing_deg))
    if args.drift_start_s is not None and args.drift_end_s is not None:
        sel = (t >= args.drift_start_s) & (t <= args.drift_end_s)
        if sel.sum() >= 5:
            slope = np.polyfit(t[sel], twist_deg[sel], 1)[0]
            out["drift_fit_deg_per_min"] = float(slope * 60.0)
        else:
            out["drift_fit_deg_per_min"] = None
    else:
        out["drift_fit_deg_per_min"] = None
    return out


def build_summary(*, args, t, ts_med, dts, twist_deg, swing_deg, twist_v0, parent_bias, child_bias,
                  parent_up, child_up, parent_info, child_info, still, tare, warnings) -> dict[str, object]:
    summary: dict[str, object] = {
        "algorithm": f"fusion_relative_quat_{args.filter}",
        "parent": args.parent,
        "child": args.child,
        "segment_twist_axis": [0.0, 0.0, 1.0],
        "parent_up_sensor": [round(float(v), 4) for v in parent_up],
        "child_up_sensor": [round(float(v), 4) for v in child_up],
        "sample_count": int(len(t)),
        "duration_s": float(t[-1]),
        "sample_rate_hz_median": float(1.0 / ts_med),
        "sample_dt_ms_p5_p95": [float(np.percentile(dts, 5) * 1000.0), float(np.percentile(dts, 95) * 1000.0)],
        "parent_bias_dps": [round(float(b), 4) for b in parent_bias],
        "child_bias_dps": [round(float(b), 4) for b in child_bias],
        "bias_sample_count": int(still.sum()),
        "tare_sample_count": int(tare.sum()),
        "bias_window_s": [float(t[still][0]), float(t[still][-1])],
        "tare_window_s": [float(t[tare][0]), float(t[tare][-1])],
        "parent_filter_info": parent_info,
        "child_filter_info": child_info,
        "twist_min_deg": float(np.min(twist_deg)),
        "twist_max_deg": float(np.max(twist_deg)),
        "twist_final_deg": float(twist_deg[-1]),
        "return_window_s": args.return_window_seconds,
    }
    summary.update(window_metrics(t, twist_deg, swing_deg, tare, args))
    if twist_v0 is not None:
        last = t >= (t[-1] - args.return_window_seconds)
        summary["v0_baseline"] = {
            "twist_final_deg": float(twist_v0[-1]),
            "return_to_zero_estimate_deg": float(np.mean(twist_v0[last])),
            "twist_min_deg": float(np.min(twist_v0)),
            "twist_max_deg": float(np.max(twist_v0)),
        }
    if warnings:
        summary["warnings"] = warnings
    return summary


# ---------------------------------------------------------------------------
# closed-loop / return-to-neutral post-processing (v1.5, optional, never default)
#
# This NEVER touches the VQF/Madgwick quaternions or the raw tared twist. It only
# reads the already-computed tared twist scalar and, IF the start/end neutral holds
# are clean enough, subtracts a drift ramp pinned by the boundary assumption
# start_pose == end_pose. It is a SELF-CONSISTENCY correction, not a ground-truth
# correction: it can only flatten residual drift/bias between the two neutral holds.
# It cannot prove the mid-trial ROM, peak twist, or swing-twist decomposition is
# physically accurate -- that still requires mocap / optical / mechanical-jig truth.
# ---------------------------------------------------------------------------

def assess_still_quality(
    label: str,
    mask: np.ndarray,
    t: np.ndarray,
    parent_gyr_raw: np.ndarray,
    child_gyr_raw: np.ndarray,
    parent_rest: np.ndarray | None,
    child_rest: np.ndarray | None,
    args: argparse.Namespace,
) -> dict[str, object]:
    """Audit a neutral-hold window. `ok=False` means the window is too dirty to anchor a correction."""
    idx = np.where(mask)[0]
    n = int(idx.size)
    q: dict[str, object] = {
        "label": label,
        "window_s": None,
        "sample_count": n,
        "gyro_std_max_dps": None,
        "rest_detected_fraction": None,
        "ok": True,
        "reasons": [],
        "notes": [],
    }
    if n < 3:
        q["ok"] = False
        q["reasons"].append(f"{label}: only {n} samples (need >= 3)")
        return q

    q["window_s"] = [float(t[idx[0]]), float(t[idx[-1]])]
    pstd = float(np.max(parent_gyr_raw[mask].std(axis=0)))
    cstd = float(np.max(child_gyr_raw[mask].std(axis=0)))
    gstd = max(pstd, cstd)
    q["gyro_std_max_dps"] = round(gstd, 4)
    if gstd > args.still_gyro_std_max:
        q["ok"] = False
        q["reasons"].append(
            f"{label}: gyro std {gstd:.2f} dps > {args.still_gyro_std_max} dps (window not still)"
        )

    if parent_rest is not None and child_rest is not None:
        rf = float(min(np.mean(parent_rest[mask]), np.mean(child_rest[mask])))
        q["rest_detected_fraction"] = round(rf, 3)
        if rf < args.min_rest_fraction:
            q["ok"] = False
            q["reasons"].append(
                f"{label}: VQF rest_detected fraction {rf:.2f} < {args.min_rest_fraction}"
            )
    else:
        q["notes"].append(f"{label}: rest detection unavailable (filter=madgwick), gate skipped")
    return q


def resolve_end_still_mask(t: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    if args.end_still_start_s is not None or args.end_still_end_s is not None:
        e0 = args.end_still_start_s if args.end_still_start_s is not None else float(t[-1] - args.return_window_seconds)
        e1 = args.end_still_end_s if args.end_still_end_s is not None else float(t[-1])
        return (t >= e0) & (t <= e1)
    return t >= (t[-1] - args.return_window_seconds)


def apply_closed_loop(result: FusionResult, pairs: FusionPair, args: argparse.Namespace) -> FusionResult:
    """Optional return-to-neutral drift correction on the tared twist scalar. No-op unless requested."""
    method = getattr(args, "closed_loop", "none")
    if method == "none":
        return result

    t = result.t_s
    twist = result.twist_deg
    n = len(t)

    start_mask = result.tare_mask if result.tare_mask is not None else (t <= args.tare_seconds)
    end_mask = resolve_end_still_mask(t, args)
    return_mask = t >= (t[-1] - args.return_window_seconds)  # same window as the existing return-to-zero metric

    raw_rtz = float(np.mean(twist[return_mask])) if np.any(return_mask) else float(twist[-1])

    start_q = assess_still_quality(
        "start_still", start_mask, t, pairs.parent_gyr_dps, pairs.child_gyr_dps,
        result.parent_rest, result.child_rest, args,
    )
    end_q = assess_still_quality(
        "end_still", end_mask, t, pairs.parent_gyr_dps, pairs.child_gyr_dps,
        result.parent_rest, result.child_rest, args,
    )

    reasons: list[str] = list(start_q["reasons"]) + list(end_q["reasons"])

    # Hard gate: end-still overlapping the tare/start-still window makes return-to-zero circular.
    overlap = bool(np.any(start_mask & end_mask))
    if overlap:
        reasons.append(
            "end-still window overlaps start-still/tare window (circular; return-to-zero would be artificially near zero)"
        )

    # Soft gate: low sample rate degrades but does not invalidate the boundary equality.
    dts = np.diff(t)
    ts_med = float(np.median(dts)) if dts.size else 0.0
    rate = (1.0 / ts_med) if ts_med > 0 else 0.0
    degraded_reasons: list[str] = []
    if rate < args.min_sample_rate_hz:
        degraded_reasons.append(
            f"sample rate {rate:.1f} Hz < {args.min_sample_rate_hz} Hz (low; correction coarse/degraded if applied)"
        )

    # Candidate correction parameters from the boundary assumption start_pose == end_pose.
    si = np.where(start_mask)[0]
    ei = np.where(end_mask)[0]
    e_start = float(np.mean(twist[start_mask])) if si.size else 0.0
    e_end = float(np.mean(twist[end_mask])) if ei.size else 0.0
    candidate_ok = si.size >= 3 and ei.size >= 3
    if method == "linear":
        # one global ramp anchored at the window centroids
        t_a = float(np.mean(t[start_mask])) if si.size else float(t[0])
        t_b = float(np.mean(t[end_mask])) if ei.size else float(t[-1])
    else:  # piecewise: flat across each hold, ramp only across the moving interval
        t_a = float(t[si[-1]]) if si.size else float(t[0])
        t_b = float(t[ei[0]]) if ei.size else float(t[-1])
    if t_b - t_a < 1e-6:
        candidate_ok = False
        reasons.append(f"{method}: start/end anchors not separable in time (t_b - t_a < 1e-6)")

    candidate_correction = np.zeros(n)
    if candidate_ok:
        frac = np.clip((t - t_a) / (t_b - t_a), 0.0, 1.0)
        candidate_correction = e_start + (e_end - e_start) * frac
    candidate_total = float(e_end - e_start)

    hard_fail = (not start_q["ok"]) or (not end_q["ok"]) or overlap or (not candidate_ok)
    applied = not hard_fail
    degraded = applied and bool(degraded_reasons)

    if applied:
        correction = candidate_correction
        corrected = twist - correction
        method_str = method if not degraded else f"{method}(degraded)"
    else:
        # Refuse: do NOT auto-correct dirty/circular data. Corrected == raw, correction == 0.
        correction = np.zeros(n)
        corrected = twist.copy()
        method_str = f"refused({method})"

    correction_at_return = float(np.mean(correction[return_mask])) if np.any(return_mask) else 0.0
    corrected_rtz = float(np.mean(corrected[return_mask])) if np.any(return_mask) else float(corrected[-1])

    result.raw_fusion_twist_deg = twist.copy()
    result.closed_loop_twist_deg = corrected
    result.correction_deg = correction

    all_reasons = reasons + degraded_reasons
    closed_loop_summary: dict[str, object] = {
        "correction_method": method_str,
        "requested_method": method,
        "applied": applied,
        "degraded": degraded,
        "filter": args.filter,
        "reasons": all_reasons,
        "raw_return_to_zero_deg": raw_rtz,
        "corrected_return_to_zero_deg": corrected_rtz,
        "correction_deg": correction_at_return,           # == raw_return_to_zero - corrected_return_to_zero
        "candidate_correction_deg": candidate_total,      # what the ramp WOULD remove, even when refused
        "sample_rate_hz_median": rate,
        "end_still_window_s": end_q["window_s"],
        "start_still_quality": start_q,
        "end_still_quality": end_q,
        "disclaimer": (
            "closed-loop only improves start/end self-consistency; it does NOT prove the accuracy of "
            "mid-trial ROM, peak twist, or the swing-twist decomposition. Swing and q_display are left "
            "uncorrected. Absolute accuracy still requires mocap / optical / mechanical-jig ground truth."
        ),
    }
    result.summary["closed_loop"] = closed_loop_summary
    return result


# ---------------------------------------------------------------------------
# outputs
# ---------------------------------------------------------------------------

def write_outputs(result: FusionResult, stem: str, args: argparse.Namespace) -> tuple[Path, Path, Path]:
    plots_dir = Path(__file__).resolve().parent / "plots"
    plots_dir.mkdir(exist_ok=True)
    out_csv = args.out_csv or plots_dir / f"{stem}_fusion_{args.filter}.csv"
    out_svg = args.plot or plots_dir / f"{stem}_fusion_{args.filter}.svg"
    out_json = args.summary or plots_dir / f"{stem}_fusion_{args.filter}_summary.json"

    header = ["t_s"]
    header += [f"q{ax}_parent" for ax in "wxyz"]
    header += [f"q{ax}_child" for ax in "wxyz"]
    header += [f"q{ax}_rel" for ax in "wxyz"]
    header += [f"q{ax}_display" for ax in "wxyz"]
    header += ["twist_deg", "swing_deg"]
    if result.twist_v0_deg is not None:
        header.append("twist_v0_deg")
    has_closed_loop = result.closed_loop_twist_deg is not None
    if has_closed_loop:
        header += ["raw_fusion_twist_deg", "closed_loop_twist_deg", "correction_deg"]
    with out_csv.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(header)
        for i in range(len(result.t_s)):
            row = [f"{result.t_s[i]:.4f}"]
            row += [f"{v:.6f}" for v in result.q_parent[i]]
            row += [f"{v:.6f}" for v in result.q_child[i]]
            row += [f"{v:.6f}" for v in result.q_rel[i]]
            row += [f"{v:.6f}" for v in result.q_display[i]]
            row += [f"{result.twist_deg[i]:.4f}", f"{result.swing_deg[i]:.4f}"]
            if result.twist_v0_deg is not None:
                row.append(f"{result.twist_v0_deg[i]:.4f}")
            if has_closed_loop:
                row += [
                    f"{result.raw_fusion_twist_deg[i]:.4f}",
                    f"{result.closed_loop_twist_deg[i]:.4f}",
                    f"{result.correction_deg[i]:.4f}",
                ]
            writer.writerow(row)

    write_svg(result, out_svg, stem, args.filter)

    with out_json.open("w", encoding="utf-8") as fh:
        json.dump(result.summary, fh, indent=2)
        fh.write("\n")
    return out_csv, out_svg, out_json


def write_svg(result: FusionResult, path: Path, stem: str, filter_name: str) -> None:
    width, height, pad = 960, 420, 56
    t, series = result.t_s, [(result.twist_deg, "#0a6cff", f"fusion {filter_name} twist (raw)")]
    if result.twist_v0_deg is not None:
        series.append((result.twist_v0_deg, "#d0342c", "v0 gyro-only twist"))
    if result.correction_deg is not None and bool(np.any(result.correction_deg != 0.0)):
        series.append((result.closed_loop_twist_deg, "#8e44ad", "closed-loop twist (corrected)"))
    series.append((result.swing_deg, "#3aa655", "swing magnitude"))

    all_y = np.concatenate([s for s, _, _ in series])
    y_min, y_max = float(np.min(all_y)), float(np.max(all_y))
    if y_max - y_min < 1.0:
        y_min, y_max = y_min - 1.0, y_max + 1.0
    span_t = max(1e-9, float(t[-1] - t[0]))
    span_y = y_max - y_min

    def sx(x: float) -> float:
        return pad + (x - t[0]) / span_t * (width - 2 * pad)

    def sy(y: float) -> float:
        return height - pad - (y - y_min) / span_y * (height - 2 * pad)

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        f'<rect width="{width}" height="{height}" fill="white"/>',
        f'<text x="{pad}" y="24" font-family="sans-serif" font-size="15">{stem}: relative twist, fusion vs baseline (deg)</text>',
    ]
    if y_min < 0 < y_max:
        zero_y = sy(0.0)
        parts.append(f'<line x1="{pad}" y1="{zero_y:.1f}" x2="{width - pad}" y2="{zero_y:.1f}" stroke="#999" stroke-dasharray="4 4"/>')
    for idx, (values, color, label) in enumerate(series):
        pts = " ".join(f"{sx(float(x)):.1f},{sy(float(y)):.1f}" for x, y in zip(t, values))
        parts.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="1.6"/>')
        parts.append(
            f'<text x="{width - pad - 230}" y="{44 + idx * 18}" font-family="sans-serif" font-size="13" fill="{color}">{label}</text>'
        )
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        xv = t[0] + frac * span_t
        yv = y_min + frac * span_y
        parts.append(f'<text x="{sx(float(xv)):.1f}" y="{height - pad + 18}" font-family="sans-serif" font-size="11" text-anchor="middle">{xv:.0f}s</text>')
        parts.append(f'<text x="{pad - 8}" y="{sy(float(yv)):.1f}" font-family="sans-serif" font-size="11" text-anchor="end">{yv:.0f}</text>')
    parts.append("</svg>")
    path.write_text("\n".join(parts), encoding="utf-8")


# ---------------------------------------------------------------------------
# synthetic self-test: known twist/bend trajectory through flipped mountings
# ---------------------------------------------------------------------------

def smoothstep(t: float, t0: float, t1: float) -> float:
    if t <= t0:
        return 0.0
    if t >= t1:
        return 1.0
    x = (t - t0) / (t1 - t0)
    return x * x * (3.0 - 2.0 * x)


def demo_truth(t: float) -> tuple[float, float]:
    twist = 30.0 * (smoothstep(t, 8, 12) - smoothstep(t, 14, 18) - smoothstep(t, 20, 24) + smoothstep(t, 26, 30))
    bend = 20.0 * (smoothstep(t, 32, 34) - smoothstep(t, 35, 37))
    return twist, bend


def make_demo_pairs(rate_hz: float = 100.0, duration_s: float = 40.0, seed: int = 7) -> tuple[FusionPair, np.ndarray]:
    rng = np.random.default_rng(seed)
    n = int(duration_s * rate_hz)
    t = np.arange(n) / rate_hz
    q_mount_parent = quat_from_axis_angle(np.array([0.0, 1.0, 0.0]), math.radians(-90.0))  # sensor +x is up
    q_mount_child = quat_from_axis_angle(np.array([0.0, 1.0, 0.0]), math.radians(90.0))    # sensor -x is up

    truth = np.zeros(n)
    q_child_w = np.zeros((n, 4))
    for i, ti in enumerate(t):
        twist, bend = demo_truth(float(ti))
        truth[i] = twist
        q_seg = qmul(
            quat_from_axis_angle(np.array([1.0, 0.0, 0.0]), math.radians(bend)),
            quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), math.radians(twist)),
        )
        q_child_w[i] = qmul(q_seg, q_mount_child)

    def gyro_from_quats(quats: np.ndarray) -> np.ndarray:
        gyr = np.zeros((len(quats), 3))
        for i in range(1, len(quats)):
            dq = qmul(qconj(quats[i - 1]), quats[i])
            dq = dq if dq[0] >= 0 else -dq
            angle = 2.0 * math.atan2(np.linalg.norm(dq[1:]), dq[0])
            axis = dq[1:] / max(1e-12, np.linalg.norm(dq[1:]))
            gyr[i] = np.degrees(axis * angle * rate_hz)
        return gyr

    def acc_from_quats(quats: np.ndarray) -> np.ndarray:
        ez = np.array([0.0, 0.0, 1.0])
        return np.array([quat_rotate(qconj(q), ez) * 1000.0 for q in quats])

    q_parent_w = np.tile(q_mount_parent, (n, 1))
    pairs = FusionPair(
        t_s=t,
        parent_acc_mg=acc_from_quats(q_parent_w) + rng.normal(0, 2.0, (n, 3)),
        parent_gyr_dps=gyro_from_quats(q_parent_w) + rng.normal(0, 0.05, (n, 3)) + np.array([0.3, -0.2, 0.1]),
        child_acc_mg=acc_from_quats(q_child_w) + rng.normal(0, 2.0, (n, 3)),
        child_gyr_dps=gyro_from_quats(q_child_w) + rng.normal(0, 0.05, (n, 3)) + np.array([-0.4, 0.25, -0.15]),
    )
    return pairs, truth


def run_demo(args: argparse.Namespace) -> int:
    pairs, truth = make_demo_pairs()
    result = run_pipeline(pairs, args)
    result = apply_closed_loop(result, pairs, args)  # no-op unless --closed-loop given; never touches result.twist_deg
    err = result.twist_deg - truth
    max_err = float(np.max(np.abs(err)))
    rtz = abs(result.summary["return_to_zero_estimate_deg"])
    bend_zone = (pairs.t_s >= 31.0) & (pairs.t_s <= 38.5)
    false_twist = float(np.max(np.abs(result.twist_deg[bend_zone])))
    ok = max_err < 2.0 and rtz < 0.5 and false_twist < 2.0
    print(f"demo filter={args.filter}")
    print(f"demo max |twist error|: {max_err:.3f} deg (limit 2.0)")
    print(f"demo return-to-zero:    {rtz:.3f} deg (limit 0.5)")
    print(f"demo false twist in pure bending: {false_twist:.3f} deg (limit 2.0)")
    print("demo RESULT:", "PASS" if ok else "FAIL")
    out_csv, out_svg, out_json = write_outputs(result, "demo", args)
    print(f"wrote {out_csv}\nwrote {out_svg}\nwrote {out_json}")
    return 0 if ok else 1


# ---------------------------------------------------------------------------

# Default argument values, shared by the CLI parser and by callers (e.g. batch_analyze.py)
# so the two never drift apart.
ARG_DEFAULTS: dict[str, object] = {
    "input": None,
    "demo": False,
    "filter": "vqf",
    "parent": "IMU1",
    "child": "IMU2",
    "parent_up": "auto",
    "child_up": "auto",
    "bias_seconds": 5.0,
    "bias_start_s": None,
    "bias_end_s": None,
    "tare_seconds": 5.0,
    "tare_start_s": None,
    "tare_end_s": None,
    "return_window_seconds": 2.0,
    "drift_start_s": None,
    "drift_end_s": None,
    "no_v0_baseline": False,
    "closed_loop": "none",
    "end_still_start_s": None,
    "end_still_end_s": None,
    "still_gyro_std_max": 0.5,
    "min_rest_fraction": 0.5,
    "min_sample_rate_hz": 25.0,
    "out_csv": None,
    "plot": None,
    "summary": None,
}


def make_args(**overrides: object) -> argparse.Namespace:
    """Build an args namespace with the same defaults as the CLI. For programmatic callers."""
    values = dict(ARG_DEFAULTS)
    values.update(overrides)
    return argparse.Namespace(**values)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="SpineSense Part 2 v1: per-IMU 6D fusion -> relative quaternion -> tare -> swing-twist."
    )
    parser.add_argument("--input", type=Path, help="Serial log (same formats as twist_bench_v0).")
    parser.add_argument("--demo", action="store_true", help="Run synthetic self-test with known twist/bend truth.")
    parser.add_argument("--filter", choices=("vqf", "madgwick"), default="vqf")
    parser.add_argument("--parent", default="IMU1")
    parser.add_argument("--child", default="IMU2")
    parser.add_argument("--parent-up", choices=("auto", *NAMED_UP), default="auto",
                        help="Longitudinal up axis in parent sensor frame. Default: estimate from still-window gravity.")
    parser.add_argument("--child-up", choices=("auto", *NAMED_UP), default="auto")
    parser.add_argument("--bias-seconds", type=float, default=5.0)
    parser.add_argument("--bias-start-s", type=float, help="Explicit bias window start; overrides --bias-seconds.")
    parser.add_argument("--bias-end-s", type=float, help="Explicit bias window end; overrides --bias-seconds.")
    parser.add_argument("--tare-seconds", type=float, default=5.0)
    parser.add_argument("--tare-start-s", type=float, help="Explicit tare window start; overrides --tare-seconds.")
    parser.add_argument("--tare-end-s", type=float, help="Explicit tare window end; overrides --tare-seconds.")
    parser.add_argument("--return-window-seconds", type=float, default=2.0)
    parser.add_argument("--drift-start-s", type=float)
    parser.add_argument("--drift-end-s", type=float)
    parser.add_argument("--no-v0-baseline", action="store_true")
    parser.add_argument(
        "--closed-loop", choices=("none", "linear", "piecewise"), default="none",
        help="Optional return-to-neutral drift correction on the tared twist (default none, never overrides raw).",
    )
    parser.add_argument("--end-still-start-s", type=float,
                        help="Start of the end neutral-hold window for closed-loop. Default: last --return-window-seconds.")
    parser.add_argument("--end-still-end-s", type=float, help="End of the end neutral-hold window. Default: trial end.")
    parser.add_argument("--still-gyro-std-max", type=float, default=0.5,
                        help="Max gyro std (dps) in a neutral hold before closed-loop refuses to correct. Default 0.5.")
    parser.add_argument("--min-rest-fraction", type=float, default=0.5,
                        help="Min VQF rest_detected fraction in a neutral hold before closed-loop refuses. Default 0.5.")
    parser.add_argument("--min-sample-rate-hz", type=float, default=25.0,
                        help="Below this rate closed-loop still applies but is flagged degraded. Default 25.")
    parser.add_argument("--out-csv", type=Path)
    parser.add_argument("--plot", type=Path)
    parser.add_argument("--summary", type=Path)
    args = parser.parse_args()

    if args.demo:
        return run_demo(args)
    if args.input is None:
        parser.error("Provide --input or use --demo.")

    pairs = load_fusion_pairs(args.input, args.parent, args.child)
    result = run_pipeline(pairs, args)
    result = apply_closed_loop(result, pairs, args)
    out_csv, out_svg, out_json = write_outputs(result, args.input.stem, args)

    s = result.summary
    print(f"algorithm: {s['algorithm']}")
    print(f"pairs: {s['sample_count']}  duration: {s['duration_s']:.1f} s  rate: {s['sample_rate_hz_median']:.1f} Hz")
    print(f"parent up (sensor): {s['parent_up_sensor']}  child up (sensor): {s['child_up_sensor']}")
    print(f"twist range: {s['twist_min_deg']:.2f} .. {s['twist_max_deg']:.2f} deg  final: {s['twist_final_deg']:.2f} deg")
    print(f"return-to-zero: {s['return_to_zero_estimate_deg']:.2f} deg")
    print(f"tare-window twist RMS: {s['tare_window_twist_rms_deg']:.3f} deg  swing max: {s['swing_max_deg']:.2f} deg")
    if "v0_baseline" in s:
        b = s["v0_baseline"]
        print(f"v0 baseline final: {b['twist_final_deg']:.2f} deg  return-to-zero: {b['return_to_zero_estimate_deg']:.2f} deg")
    if "closed_loop" in s:
        cl = s["closed_loop"]
        print(f"closed-loop [{cl['correction_method']}] applied={cl['applied']} degraded={cl['degraded']}")
        print(f"  return-to-zero raw: {cl['raw_return_to_zero_deg']:.2f} deg  corrected: {cl['corrected_return_to_zero_deg']:.2f} deg  (correction {cl['correction_deg']:.2f} deg)")
        if not cl["applied"]:
            print(f"  candidate correction NOT applied: {cl['candidate_correction_deg']:.2f} deg")
        for r in cl["reasons"]:
            print(f"  closed-loop note: {r}")
        print("  closed-loop is self-consistency only; it does NOT validate mid-trial ROM/peak/decomposition.")
    for w in s.get("warnings", []):
        print(f"WARNING: {w}")
    print(f"wrote {out_csv}\nwrote {out_svg}\nwrote {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
