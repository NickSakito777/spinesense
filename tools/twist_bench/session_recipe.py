from __future__ import annotations

"""Reusable pieces of a session processing recipe.

A *session recipe* turns one raw five-IMU log plus its optical-motion-capture
counterpart into scored movement blocks.  The parts that vary per session --
segment boundaries, the clock fit that maps mocap time onto IMU time, which
protocol blocks a recording actually covers -- belong to the calling script.
The parts that do not vary live here: loading, twist/swing decomposition,
bout segmentation, and block scoring.

Nothing in this module reads a fixed path or carries session constants.  Every
function takes what it needs as an argument.

See ``session_recipe_example.py`` for a minimal recipe built on top of this.

Coordinate and sign conventions
-------------------------------
``SIGN`` fixes the chain sign convention.  It is frozen deliberately: refitting
signs per block lets a sign error hide as a good correlation, so the convention
is asserted once and violations are allowed to show up as bad scores.

The on-chip SFLP quaternion is a game-rotation quaternion -- no magnetometer,
so absolute heading is unobservable.  Everything here works with *relative*
orientation between adjacent segments, and re-tares against a still window
before each bout rather than assuming a shared global yaw.
"""

from pathlib import Path
from typing import Callable, Iterable, Sequence

import numpy as np

import five_imu_fusion as fiv
import signed_diagnostic as sd
import twist_bench_fusion as pf
import twist_bench_v0 as v0
import validation3_cluster_orientation as v3

# Frozen chain sign convention. Not re-fit per block -- see module docstring.
SIGN = {"sacrum_to_lower": +1, "lower_to_mid": -1, "mid_to_upper": -1, "upper_to_sternum": +1}
CHAIN = list(SIGN)


# --------------------------------------------------------------------------
# Loading
# --------------------------------------------------------------------------

def normalize_interp_quat_series(
    qraw: Sequence[object],
    label: str = "",
    bad_counts: dict[str, int] | None = None,
) -> np.ndarray:
    """Clean one SFLP quaternion series: interpolate bad rows, normalise, unwrap sign.

    Isolated non-finite or zero-norm rows are linearly interpolated from their
    neighbours.  Consecutive quaternions are then sign-aligned so that a series
    crossing the q/-q ambiguity does not read as a 360-degree jump.

    The interpolation is applied to the derived series only; the raw log is
    never modified.  Pass ``bad_counts`` to record how many rows needed repair
    -- a high count is a hardware or link problem, not something to silently
    smooth over.
    """
    q = np.asarray(qraw, dtype=float)
    finite = np.all(np.isfinite(q), axis=1)
    norms = np.linalg.norm(np.where(np.isfinite(q), q, 0.0), axis=1)
    bad = (~finite) | (norms < 1e-9)
    if bad_counts is not None:
        bad_counts[label] = int(np.count_nonzero(bad))
    if np.any(bad):
        x = np.arange(len(q))
        good = ~bad
        if int(np.count_nonzero(good)) < 2:
            raise ValueError(f"{label or 'series'} has too few good SFLP quaternion rows")
        for col in range(4):
            q[bad, col] = np.interp(x[bad], x[good], q[good, col])
    q = q / np.linalg.norm(q, axis=1)[:, None]
    for i in range(1, len(q)):
        if float(np.dot(q[i - 1], q[i])) < 0.0:
            q[i] = -q[i]
    return q


def tolerant_load_five_streams(
    path: Path,
    layout: dict[str, str],
    bad_counts: dict[str, int] | None = None,
) -> tuple[np.ndarray, dict[str, fiv.ImuStream]]:
    """Load a five-IMU log, keeping only frames where all five devices reported.

    Tolerant of dropped rows: a frame missing any of the five IMUs is discarded
    rather than filled.  Timestamps are re-based to zero.

    This delegates parsing to ``twist_bench_v0``, which refuses a log containing
    an unhandled timestamp reset.  Do not work around that -- split the log
    first.  A reset absorbed into a time-keyed dictionary silently overwrites or
    reorders samples and produces plausible-looking but wrong synchronisation.
    """
    text = path.read_text(encoding="utf-8", errors="replace")
    records = v0.parse_serial_text(text)
    if not records:
        records = v0.parse_long_table_rows(v0.read_dict_rows(text))
    imu_ids = {imu.upper() for imu in layout.values()}
    by_time: dict[float, dict[str, object]] = {}
    for record in records:
        imu = record.imu.upper()
        if imu in imu_ids:
            by_time.setdefault(record.t_s, {})[imu] = record

    rows_t: list[float] = []
    acc: dict[str, list[tuple[float, float, float]]] = {imu: [] for imu in imu_ids}
    gyr: dict[str, list[tuple[float, float, float]]] = {imu: [] for imu in imu_ids}
    quat: dict[str, list[object]] = {imu: [] for imu in imu_ids}
    for t_s in sorted(by_time):
        group = by_time[t_s]
        if all(imu in group for imu in imu_ids):
            rows_t.append(t_s)
            for imu in imu_ids:
                sample = group[imu]
                acc[imu].append((sample.ax_mg, sample.ay_mg, sample.az_mg))
                gyr[imu].append((sample.gx_dps, sample.gy_dps, sample.gz_dps))
                quat[imu].append((np.nan, np.nan, np.nan, np.nan) if sample.sflp_quat is None else sample.sflp_quat)

    if not rows_t:
        raise ValueError(f"{path.name}: no frame contained all of {sorted(imu_ids)}")

    t = np.asarray(rows_t, dtype=float)
    t = t - t[0]
    streams = {
        imu: fiv.ImuStream(
            imu=imu,
            acc_mg=np.asarray(acc[imu], dtype=float),
            gyr_dps=np.asarray(gyr[imu], dtype=float),
            q_sflp=normalize_interp_quat_series(quat[imu], f"{path.stem}:{imu}", bad_counts),
        )
        for imu in sorted(imu_ids)
    }
    return t, streams


def signals_for(mocap: Path) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Load the signed mocap reference: flexion, lateral bend, axial rotation."""
    tm, flex, lat, axial = sd.mocap_signed(mocap)
    return tm, {"flex": flex, "lat": lat, "axial": axial}


# --------------------------------------------------------------------------
# Orientation decomposition
# --------------------------------------------------------------------------

def qrel(res: fiv.FiveImuResult, parent: str, child: str) -> np.ndarray:
    """Relative orientation of ``child`` expressed in the ``parent`` segment frame."""
    return pf.qmul(pf.qconj(res.sensors[parent].q_segment), res.sensors[child].q_segment)


def pre_mask(ti: np.ndarray, a: float, b: float, lo: float) -> np.ndarray:
    """Still window immediately before a bout, used as the local re-tare reference.

    Takes IMU samples covering mocap time ``lo - 1.2`` to ``lo - 0.2`` seconds
    under the clock fit ``t_imu = a * t_mocap + b``.  Falls back to the first
    8 seconds when that window is too short, which is why every recording should
    open with a genuinely still period.
    """
    mask = (ti >= a * (lo - 1.2) + b) & (ti <= a * (lo - 0.2) + b)
    if int(np.count_nonzero(mask)) < 3:
        mask = ti <= min(8.0, float(ti[-1]))
    return mask


def local_swing(
    res: fiv.FiveImuResult,
    a: float,
    b: float,
    lo: float,
    relation: str = "sacrum_to_upper",
) -> np.ndarray:
    """Bend (swing) angle for one relation, re-tared against the still window before the bout.

    ``relation`` names a key in ``res.relations`` and therefore depends on the
    layout preset: the anatomical presets use ``sacrum_to_upper``, the bench
    preset names its links ``bottom_to_top`` and so on.
    """
    q = res.relations[relation].q_rel
    q0 = pf.quat_average(q[pre_mask(res.t_s, a, b, lo)])
    qt = pf.qmul(pf.qconj(q0)[None, :], q)
    _, swing = pf.swing_twist_deg(qt, pf.SEGMENT_TWIST_AXIS)
    return swing


def local_twist(
    res: fiv.FiveImuResult,
    a: float,
    b: float,
    lo: float,
    root: str = "sacrum",
    tip: str = "sternum",
) -> np.ndarray:
    """Whole-trunk axial twist: ``tip`` relative to ``root``, locally re-tared.

    Name the two ends explicitly rather than reusing a link index.  Under a
    mis-assigned placement map, a role name can resolve to a different physical
    sensor than intended -- a readout labelled upper-to-sternum then silently
    measures sacrum-to-sternum, which is a different quantity with a plausible
    shape.  Naming both ends makes that failure visible instead of silent.

    True upper-to-sternum is a short upper-thorax cross-check and is *not* the
    whole-trunk axial reference the mocap comparison uses.

    ``root``/``tip`` are keys in ``res.sensors`` and follow the layout preset.
    """
    q = qrel(res, root, tip)
    q0 = pf.quat_average(q[pre_mask(res.t_s, a, b, lo)])
    qt = pf.qmul(pf.qconj(q0)[None, :], q)
    twist, _ = pf.swing_twist_deg(qt, pf.SEGMENT_TWIST_AXIS)
    return -pf.unwrap_deg(twist)


def tared_bend_quat(
    res: fiv.FiveImuResult,
    a: float,
    b: float,
    lo: float,
    relation: str = "sacrum_to_upper",
) -> np.ndarray:
    """Locally re-tared relative quaternion -- the quaternion form of ``local_swing``."""
    q = res.relations[relation].q_rel
    q0 = pf.quat_average(q[pre_mask(res.t_s, a, b, lo)])
    return pf.qmul(pf.qconj(q0)[None, :], q)


def swing_rotvec_deg(qt: np.ndarray, axis: np.ndarray = pf.SEGMENT_TWIST_AXIS) -> np.ndarray:
    """Signed swing rotation vector in degrees, per sample.

    Removes the twist-about-``axis`` component (the same decomposition as
    ``swing_twist_deg``), then log-maps the residual swing quaternion. Columns
    are ``[x, y, z]`` with ``z ~ 0`` by construction; ``x``/``y`` are the two
    horizontal signed bend components.

    Signed, unlike a swing *magnitude*: direction is what distinguishes a left
    bend from a right one, and a magnitude-only readout throws that away.
    Which of x/y is sagittal and which is frontal depends on mounting heading,
    which a magnetometer-free system cannot resolve -- so both are emitted and
    the mapping is left to whatever consumes them.
    """
    q = pf.qnormalize(qt)
    proj = q[..., 1:] @ axis
    q_twist = np.zeros_like(q)
    q_twist[..., 0] = q[..., 0]
    q_twist[..., 1:] = proj[..., None] * axis
    norms = np.linalg.norm(q_twist, axis=-1, keepdims=True)
    q_twist = np.where(norms < 1e-9, np.array([1.0, 0.0, 0.0, 0.0]), q_twist / np.maximum(norms, 1e-12))
    q_swing = pf.qmul(q, pf.qconj(q_twist))
    w = q_swing[..., 0].copy()
    vec = q_swing[..., 1:].copy()
    flip = w < 0.0
    w[flip] *= -1.0
    vec[flip] *= -1.0
    w = np.clip(w, -1.0, 1.0)
    phi = 2.0 * np.arccos(w)                       # rotation magnitude, rad
    s = np.sqrt(np.maximum(1.0 - w * w, 0.0))
    unit = np.where(s[..., None] < 1e-9, 0.0, vec / np.maximum(s[..., None], 1e-12))
    return np.degrees(phi[..., None] * unit)


def seg_twists(
    logpath: Path,
    layout_preset: str,
    filter: str = "vqf",
    chain: Sequence[str] = tuple(CHAIN),
    whole: str = "sacrum_to_sternum",
):
    """Per-link twist along the chain, plus the whole-trunk end-to-end twist.

    Returns ``(t_s, {link: twist_deg}, whole_twist_deg)``.  Comparing the sum of
    the per-link twists against the whole-trunk value is the chain-consistency
    check: they should track, and a large divergence means a link is mis-signed
    or a sensor has drifted out of the chain.
    """
    res = fiv.run_pipeline(logpath, fiv.make_args(layout_preset=layout_preset, filter=filter))
    segs = {r: res.relations[r].twist_deg for r in chain}
    long = res.relations[whole].twist_deg
    return res.t_s, segs, long


# --------------------------------------------------------------------------
# Segmentation
# --------------------------------------------------------------------------

def moving_mean(x: np.ndarray, n: int) -> np.ndarray:
    n = max(3, int(n))
    return np.convolve(x, np.ones(n) / n, mode="same")


def movement_bouts(
    tm: np.ndarray,
    flex: np.ndarray,
    lat: np.ndarray,
    axial: np.ndarray,
) -> dict[str, list[tuple[float, float]]]:
    """Segment the six directed movement classes from the signed mocap channels.

    Each class is segmented on its own channel with the other two supplied as
    purity constraints, so a lateral bend contaminated by rotation does not
    count as a clean bend.  Extension (``flex-``) uses looser thresholds because
    its range of motion is intrinsically smaller than flexion's.
    """
    return {
        "flex+": v3.segment(tm, flex, +1, (lat, axial)),
        "flex-": v3.segment(tm, flex, -1, (lat, axial), hi=8, lo=3, opurity=12),
        "lat+": v3.segment(tm, lat, +1, (flex, axial)),
        "lat-": v3.segment(tm, lat, -1, (flex, axial)),
        "tw+": v3.segment(tm, axial, +1, (flex, lat)),
        "tw-": v3.segment(tm, axial, -1, (flex, lat)),
    }


def peak_bouts(
    tm: np.ndarray,
    sig: np.ndarray,
    window: list[float],
    sign: int,
    min_dist_s: float,
    base_s: float = 24.0,
    frac: float = 0.35,
    min_dur_s: float = 0.8,
) -> list[tuple[float, float]]:
    """Deterministic peak-based bout detector for a single channel.

    Subtracts a long moving-mean baseline, finds local maxima above a threshold,
    greedily keeps the tallest peaks that are at least ``min_dist_s`` apart, and
    grows each bout outwards until the signal falls below ``frac`` of its peak.

    Deterministic by construction -- same input, same bouts, no random seed.
    ``sign`` selects direction: +1, -1, or 0 for magnitude.
    """
    dt = float(np.median(np.diff(tm)))
    baseline = moving_mean(sig, int(base_s / dt))
    delta = sig - baseline
    y = np.abs(delta) if sign == 0 else sign * delta
    mask = (tm >= window[0]) & (tm <= window[1])
    idx = np.where(mask)[0]
    if len(idx) < 3:
        return []
    yy = y.copy()
    yy[~mask] = -np.inf
    local = np.where((yy[1:-1] > yy[:-2]) & (yy[1:-1] >= yy[2:]))[0] + 1
    local = local[mask[local]]
    threshold = max(2.0, 0.35 * float(np.nanmax(y[mask])))
    candidates = [i for i in local if y[i] >= threshold]
    selected: list[int] = []
    for i in sorted(candidates, key=lambda k: y[k], reverse=True):
        if all(abs(float(tm[i] - tm[j])) >= min_dist_s for j in selected):
            selected.append(i)
    bouts = []
    for i in sorted(selected):
        th = frac * y[i]
        a = b = i
        while a > idx[0] and y[a] > th:
            a -= 1
        while b < idx[-1] and y[b] > th:
            b += 1
        if float(tm[b] - tm[a]) >= min_dur_s:
            bouts.append((float(tm[a]), float(tm[b])))
    return bouts


def find_reps(t, s, min_dist_s: float = 3.0, prom_frac: float = 0.35):
    """Trough indices bounding each repetition in a cyclic signal."""
    thr = prom_frac * (np.percentile(s, 98) - np.percentile(s, 2))
    md = max(1, int(min_dist_s / np.median(np.diff(t))))
    peaks = []
    for i in range(1, len(s) - 1):
        if s[i] >= s[i - 1] and s[i] > s[i + 1] and s[i] > thr:
            if not peaks or (i - peaks[-1]) > md:
                peaks.append(i)
            elif s[i] > s[peaks[-1]]:
                peaks[-1] = i
    if not peaks:
        return []
    bounds = [0] + peaks + [len(s) - 1]
    return [a + int(np.argmin(s[a:b + 1])) for a, b in zip(bounds[:-1], bounds[1:])]


def filter_bouts(
    raw_bouts: Iterable[tuple[float, float]],
    protocol_window: Sequence[float],
    segment_overlap: Sequence[float],
) -> list[tuple[float, float]]:
    """Keep bouts fully inside both the protocol block and the synchronised overlap."""
    lo, hi = protocol_window
    start, end = segment_overlap
    return [
        (float(a), float(b))
        for a, b in raw_bouts
        if a >= lo and b <= hi and a >= start and b <= end
    ]


def win(bouts: Sequence[tuple[float, float]]) -> list[float] | None:
    """Overall [start, end] spanned by a bout list, rounded for reporting."""
    if not bouts:
        return None
    return [round(float(bouts[0][0]), 1), round(float(bouts[-1][1]), 1)]


# --------------------------------------------------------------------------
# Scoring
# --------------------------------------------------------------------------

def rom(sig, troughs):
    """Sign-agnostic per-rep peak-to-valley: linear-detrend each rep window, take max-min."""
    out = []
    for k in range(len(troughs) - 1):
        seg = sig[troughs[k]:troughs[k + 1] + 1]
        if len(seg) < 4:
            continue
        x = np.arange(len(seg))
        detr = seg - np.polyval(np.polyfit(x, seg, 1), x)
        out.append(float(detr.max() - detr.min()))
    return np.array(out)


def score_block(
    res: fiv.FiveImuResult,
    clock: dict[str, float],
    tm: np.ndarray,
    msig: np.ndarray,
    bouts: Sequence[tuple[float, float]],
    series_factory: Callable[[float], np.ndarray],
    abs_mocap: bool,
    lag_search: bool,
) -> dict[str, object]:
    """Score one protocol block: IMU-derived series against the mocap reference.

    ``clock`` is the fit ``t_imu = a * t_mocap + b``.  ``series_factory(lo)``
    returns the IMU series for a bout starting at mocap time ``lo`` -- pass a
    locally re-tared series (``local_twist`` / ``local_swing``) or a global one.

    Reports two accuracy families, and the distinction matters:

    * ``raw_*``      -- native sensor degrees against mocap degrees, no fitting.
    * ``heldout_*``  -- leave-one-bout-out, refitting gain on the other bouts.
      This is an honest estimate *given* a per-session calibration, not an
      out-of-the-box accuracy.

    ``accuracy_basis`` is ``native`` only when the raw error is small *and* the
    fitted gain is near unity.  A high held-out accuracy with a gain far from 1
    still depends on per-session gain correction; labelling that ``native``
    would overstate what the sensor does on its own.
    """
    a = float(clock["a"])
    b = float(clock["b"])
    ti = res.t_s
    xs: list[np.ndarray] = []
    ys: list[np.ndarray] = []
    per_rep_r: list[float] = []
    lags: list[float] = []

    for lo, hi in bouts:
        series = series_factory(lo)
        grid = np.arange(lo, hi, 0.01)
        ok = (a * grid + b >= ti[0]) & (a * grid + b <= ti[-1])
        grid = grid[ok]
        if len(grid) < 5:
            continue

        lag = 0.0
        if lag_search:
            best_corr = None
            for candidate in np.arange(-0.5, 0.5 + 1e-9, 0.01):
                m0 = np.interp(grid, tm, msig)
                i0 = np.interp(a * grid + b + candidate, ti, series)
                if abs_mocap:
                    m0 = np.abs(m0)
                if np.std(m0) < 1e-6 or np.std(i0) < 1e-6:
                    continue
                corr = float(np.corrcoef(i0, m0)[0, 1])
                if best_corr is None or abs(corr) > abs(best_corr):
                    best_corr = corr
                    lag = float(candidate)

        pre = np.arange(max(float(tm[0]), lo - 1.2), lo - 0.2, 0.01)
        mocap_zero = float(np.mean(np.interp(pre, tm, msig))) if len(pre) >= 5 else 0.0
        imu_zero = 0.0
        if len(pre) >= 5 and a * pre[0] + b + lag >= ti[0] and a * pre[-1] + b + lag <= ti[-1]:
            imu_zero = float(np.mean(np.interp(a * pre + b + lag, ti, series)))

        m = np.interp(grid, tm, msig) - mocap_zero
        i = np.interp(a * grid + b + lag, ti, series) - imu_zero
        if abs_mocap:
            m = np.abs(m)
        xs.append(i)
        ys.append(m)
        lags.append(lag)
        if np.std(i) >= 1e-6 and np.std(m) >= 1e-6:
            per_rep_r.append(float(np.corrcoef(i, m)[0, 1]))

    if not xs:
        return {"n_reps": 0}

    x = np.concatenate(xs)
    y = np.concatenate(ys)
    gain, intercept = np.polyfit(x, y, 1)
    pred = gain * x + intercept
    rom_deg = float(np.ptp(y))
    raw_rmse = float(np.sqrt(np.mean((y - x) ** 2)))
    out: dict[str, object] = {
        "n_reps": len(xs),
        "pooled_r": round(float(np.corrcoef(x, y)[0, 1]), 3),
        "gain": round(float(gain), 3),
        "over_read_x": round(1.0 / gain, 2) if abs(gain) > 1e-6 else None,
        "raw_rmse_deg": round(raw_rmse, 2),
        "raw_acc": round(1.0 - raw_rmse / max(rom_deg, 1e-6), 3),
        "calibrated_rmse_deg": round(float(np.sqrt(np.mean((y - pred) ** 2))), 2),
        "rom_deg": round(rom_deg, 1),
        "per_rep_r_median": round(float(np.median(per_rep_r)), 3) if per_rep_r else None,
        "per_rep_r_min": round(float(np.min(per_rep_r)), 3) if per_rep_r else None,
        "lag_median_s": round(float(np.median(lags)), 3),
    }
    native_scale = 0.7 <= float(gain) <= 1.4
    out["accuracy_basis"] = "native" if out["raw_acc"] >= 0.6 and native_scale else "gain_corrected_only"
    if len(xs) >= 2:
        sse = 0.0
        n = 0
        gains = []
        for k in range(len(xs)):
            train_x = np.concatenate([xs[i] for i in range(len(xs)) if i != k])
            train_y = np.concatenate([ys[i] for i in range(len(xs)) if i != k])
            fold_gain, fold_intercept = np.polyfit(train_x, train_y, 1)
            gains.append(float(fold_gain))
            sse += float(np.sum((ys[k] - (fold_gain * xs[k] + fold_intercept)) ** 2))
            n += len(xs[k])
        heldout = float(np.sqrt(sse / n))
        out["heldout_rmse_deg"] = round(heldout, 2)
        out["heldout_acc"] = round(1.0 - heldout / max(rom_deg, 1e-6), 3)
        out["gain_range"] = [round(float(np.min(gains)), 3), round(float(np.max(gains)), 3)]
    return out


def score_for_mode(
    res: fiv.FiveImuResult,
    tm: np.ndarray,
    sig: np.ndarray,
    bouts: Sequence[tuple[float, float]],
    mode: str,
    clock: dict[str, float],
    lag: bool = False,
) -> dict[str, object]:
    """Score a block under one of the three IMU readout modes.

    ``global_yawmasked_swing``       -- swing from the pipeline's global frame
    ``local_retare_yawmasked_swing`` -- swing re-tared per bout
    anything else                    -- locally re-tared axial twist
    """
    a = float(clock["a"])
    b = float(clock["b"])
    if mode == "global_yawmasked_swing":
        return score_block(res, clock, tm, sig, bouts, lambda lo, r=res: r.relations["sacrum_to_upper"].swing_deg, True, lag)
    if mode == "local_retare_yawmasked_swing":
        return score_block(res, clock, tm, sig, bouts, lambda lo, r=res, aa=a, bb=b: local_swing(r, aa, bb, lo), True, lag)
    return score_block(res, clock, tm, sig, bouts, lambda lo, r=res, aa=a, bb=b: local_twist(r, aa, bb, lo), False, lag)


def choose_bend(global_score: dict[str, object], local_score: dict[str, object]) -> str:
    """Pick the bend readout mode, requiring a margin before preferring the local re-tare.

    The 0.3-degree margin keeps the choice from flipping on noise; ties go to
    the global mode because it has one fewer per-bout fitting step.
    """
    g = float(global_score.get("heldout_rmse_deg", 999.0))
    l = float(local_score.get("heldout_rmse_deg", 999.0))
    return "local_retare_yawmasked_swing" if l + 0.3 < g else "global_yawmasked_swing"


def mark_bend_gain_corrected(score: dict[str, object]) -> dict[str, object]:
    """Force ``accuracy_basis`` to gain-corrected.

    Bend readouts go through a yaw-masked swing magnitude, so their scale is not
    directly comparable to a native degree reading even when the raw error looks
    small.  Marking them explicitly prevents the aggregate from claiming native
    accuracy it does not have.
    """
    score = dict(score)
    score["accuracy_basis"] = "gain_corrected_only"
    return score


def stat(imu, moc) -> dict[str, float]:
    """Mean, bias, and RMSE of an IMU series against its mocap counterpart."""
    return {
        "mean": round(float(np.mean(imu)), 1),
        "bias": round(float(np.mean(imu - moc)), 1),
        "rmse": round(float(np.sqrt(np.mean((imu - moc) ** 2))), 1),
    }


# --------------------------------------------------------------------------
# Reporting helpers
# --------------------------------------------------------------------------

def jsonable(value):
    """Recursively convert numpy scalars and tuples so ``json.dump`` accepts them."""
    if isinstance(value, dict):
        return {k: jsonable(v) for k, v in value.items()}
    if isinstance(value, list):
        return [jsonable(v) for v in value]
    if isinstance(value, tuple):
        return [jsonable(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    return value
