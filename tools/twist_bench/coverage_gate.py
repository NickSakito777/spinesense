from __future__ import annotations

"""Outcome-independent local timestamp coverage gate for MoCap-defined bouts."""

from typing import Any


def assess_bout_coverage(
    ti: Any,
    a: float,
    b: float,
    bouts: list[tuple[float, float]],
    np: Any,
) -> tuple[list[tuple[float, float]], list[dict[str, Any]]]:
    """Return bouts with >=95% local timestamp support and no long local gap.

    Support is measured on a 10 ms grid in IMU time.  A grid point is supported
    when its nearest fused IMU timestamp is within max(50 ms, 5*median dt).
    This uses timestamps only; signal values, labels, MoCap agreement and mapping
    outcomes cannot influence acceptance.
    """

    positive_dt = np.diff(ti)
    positive_dt = positive_dt[(positive_dt > 0.0) & np.isfinite(positive_dt)]
    median_dt = float(np.median(positive_dt)) if len(positive_dt) else 0.01
    tolerance = max(0.05, 5.0 * median_dt)
    accepted: list[tuple[float, float]] = []
    diagnostics: list[dict[str, Any]] = []
    for lo, hi in bouts:
        imu_lo, imu_hi = float(a * lo + b), float(a * hi + b)
        mask = (ti >= imu_lo) & (ti <= imu_hi)
        samples = ti[mask]
        grid = np.arange(imu_lo, imu_hi, 0.01)
        if len(grid):
            right = np.searchsorted(ti, grid, side="left")
            right = np.clip(right, 0, len(ti) - 1)
            left = np.clip(right - 1, 0, len(ti) - 1)
            nearest = np.minimum(np.abs(ti[right] - grid), np.abs(ti[left] - grid))
            support_fraction = float(np.mean(nearest <= tolerance))
        else:
            support_fraction = 0.0
        if len(samples):
            gaps = np.diff(samples)
            edge_gaps = np.asarray(
                [max(0.0, float(samples[0] - imu_lo)), max(0.0, float(imu_hi - samples[-1]))]
            )
            max_gap = float(max(np.max(gaps) if len(gaps) else 0.0, np.max(edge_gaps)))
        else:
            max_gap = float(max(0.0, imu_hi - imu_lo))
        n_samples = int(len(samples))
        max_allowed_gap = 2.0 * tolerance
        valid = (
            n_samples >= 5
            and support_fraction >= 0.95
            and max_gap <= max_allowed_gap
        )
        reasons: list[str] = []
        if n_samples < 5:
            reasons.append("n_samples_lt_5")
        if support_fraction < 0.95:
            reasons.append("support_fraction_lt_0p95")
        if max_gap > max_allowed_gap:
            reasons.append("max_gap_gt_2x_nearest_tolerance")
        diagnostics.append(
            {
                "bout": [float(lo), float(hi)],
                "n_samples": n_samples,
                "support_fraction": round(support_fraction, 6),
                "max_gap_s": round(max_gap, 6),
                "nearest_sample_tolerance_s": round(tolerance, 6),
                "max_allowed_gap_s": round(max_allowed_gap, 6),
                "accepted": valid,
                "exclusion_reasons": reasons,
            }
        )
        if valid:
            accepted.append((float(lo), float(hi)))
    return accepted, diagnostics


def assess_scoring_coverage(
    ti: Any,
    a: float,
    b: float,
    bouts: list[tuple[float, float]],
    np: Any,
) -> tuple[list[tuple[float, float]], list[dict[str, Any]]]:
    """Require support for both each movement and its fixed [-1.2,-0.2]s neutral."""

    _, movement = assess_bout_coverage(ti, a, b, bouts, np)
    neutral_windows = [(float(lo - 1.2), float(lo - 0.2)) for lo, _ in bouts]
    _, neutral = assess_bout_coverage(ti, a, b, neutral_windows, np)
    accepted: list[tuple[float, float]] = []
    diagnostics: list[dict[str, Any]] = []
    for bout, movement_qc, neutral_qc in zip(bouts, movement, neutral):
        valid = bool(movement_qc["accepted"] and neutral_qc["accepted"])
        reasons = [f"movement:{item}" for item in movement_qc["exclusion_reasons"]]
        reasons.extend(f"neutral:{item}" for item in neutral_qc["exclusion_reasons"])
        diagnostics.append(
            {
                "bout": [float(bout[0]), float(bout[1])],
                "movement": movement_qc,
                "neutral": neutral_qc,
                "accepted": valid,
                "exclusion_reasons": reasons,
            }
        )
        if valid:
            accepted.append((float(bout[0]), float(bout[1])))
    return accepted, diagnostics
