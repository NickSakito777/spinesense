from __future__ import annotations

"""Precise MoCap->IMU clock map for Validation1 (n=1, 2026-06-26).

Fit  t_imu = scale * t_mocap + offset  by maximizing correlation between:
  IMU total relative trunk angle  : sacrum_to_t3 total angle = 2*deg*arccos(|q_tared w|)
  MoCap total trunk angle         : sqrt(axial^2 + flex^2 + lateral^2), unwrapped + tared

Step 1: coarse + fine search for the best CONSTANT offset (scale=1), 0..180 s.
Step 2: refine by also fitting a small scale (clock drift), report as ppm.
Report r constant vs r after drift, and whether drift is significant.
"""

import json
import sys
from pathlib import Path

import numpy as np

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE))
import five_imu_fusion as fiv
import mocap_adapter as mc

DT = 0.02  # 50 Hz common grid


def _r(x, y) -> float:
    if len(x) < 3:
        return float("nan")
    return float(np.corrcoef(x, y)[0, 1])


def corr_for_map(ti, xi, tm, xm, scale: float, offset: float, margin: float = 1.0) -> tuple[float, float]:
    """Correlation of IMU vs MoCap under t_imu = scale*t_mocap + offset.
    MoCap clock is mapped into IMU time, both resampled to a common grid. Returns (r, overlap_s)."""
    tm_in_imu = scale * tm + offset
    lo = max(ti[0], tm_in_imu[0]) + margin
    hi = min(ti[-1], tm_in_imu[-1]) - margin
    if hi - lo < 5.0:
        return float("nan"), 0.0
    g = np.arange(lo, hi, DT)
    a = np.interp(g, ti, xi)
    b = np.interp(g, tm_in_imu, xm)
    return _r(a, b), float(len(g) * DT)


def main() -> int:
    imu_log = Path(sys.argv[1]) if len(sys.argv) > 1 else BASE / "data" / "twist_trial_20260626_170028.log"
    mocap_csv = Path(sys.argv[2]) if len(sys.argv) > 2 else BASE / "data" / "mocap" / "Testing_01(1).csv"

    # ---------------- IMU total relative trunk angle (sacrum_to_t3) ----------------
    pm = json.loads((BASE / "data" / "validation1_layout.json").read_text(encoding="utf-8"))["placement_map"]
    res = fiv.run_pipeline(
        imu_log,
        fiv.make_args(layout_preset="validation1", filter="vqf", **{role: imu for imu, role in pm.items()}),
    )
    ti = res.t_s
    r_t3 = res.relations["sacrum_to_t3"]
    imu_total = 2.0 * np.degrees(np.arccos(np.clip(np.abs(r_t3.q_tared[:, 0]), -1.0, 1.0)))
    print(f"IMU   {imu_log.name}: {len(ti)} frames, {ti[-1]:.1f}s, "
          f"median {1.0/np.median(np.diff(ti)):.2f} Hz")

    # ---------------- MoCap total trunk angle ----------------
    tm, markers, _ = mc.parse_motive_csv(mocap_csv)
    tm = tm - tm[0]
    markers = {k: mc._fill_gaps(v) for k, v in markers.items()}
    ang = mc.trunk_angles(markers)
    still = tm <= 8.0
    uw = lambda x: np.degrees(np.unwrap(np.radians(x)))  # noqa: E731
    axial, flex, lateral = uw(ang["axial"]), uw(ang["flex"]), uw(ang["lateral"])
    axial -= np.median(axial[still]); flex -= np.median(flex[still]); lateral -= np.median(lateral[still])
    moc_total = np.sqrt(axial ** 2 + flex ** 2 + lateral ** 2)
    print(f"MoCap {mocap_csv.name}: {len(tm)} frames, {tm[-1]:.1f}s, "
          f"median {1.0/np.median(np.diff(tm)):.2f} Hz")

    # =================================================================
    # STEP 1: best CONSTANT offset (scale = 1), 0..180 s.
    # Coarse 0.1 s grid, then parabolic-ish fine refine at 0.005 s.
    # =================================================================
    coarse = np.arange(0.0, 180.0 + 1e-9, 0.1)
    r_coarse = np.array([corr_for_map(ti, imu_total, tm, moc_total, 1.0, off)[0] for off in coarse])
    best_i = int(np.nanargmax(r_coarse))
    off0 = float(coarse[best_i])
    fine = np.arange(off0 - 0.2, off0 + 0.2 + 1e-9, 0.005)
    r_fine = np.array([corr_for_map(ti, imu_total, tm, moc_total, 1.0, off)[0] for off in fine])
    bf = int(np.nanargmax(r_fine))
    offset_const = float(fine[bf])
    r_const, overlap_const = corr_for_map(ti, imu_total, tm, moc_total, 1.0, offset_const)
    print(f"\n[STEP 1] constant offset: MoCap t=0 -> IMU t = {offset_const:.3f} s")
    print(f"         r(total trunk angle) = {r_const:.4f}   overlap {overlap_const:.0f}s")

    # =================================================================
    # STEP 2: jointly fit scale + offset (clock drift).
    # Scale near 1; ppm = (scale-1)*1e6. Search scale on a fine grid, and
    # for each scale re-optimize the offset locally, maximizing r.
    # =================================================================
    # Anchor scale rotation about the middle of the overlapping MoCap span so offset and scale
    # stay nearly decoupled (offset ~ stable, scale absorbs drift).
    tm_mid = 0.5 * (tm[0] + tm[-1])

    def corr_scaled(scale: float, offset_about_mid: float) -> tuple[float, float]:
        # t_imu = scale*(tm - tm_mid) + tm_mid + offset_about_mid  (offset measured at tm_mid)
        tm_in_imu = scale * (tm - tm_mid) + tm_mid + offset_about_mid
        lo = max(ti[0], tm_in_imu[0]) + 1.0
        hi = min(ti[-1], tm_in_imu[-1]) - 1.0
        if hi - lo < 5.0:
            return float("nan"), 0.0
        g = np.arange(lo, hi, DT)
        a = np.interp(g, ti, imu_total)
        b = np.interp(g, tm_in_imu, moc_total)
        return _r(a, b), float(len(g) * DT)

    # offset_about_mid that reproduces the step-1 constant map at scale=1:
    off_mid_const = offset_const  # at scale=1, identity shift, offset about mid == plain offset

    scales = 1.0 + np.arange(-3000.0, 3000.0 + 1e-9, 25.0) * 1e-6  # +/-3000 ppm, 25 ppm step
    best = (-2.0, 1.0, off_mid_const, 0.0)  # r, scale, off_mid, overlap
    for sc in scales:
        # local offset search around the constant solution (drift shifts effective offset slightly)
        for om in np.arange(off_mid_const - 1.0, off_mid_const + 1.0 + 1e-9, 0.01):
            r, ov = corr_scaled(sc, om)
            if np.isfinite(r) and r > best[0]:
                best = (r, sc, om, ov)
    r_drift, scale_best, off_mid_best, overlap_drift = best

    # Fine-refine scale around the winner.
    for sc in np.arange(scale_best - 50e-6, scale_best + 50e-6 + 1e-12, 2e-6):
        for om in np.arange(off_mid_best - 0.1, off_mid_best + 0.1 + 1e-9, 0.005):
            r, ov = corr_scaled(sc, om)
            if np.isfinite(r) and r > r_drift:
                r_drift, scale_best, off_mid_best, overlap_drift = r, sc, om, ov

    ppm = (scale_best - 1.0) * 1e6
    # Convert the about-mid offset back to the plain  t_imu = scale*t_mocap + offset_plain  form:
    #   t_imu = scale*(tm - tm_mid) + tm_mid + off_mid = scale*tm + (tm_mid*(1-scale) + off_mid)
    offset_plain = tm_mid * (1.0 - scale_best) + off_mid_best

    print(f"\n[STEP 2] drift-corrected map: t_imu = {scale_best:.9f} * t_mocap + {offset_plain:.3f} s")
    print(f"         scale = {ppm:+.1f} ppm   (offset at MoCap-mid t={tm_mid:.1f}s is {off_mid_best:.3f}s)")
    print(f"         r(total trunk angle) = {r_drift:.4f}   overlap {overlap_drift:.0f}s")

    # =================================================================
    # Significance verdict.
    # =================================================================
    dr = r_drift - r_const
    drift_sig = (dr > 0.03) or (abs(ppm) > 500.0)
    print(f"\n[VERDICT] delta_r = {dr:+.4f}   |ppm| = {abs(ppm):.1f}")
    print(f"          drift significant: {drift_sig}  "
          f"(criterion: delta_r>0.03 OR |ppm|>500)")

    # Sanity: total span drift in seconds across the ~300s overlap.
    span = tm[-1] - tm[0]
    drift_s_over_span = (scale_best - 1.0) * span
    print(f"          implied total clock drift over {span:.0f}s MoCap span: {drift_s_over_span*1000:.1f} ms")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
