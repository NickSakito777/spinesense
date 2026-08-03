from __future__ import annotations

"""Validation1 accuracy + significance (P01 MoCap pilot), with the functional twist-axis fix.

Extends accuracy_summary.py: per-movement, frame-aligned (drift-corrected), per-channel linear
gain+offset calibration, residual RMSE/MAE/accuracy% of IMU vs MoCap. Adds:
  - TWIST reported BOTH 'fixed' [0,0,1] axis and 'functional' (bend-decoupled) axis, so the
    flexion->twist cross-talk fix is visible side by side.
  - Honest P-values: 120 Hz frames are autocorrelated, so a raw Pearson p over thousands of frames
    is meaningless. We deflate to an effective sample size (Bartlett / Pyper-Peterman multi-lag) and
    test r via Fisher-z (normal CDF, no scipy). N=1 subject => p shows within-trial non-randomness,
    NOT population generalisation. Neff ~ number of movement reps.

Alignment map + clean single-axis windows are pilot-specific constants (re-derive for new recordings).

Run (from tools/twist_bench):
    python validation1_accuracy_stats.py
or:
    python validation1_accuracy_stats.py --imu <log> --mocap <csv> --layout <json> --filter madgwick
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import five_imu_fusion as fiv
import mocap_adapter as mc

# pilot-specific drift-corrected time map: t_imu = A * t_mocap + B  (IMU started ~62 s before MoCap)
A, B = 1.002402, 61.889
# clean single-axis MoCap windows (seconds)
WIN = {
    "twist (upright)": [(65.78, 88.05)],
    "flexion":         [(16.21, 32.15), (110.32, 131.01)],
    "lateral":         [(42.01, 59.33), (135.41, 143.72)],
}
# functional twist-axis calibration: flexion bout1 + lateral bout1 (pure bend, NOT the twist window)
CALIB_MOC = [(16.21, 32.15), (42.01, 59.33)]


def _ndcdf(z: float) -> float:
    return 0.5 * (1.0 + math.erf(z / math.sqrt(2.0)))


def _neff_multilag(x: np.ndarray, y: np.ndarray) -> float:
    """Effective N for correlation of two autocorrelated series (Bartlett / Pyper-Peterman):
    N_eff = N / (1 + 2*sum_k (1-k/N) rho_x(k) rho_y(k)). ~ number of independent chunks (reps)."""
    n = len(x)
    k = min(n // 4, 600)
    if k < 1:
        return float(n)

    def acf(v: np.ndarray) -> np.ndarray:
        v = v - v.mean()
        d = float(np.dot(v, v))
        return np.array([np.dot(v[:-i], v[i:]) / d for i in range(1, k + 1)]) if d > 0 else np.zeros(k)

    s = float(np.sum((1 - np.arange(1, k + 1) / n) * acf(x) * acf(y)))
    return max(3.0, min(float(n), n / (1 + 2 * s)))


def corr_stats(I: np.ndarray, M: np.ndarray) -> tuple[float, int, float, float, float, float]:
    """Pearson r, N, effective N, Fisher-z two-sided p, 95% CI on r."""
    n = len(I)
    r = float(np.corrcoef(I, M)[0, 1])
    n_eff = _neff_multilag(I, M)
    rc = min(0.999999, max(-0.999999, r))
    se = 1.0 / math.sqrt(n_eff - 3.0)
    p = 2.0 * (1.0 - _ndcdf(abs(math.atanh(rc) / se)))
    lo = math.tanh(math.atanh(rc) - 1.96 * se)
    hi = math.tanh(math.atanh(rc) + 1.96 * se)
    return r, n, n_eff, p, lo, hi


def _grab(wins, ti, iy, tm, my):
    I, M = [], []
    for a, b in wins:
        g = np.arange(a, b, 0.01)
        I.append(np.interp(A * g + B, ti, iy))
        M.append(np.interp(g, tm, my))
    return np.concatenate(I), np.concatenate(M)


def _line(tag, wins, ti, iy, tm, my):
    I, M = _grab(wins, ti, iy, tm, my)
    a, b = np.polyfit(I, M, 1)
    rmse_c = float(np.sqrt(np.mean((M - (a * I + b)) ** 2)))
    mae_c = float(np.mean(np.abs(M - (a * I + b))))
    bias_raw = float(np.mean(I - M))
    acc = max(0.0, 1.0 - rmse_c / max(np.ptp(M), 1e-6)) * 100
    r, n, ne, p, lo, hi = corr_stats(I, M)
    ps = "<1e-12" if p < 1e-12 else f"{p:.1e}"
    print(f"{tag:26} r={r:5.2f} [{lo:5.2f},{hi:4.2f}]  p={ps:>7}  N={n:4d} Neff={ne:5.1f}  "
          f"calRMSE={rmse_c:4.1f}  MAE={mae_c:4.1f}  acc={acc:3.0f}%  gain={a:5.2f}  bias={bias_raw:+5.1f}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--imu", type=Path, default=HERE / "data/twist_trial_20260626_170028.log")
    ap.add_argument("--mocap", type=Path, default=HERE / "data/mocap/Testing_01(1).csv")
    ap.add_argument("--layout", type=Path, default=HERE / "data/validation1_layout.json")
    ap.add_argument("--filter", default="vqf", help="vqf (preferred) or madgwick (numpy-only fallback)")
    args = ap.parse_args()

    flt = args.filter
    if flt == "vqf":
        try:
            import vqf  # noqa: F401
        except Exception:
            flt = "madgwick"
            print("[warn] vqf unavailable -> filter=madgwick (cross-talk mechanism is filter-independent)")

    pm = json.loads(args.layout.read_text(encoding="utf-8"))["placement_map"]
    common = dict(layout_preset="validation1", filter=flt, **{role: imu for imu, role in pm.items()})
    calib = [[A * a + B, A * b + B] for a, b in CALIB_MOC]
    fix = fiv.run_pipeline(args.imu, fiv.make_args(**common))
    fn = fiv.run_pipeline(args.imu, fiv.make_args(twist_axis_mode="functional",
                                                  twist_calib_window=calib, **common))
    ti = fix.t_s
    tw_fix = {n: fix.relations[n].twist_deg for n in ("sacrum_to_t3", "sacrum_to_sternum")}
    tw_fn = {n: fn.relations[n].twist_deg for n in ("sacrum_to_t3", "sacrum_to_sternum")}
    sw_t3 = fix.relations["sacrum_to_t3"].swing_deg

    tm, mk, _ = mc.parse_motive_csv(args.mocap)
    tm = tm - tm[0]
    mk = {k: mc._fill_gaps(v) for k, v in mk.items()}
    ang = mc.trunk_angles(mk)
    st = tm <= 8
    uw = lambda x: np.degrees(np.unwrap(np.radians(x)))  # noqa: E731
    ax = uw(ang["axial"]); fl = uw(ang["flex"]); lt = uw(ang["lateral"])
    ax -= np.median(ax[st]); fl -= np.median(fl[st]); lt -= np.median(lt[st])
    bend = np.sqrt(fl ** 2 + lt ** 2)

    print(f"P01 pilot (N=1, 3 reps/movement, MoCap truth, filter={flt}). "
          f"p = Fisher-z, autocorr-corrected (Neff ~ reps).\n")
    print("=== TWIST  (IMU twist vs MoCap axial) ===")
    _line("twist sternum  FIXED", WIN["twist (upright)"], ti, tw_fix["sacrum_to_sternum"], tm, ax)
    _line("twist sternum  FUNC", WIN["twist (upright)"], ti, tw_fn["sacrum_to_sternum"], tm, ax)
    _line("twist t3       FIXED", WIN["twist (upright)"], ti, tw_fix["sacrum_to_t3"], tm, ax)
    _line("twist t3       FUNC", WIN["twist (upright)"], ti, tw_fn["sacrum_to_t3"], tm, ax)
    print("\n=== BENDING  (IMU swing vs MoCap flexion+lateral; twist-axis-independent) ===")
    _line("lateral  swing t3", WIN["lateral"], ti, sw_t3, tm, bend)
    _line("flexion  swing t3", WIN["flexion"], ti, sw_t3, tm, bend)
    print("\n=== CROSS-TALK  (IMU twist during pure flexion; true axial ~0, lower=better) ===")
    for tag, twd in (("FIXED", tw_fix["sacrum_to_sternum"]), ("FUNC", tw_fn["sacrum_to_sternum"])):
        I, M = _grab(WIN["flexion"], ti, twd, tm, ax)
        print(f"  sternum {tag:6}: spurious twist RMS={np.sqrt(np.mean(I**2)):4.1f}  peak={np.max(np.abs(I)):4.1f}  "
              f"(true axial RMS={np.sqrt(np.mean(M**2)):.1f})")
    print("\nacc% = 1 - calRMSE/ROM. gain<1 = IMU over-reads, >1 = under-reads. "
          "bias = raw IMU-MoCap mean before gain calib.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
