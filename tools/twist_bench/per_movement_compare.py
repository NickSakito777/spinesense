from __future__ import annotations

"""Per-movement IMU-vs-MoCap validation (Validation1 pilot, n=1, 2026-06-26).

Uses the drift-corrected clock map t_imu = scale*t_mocap + offset to frame-align IMU and MoCap,
then on each *movement window* (MoCap seconds, mapped into IMU time) resamples both to a 100 Hz
common grid and reports correlation, IMU ROM, MoCap ROM, best-fit scale (MoCap per IMU).

Movements (MoCap seconds, from segmentation):
  TWIST   : upright fixed-pelvis axial twist  -> IMU twist (sacrum_to_t3, sacrum_to_sternum) vs MoCap AXIAL
  FLEXION : pure flexion/extension            -> IMU swing vs MoCap |flex,lateral|; + IMU twist ROM vs MoCap axial~0 (cross-talk)
  LATERAL : lateral bend L/R                  -> IMU swing vs MoCap bending
"""

import json
import sys
from pathlib import Path

import numpy as np

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE))
import five_imu_fusion as fiv
import mocap_adapter as mc

GRID_HZ = 100.0
DT = 1.0 / GRID_HZ

# Drift-corrected clock map (amplitude-robust sliding-window regression form from alignment step):
#   t_imu = SCALE * t_mocap + OFFSET
SCALE = 1.002402
OFFSET = 61.889

# Movement windows in MoCap seconds. Chosen from the ATOMIC single-axis segments so each window is
# dominated by one plane (clean ground truth), avoiding the multi-axis merged blocks.
MOVEMENTS = {
    # cleanest upright axial twist epoch (rep1, coupled flex/lat <=12 deg; the 145-189s block is
    # excluded because segmentation flags it as carrying lateral cross-talk that contaminates twist GT)
    "twist": [(65.78, 88.05)],
    "twist_all": [(65.78, 88.05), (145.36, 189.40)],
    # pure flexion/extension (clean PURE FLEXION block B + rep1 flexion); axial GT ~ 0 here
    "flexion": [(16.21, 32.15), (110.32, 131.03)],
    # pure lateral bend L/R
    "lateral": [(42.01, 59.33), (135.41, 143.72)],
}


def _r(x, y):
    if len(x) < 3 or np.std(x) < 1e-9 or np.std(y) < 1e-9:
        return float("nan")
    return float(np.corrcoef(x, y)[0, 1])


def _best_scale(imu, moc):
    """Least-squares scale s minimizing |moc - s*imu| (MoCap deg per IMU deg), zero-mean signals."""
    xi = imu - np.mean(imu)
    xm = moc - np.mean(moc)
    denom = float(np.dot(xi, xi))
    return float(np.dot(xi, xm) / denom) if denom > 1e-9 else float("nan")


def main() -> int:
    imu_log = BASE / "data" / "twist_trial_20260626_170028.log"
    mocap_csv = BASE / "data" / "mocap" / "Testing_01(1).csv"

    # --- IMU pipeline (VQF, validation1 layout, authoritative placement_map) ---
    pm = json.loads((BASE / "data" / "validation1_layout.json").read_text(encoding="utf-8"))["placement_map"]
    res = fiv.run_pipeline(
        imu_log,
        fiv.make_args(layout_preset="validation1", filter="vqf",
                      **{role: imu for imu, role in pm.items()}),
    )
    ti = res.t_s
    rel = {n: res.relations[n] for n in ("sacrum_to_t3", "sacrum_to_sternum")}
    print(f"IMU  {imu_log.name}: {len(ti)} frames, {ti[-1]:.0f}s, layout {res.summary['layout']}")

    # --- MoCap GT angles (unwrapped, tared on 0-8s neutral) ---
    tm, markers, _ = mc.parse_motive_csv(mocap_csv)
    tm = tm - tm[0]
    markers = {k: mc._fill_gaps(v) for k, v in markers.items()}
    ang = mc.trunk_angles(markers)
    still = tm <= 8.0
    uw = lambda x: np.degrees(np.unwrap(np.radians(x)))  # noqa: E731
    axial, flex, lateral = uw(ang["axial"]), uw(ang["flex"]), uw(ang["lateral"])
    axial -= np.median(axial[still]); flex -= np.median(flex[still]); lateral -= np.median(lateral[still])
    moc_bend = np.sqrt(flex ** 2 + lateral ** 2)
    print(f"MoCap {mocap_csv.name}: {len(tm)} frames, {tm[-1]:.0f}s")
    print(f"clock map: t_imu = {SCALE:.6f}*t_mocap + {OFFSET:.3f}\n")

    rows = []

    def resample_window(win_list, imu_y, moc_y):
        """Concatenate per-window resamples of IMU and MoCap onto a 100 Hz grid in MoCap time
        (mapped to IMU time for the IMU series). Returns (imu_grid, moc_grid)."""
        imu_chunks, moc_chunks = [], []
        for (m0, m1) in win_list:
            gm = np.arange(m0, m1, DT)                 # MoCap-time grid
            gi = SCALE * gm + OFFSET                    # mapped IMU-time grid
            if gi[0] < ti[0] or gi[-1] > ti[-1]:
                gm = gm[(SCALE * gm + OFFSET >= ti[0]) & (SCALE * gm + OFFSET <= ti[-1])]
                gi = SCALE * gm + OFFSET
            imu_chunks.append(np.interp(gi, ti, imu_y))
            moc_chunks.append(np.interp(gm, tm, moc_y))
        return np.concatenate(imu_chunks), np.concatenate(moc_chunks)

    # ===================== TWIST =====================
    print("=" * 78)
    print("TWIST movement (upright fixed-pelvis axial twist)  -- IMU twist vs MoCap AXIAL")
    for name, r in rel.items():
        imu_tw, moc_ax = resample_window(MOVEMENTS["twist"], r.twist_deg, axial)
        corr = _r(imu_tw, moc_ax)
        imu_rom = float(np.ptp(imu_tw)); moc_rom = float(np.ptp(moc_ax))
        scale = _best_scale(imu_tw, moc_ax)
        rows.append({"movement": "twist", "imu_relation": name, "metric": "twist_vs_axial",
                     "imu_rom_deg": round(imu_rom, 1), "mocap_rom_deg": round(moc_rom, 1),
                     "correlation": round(corr, 3), "scale_mocap_per_imu": round(scale, 3)})
        print(f"  {name:18} r={corr:+.3f}  IMU ROM {imu_rom:5.1f}  MoCap ROM {moc_rom:5.1f}  "
              f"scale(MoCap/IMU)={scale:.2f}")

    # ===================== FLEXION =====================
    print("=" * 78)
    print("FLEXION movement (pure flexion/extension)")
    # (a) bending agreement: IMU swing vs MoCap |flex,lateral|
    for name, r in rel.items():
        imu_sw, moc_bd = resample_window(MOVEMENTS["flexion"], r.swing_deg, moc_bend)
        corr = _r(imu_sw, moc_bd)
        imu_rom = float(np.ptp(imu_sw)); moc_rom = float(np.ptp(moc_bd))
        scale = _best_scale(imu_sw, moc_bd)
        rows.append({"movement": "flexion", "imu_relation": name, "metric": "swing_vs_bending",
                     "imu_rom_deg": round(imu_rom, 1), "mocap_rom_deg": round(moc_rom, 1),
                     "correlation": round(corr, 3), "scale_mocap_per_imu": round(scale, 3)})
        print(f"  swing  {name:18} r={corr:+.3f}  IMU ROM {imu_rom:5.1f}  MoCap bend ROM {moc_rom:5.1f}  "
              f"scale(MoCap/IMU)={scale:.2f}")
    # (b) cross-talk: IMU twist ROM while MoCap axial ~ 0
    for name, r in rel.items():
        imu_tw, moc_ax = resample_window(MOVEMENTS["flexion"], r.twist_deg, axial)
        corr = _r(imu_tw, moc_ax)
        imu_rom = float(np.ptp(imu_tw)); moc_rom = float(np.ptp(moc_ax))
        scale = _best_scale(imu_tw, moc_ax)
        rows.append({"movement": "flexion", "imu_relation": name, "metric": "twist_vs_axial",
                     "imu_rom_deg": round(imu_rom, 1), "mocap_rom_deg": round(moc_rom, 1),
                     "correlation": round(corr, 3), "scale_mocap_per_imu": round(scale, 3)})
        print(f"  XTALK twist {name:14} IMU twist ROM {imu_rom:5.1f}  while MoCap axial ROM {moc_rom:4.1f}  "
              f"(spurious twist during pure flexion)")

    # ===================== LATERAL =====================
    print("=" * 78)
    print("LATERAL movement (lateral bend L/R)  -- IMU swing vs MoCap bending")
    for name, r in rel.items():
        imu_sw, moc_bd = resample_window(MOVEMENTS["lateral"], r.swing_deg, moc_bend)
        corr = _r(imu_sw, moc_bd)
        imu_rom = float(np.ptp(imu_sw)); moc_rom = float(np.ptp(moc_bd))
        scale = _best_scale(imu_sw, moc_bd)
        rows.append({"movement": "lateral", "imu_relation": name, "metric": "swing_vs_bending",
                     "imu_rom_deg": round(imu_rom, 1), "mocap_rom_deg": round(moc_rom, 1),
                     "correlation": round(corr, 3), "scale_mocap_per_imu": round(scale, 3)})
        print(f"  swing  {name:18} r={corr:+.3f}  IMU ROM {imu_rom:5.1f}  MoCap bend ROM {moc_rom:5.1f}  "
              f"scale(MoCap/IMU)={scale:.2f}")

    print("=" * 78)
    print(json.dumps(rows, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
