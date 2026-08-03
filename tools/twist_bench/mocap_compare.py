from __future__ import annotations

"""Validate IMU trunk estimates against MoCap (OptiTrack/Motive) ground truth.

  IMU log   -> VQF validation1 relations: per-relation twist / swing / total relative angle
  MoCap csv -> thorax-vs-pelvis axial (twist GT) + flexion + lateral
  align the two independent clocks by cross-correlating the TOTAL trunk angle (robust, sync-free),
  then report on the overlap:
    BENDING : IMU swing  vs MoCap |flexion,lateral|   (does the IMU capture bending?)
    TWIST   : IMU twist  vs MoCap axial               (does the IMU capture axial rotation?)
             split into twist-dominant frames and flexion frames to expose swing->twist cross-talk.

Usage: mocap_compare.py [imu_log] [mocap_csv]   (defaults to the 2026-06-26 pilot pair)
"""

import json
import sys
from pathlib import Path

import numpy as np

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE))
import five_imu_fusion as fiv
import mocap_adapter as mc

DT = 0.02  # common resample grid (50 Hz)


def _xcorr_offset(ti, xi, tm, xm, max_off: float) -> float:
    """Time offset (s) so that t_imu ~ t_mocap + offset, by cross-correlating two positive signals
    resampled to a common grid. Searched in [0, max_off]; normalized so partial overlaps don't win."""
    gi = np.arange(0, ti[-1], DT)
    a = np.interp(gi, ti, xi) - np.mean(np.interp(gi, ti, xi))
    gm = np.arange(0, tm[-1], DT)
    b = np.interp(gm, tm, xm) - np.mean(np.interp(gm, tm, xm))
    n = len(a) + len(b) - 1
    nfft = 1 << (n - 1).bit_length()
    cc = np.fft.irfft(np.fft.rfft(a, nfft) * np.conj(np.fft.rfft(b, nfft)), nfft)
    cc = np.concatenate([cc[-(len(b) - 1):], cc[:len(a)]])
    lags = np.arange(-(len(b) - 1), len(a))
    overlap = np.minimum.reduce([np.full_like(lags, len(b)), len(a) - lags, lags + len(b), np.full_like(lags, len(a))])
    cc = cc / np.maximum(overlap, 1)
    mask = (lags >= 0) & (lags <= int(max_off / DT))
    idx = np.where(mask)[0]
    return float(lags[idx[np.argmax(cc[idx])]] * DT)


def _r(x, y):
    return float(np.corrcoef(x, y)[0, 1]) if len(x) > 2 else float("nan")


def main() -> int:
    imu_log = Path(sys.argv[1]) if len(sys.argv) > 1 else BASE / "data" / "twist_trial_20260626_170028.log"
    mocap_csv = Path(sys.argv[2]) if len(sys.argv) > 2 else BASE / "data" / "mocap" / "Testing_01(1).csv"

    # --- IMU ---
    pm = json.loads((BASE / "data" / "validation1_layout.json").read_text(encoding="utf-8"))["placement_map"]
    res = fiv.run_pipeline(imu_log, fiv.make_args(layout_preset="validation1", filter="vqf",
                                                  **{role: imu for imu, role in pm.items()}))
    ti = res.t_s
    rel = {n: res.relations[n] for n in ("sacrum_to_t3", "sacrum_to_sternum")}
    total = lambda r: 2.0 * np.degrees(np.arccos(np.clip(np.abs(r.q_tared[:, 0]), -1.0, 1.0)))  # noqa: E731
    imu_total = total(rel["sacrum_to_t3"])
    print(f"IMU  {imu_log.name}: {len(ti)} frames, {ti[-1]:.0f}s")

    # --- MoCap ---
    tm, markers, _ = mc.parse_motive_csv(mocap_csv)
    tm = tm - tm[0]
    markers = {k: mc._fill_gaps(v) for k, v in markers.items()}
    ang = mc.trunk_angles(markers)
    still = tm <= 8.0
    uw = lambda x: np.degrees(np.unwrap(np.radians(x)))  # noqa: E731
    axial, flex, lateral = uw(ang["axial"]), uw(ang["flex"]), uw(ang["lateral"])
    axial -= np.median(axial[still]); flex -= np.median(flex[still]); lateral -= np.median(lateral[still])
    moc_total = np.sqrt(axial ** 2 + flex ** 2 + lateral ** 2)
    moc_bend = np.sqrt(flex ** 2 + lateral ** 2)
    print(f"MoCap {mocap_csv.name}: {len(tm)} frames, {tm[-1]:.0f}s")

    # --- align clocks by total trunk angle ---
    offset = _xcorr_offset(ti, imu_total, tm, moc_total, max_off=180.0)
    g = np.arange(max(ti[0], offset) + 1.0, min(ti[-1], tm[-1] + offset) - 1.0, DT)
    on_i = lambda y: np.interp(g, ti, y)          # noqa: E731  IMU series on common grid
    on_m = lambda y: np.interp(g, tm + offset, y)  # noqa: E731  MoCap series on common grid
    ax, fl, lt = on_m(axial), on_m(flex), on_m(lateral)
    print(f"\nsync: MoCap t=0 -> IMU t={offset:.1f}s   r(total trunk angle)={_r(on_i(imu_total), on_m(moc_total)):.2f}"
          f"   overlap {len(g)*DT:.0f}s")

    twist_dom = (np.abs(ax) > 8) & (np.abs(fl) < 15) & (np.abs(lt) < 15)   # upright twisting
    flex_dom = np.abs(fl) > 15                                             # bending (axial GT ~ 0)
    print(f"\nBENDING  (IMU swing vs MoCap flexion+lateral):")
    for name, r in rel.items():
        sw = on_i(r.swing_deg)
        print(f"  {name:18} r={_r(sw, on_m(moc_bend)):.3f}   swing ROM IMU {np.ptp(sw):.0f} / MoCap {np.ptp(moc_bend):.0f} deg")
    print(f"\nTWIST    (IMU twist vs MoCap axial rotation = ground truth):")
    for name, r in rel.items():
        tw = on_i(r.twist_deg)
        print(f"  {name}")
        print(f"    all frames          r={_r(tw, ax):6.3f}   IMU ±{np.ptp(tw)/2:>3.0f}  MoCap ±{np.ptp(ax)/2:>3.0f} deg")
        print(f"    twist-dominant {twist_dom.sum()*DT:>3.0f}s  r={_r(tw[twist_dom], ax[twist_dom]):6.3f}   "
              f"IMU ±{np.ptp(tw[twist_dom])/2:>3.0f}  MoCap ±{np.ptp(ax[twist_dom])/2:>3.0f} deg  (real twisting)")
        print(f"    during flexion {flex_dom.sum()*DT:>3.0f}s  IMU twist ±{np.ptp(tw[flex_dom])/2:>3.0f}  "
              f"while MoCap axial ±{np.ptp(ax[flex_dom])/2:>3.0f} deg  (cross-talk: flexion leaking into twist)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
