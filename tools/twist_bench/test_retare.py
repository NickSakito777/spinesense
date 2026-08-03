from __future__ import annotations
"""Scout the main re-tare hypotheses on twist-L (P02 x T02-2). Non-circular: re-tare windows
are IMU-gyro-quietest sub-windows; MoCap only scores."""
import sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import retare_harness as H
import twist_bench_fusion as pf
import validation3_cluster_orientation as v3
AXIS = pf.SEGMENT_TWIST_AXIS


def score_dir(d, series, bouts):
    s = v3.score(d["tm"], d["twist"], d["ti"], series, bouts, signed=True)
    lo = v3.loro(d["tm"], d["twist"], d["ti"], series, bouts, signed=True)
    if s is None:
        return "no data"
    r, sl, raw, cal, rom = s
    txt = f"r={r:+.2f} gain={sl:.2f}"
    if lo is not None:
        rmse, gains, romL = lo
        txt += f" | held-out {rmse:.1f}° ({max(0,1-rmse/max(romL,1e-6))*100:.0f}%) gain[{min(gains):.1f}-{max(gains):.1f}]"
    return txt


def decomp_local(d, q_rel, tare_idx, axis=AXIS):
    q0 = pf.quat_average(q_rel[tare_idx])
    q_t = pf.qmul(pf.qconj(q0)[None, :], q_rel)
    tw, _ = pf.swing_twist_deg(q_t, axis)
    return -pf.unwrap_deg(tw)


def perblock(d, q_rel, bouts, axis=AXIS):
    t0 = min(b[0] for b in bouts)
    idx = H.still_before(d, t0, look=8.0, win=1.2)
    return decomp_local(d, q_rel, idx, axis)


def perbout(d, q_rel, bouts, axis=AXIS):
    """Full series; within each bout's neighborhood, decompose against a bout-local still q0."""
    ti = d["ti"]
    master = decomp_local(d, q_rel, ti <= 8.0, axis)
    for (a, b) in bouts:
        idx = H.still_before(d, a, look=2.5, win=0.6)
        if idx.sum() < 3:
            continue
        loc = decomp_local(d, q_rel, idx, axis)
        i0 = np.searchsorted(ti, d["A"] * (a - 1.0) + d["B"])
        i1 = np.searchsorted(ti, d["A"] * (b + 0.6) + d["B"])
        master[i0:i1] = loc[i0:i1]
    return master


def functional_axis(d, q_rel):
    """Bend-only calib window from the bending block (IMU-detected, no MoCap angle): estimate axis."""
    # bending block = where MoCap bend is large but we only use its TIME to grab IMU samples
    bend = np.hypot(d["flex"], d["lat"])
    bw = [(d["tm"][i], d["tm"][j - 1]) for i, j in v3.runs(d["tm"], bend > 20)]
    if not bw:
        return AXIS
    ti = d["ti"]
    mask = np.zeros(len(ti), dtype=bool)
    for a, b in bw:
        mask |= (ti >= d["A"] * a + d["B"]) & (ti <= d["A"] * b + d["B"])
    q0 = pf.quat_average(q_rel[ti <= 8.0])
    q_t = pf.qmul(pf.qconj(q0)[None, :], q_rel)
    return pf.estimate_twist_axis(q_t, mask)


def main():
    d = H.load()
    q_ms = H.qrel(d, "mid", "sternum")
    print(f"covered bouts L={len(d['boutsL'])} R={len(d['boutsR'])}\n")

    print("S0 baseline global tare, mid->sternum, fixed axis")
    base = decomp_local(d, q_ms, d["ti"] <= 8.0)
    print("   L:", score_dir(d, base, d["boutsL"]))
    print("   R:", score_dir(d, base, d["boutsR"]))

    print("\nS1 per-BLOCK full re-tare (local still before each block)")
    print("   L:", score_dir(d, perblock(d, q_ms, d["boutsL"]), d["boutsL"]))
    print("   R:", score_dir(d, perblock(d, q_ms, d["boutsR"]), d["boutsR"]))

    print("\nS2 per-BOUT full re-tare (local still before each bout)")
    print("   L:", score_dir(d, perbout(d, q_ms, d["boutsL"]), d["boutsL"]))
    print("   R:", score_dir(d, perbout(d, q_ms, d["boutsR"]), d["boutsR"]))

    print("\nS3 functional twist axis (bend-calib), global tare")
    ax = functional_axis(d, q_ms)
    print(f"   axis={np.round(ax,3)}")
    fa = decomp_local(d, q_ms, d["ti"] <= 8.0, ax)
    print("   L:", score_dir(d, fa, d["boutsL"]))
    print("   R:", score_dir(d, fa, d["boutsR"]))

    print("\nS4 per-BOUT re-tare + functional axis")
    print("   L:", score_dir(d, perbout(d, q_ms, d["boutsL"], ax), d["boutsL"]))
    print("   R:", score_dir(d, perbout(d, q_ms, d["boutsR"], ax), d["boutsR"]))

    print("\nS5 alternate relations (global tare, fixed axis), twist-L:")
    for rel in (("sacrum", "sternum"), ("upper", "sternum"), ("lower", "sternum"),
                ("mid", "upper"), ("sacrum", "upper")):
        q = H.qrel(d, *rel)
        s = decomp_local(d, q, d["ti"] <= 8.0)
        print(f"   {rel[0]:>6}->{rel[1]:<7} L:", score_dir(d, s, d["boutsL"]))


if __name__ == "__main__":
    main()
