from __future__ import annotations

"""Validation1 (Take1) recompute with MOVEMENT-DEPENDENT reference (parent) segment.

Fox's idea (2026-06-30): use S1/sacrum as the baseline only for FLEXION (whole trunk over pelvis),
and use T12 as the upper-back baseline for EXTENSION / LATERAL-L/R / TWIST-L/R (isolate the thoracic
motion, drop lumbar+pelvis contribution). Recompute per-movement gain/r/RMSE/held-out vs the old
all-sacrum reference, on the clean Take1 MoCap (thorax-vs-pelvis truth, unchanged).

  flexion   : MoCap flex  vs IMU sacrum_to_t3.swing   (parent = S1, unchanged)
  extension : MoCap flex  vs IMU t12_to_t3.swing      (parent = T12  <- NEW)
  lat-L/R   : MoCap lat   vs IMU t12_to_t3.swing      (parent = T12  <- NEW)
  twist-L/R : MoCap axial vs IMU t12_to_sternum.twist (parent = T12  <- NEW)
OLD baseline for each: sacrum_to_t3 (bend) / sacrum_to_sternum (twist). Run with .venv python.
"""

import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import five_imu_fusion as fiv
import mocap_adapter as mc

A, B = 1.002402, 61.889
RAW = HERE / "data/twist_trial_20260626_170028.log"
MOC = HERE / "data/mocap/Testing_01(1).csv"
PM = {"IMU0": "sacrum", "IMU1": "t12", "IMU2": "t6", "IMU3": "t3", "IMU4": "sternum"}

# additive: register the two T12-parent relations Fox's idea needs (runtime, no file edit)
for rel in (("t12_to_t3", "t12", "t3"), ("t12_to_sternum", "t12", "sternum")):
    if rel not in fiv.RELATION_PRESETS["validation1"]:
        fiv.RELATION_PRESETS["validation1"].append(rel)


def uw(x):
    return np.degrees(np.unwrap(np.radians(x)))


def relock(tm, msig, ti, isig, win, signed):
    g = np.arange(win[0], win[1], 0.01); M = np.interp(g, tm, msig if signed else np.abs(msig))
    best, lag = -np.inf, 0.0
    for L in np.arange(-0.5, 0.5 + 1e-9, 0.01):
        I = np.interp(A * g + B + L, ti, isig)
        if np.std(I) < 1e-6 or np.std(M) < 1e-6:
            continue
        c = np.corrcoef(I, M)[0, 1]
        if abs(c) > best:
            best, lag = abs(c), float(L)
    return lag


def per_bout(tm, msig, ti, isig, bouts, signed):
    out = []
    for a, b, *_ in bouts:
        lag = relock(tm, msig, ti, isig, (a, b), signed)
        hi = a - 0.2; lo = hi - 1.0; m0 = i0 = 0.0
        if hi > tm[0]:
            g0 = np.arange(max(tm[0], lo), hi, 0.01)
            if len(g0) >= 5:
                m0 = float(np.mean(np.interp(g0, tm, msig))); i0 = float(np.mean(np.interp(A * g0 + B + lag, ti, isig)))
        g = np.arange(a, b, 0.01)
        M = np.interp(g, tm, msig) - m0; I = np.interp(A * g + B + lag, ti, isig) - i0
        if not signed:
            M = np.abs(M)
        out.append((I, M))
    return out


def score(tm, msig, ti, isig, bouts, signed=False):
    pb = per_bout(tm, msig, ti, isig, bouts, signed)
    if not pb:
        return None
    I = np.concatenate([p[0] for p in pb]); M = np.concatenate([p[1] for p in pb])
    if np.std(I) < 1e-6 or np.std(M) < 1e-6:
        return None
    slope, intc = np.polyfit(I, M, 1)
    return (float(np.corrcoef(I, M)[0, 1]), float(slope),
            float(np.sqrt(np.mean((M - (slope * I + intc)) ** 2))), float(np.ptp(M)))


def loro(tm, msig, ti, isig, bouts, signed=False):
    pb = per_bout(tm, msig, ti, isig, bouts, signed)
    if len(pb) < 2:
        return None
    gains, sse, n = [], 0.0, 0; rom = float(np.ptp(np.concatenate([m for _, m in pb])))
    for k in range(len(pb)):
        Itr = np.concatenate([pb[j][0] for j in range(len(pb)) if j != k])
        Mtr = np.concatenate([pb[j][1] for j in range(len(pb)) if j != k])
        sl, ic = np.polyfit(Itr, Mtr, 1); gains.append(float(sl))
        Ite, Mte = pb[k]; sse += float(np.sum((Mte - (sl * Ite + ic)) ** 2)); n += len(Mte)
    return float(np.sqrt(sse / n)), gains, rom


def fmt(s):
    if s is None:
        return "   (no data)        "
    r, sl, cal, rom = s
    return f"gain={sl:4.2f} r={r:+4.2f} calRMSE={cal:4.1f} ({max(0,1-cal/max(rom,1e-6))*100:3.0f}%)"


def main() -> int:
    tm, mk, _ = mc.parse_motive_csv(MOC); tm = tm - tm[0]
    mk = {k: mc._fill_gaps(v) for k, v in mk.items()}
    ang = mc.trunk_angles(mk); st = tm <= 8
    flex = -(uw(ang["flex"]) - np.median(uw(ang["flex"])[st]))
    lat = -(uw(ang["lateral"]) - np.median(uw(ang["lateral"])[st]))
    ax = (uw(ang["axial"]) - np.median(uw(ang["axial"])[st]))

    def runs(mask, md=0.4):
        d = np.diff(mask.astype(int)); s = np.where(d == 1)[0] + 1; e = np.where(d == -1)[0] + 1
        if mask[0]: s = np.r_[0, s]
        if mask[-1]: e = np.r_[e, len(mask)]
        return [(i, j) for i, j in zip(s, e) if tm[j - 1] - tm[i] >= md]

    def segment(sig, sign, others, hi=20, lo=5, op=18, frac=0.30):
        s = sign * sig; out = []
        for i, j in runs(s > lo):
            pk = float(s[i:j].max())
            if pk < hi: continue
            k = i + int(np.argmax(s[i:j]))
            if abs(others[0][k]) > op or abs(others[1][k]) > op: continue
            thr = frac * pk; aa = k; bb = k
            while aa > i and s[aa] > thr: aa -= 1
            while bb < j - 1 and s[bb] > thr: bb += 1
            out.append((tm[aa], tm[bb]))
        return out

    MOVES = {
        "前倾 flexion": segment(flex, +1, (lat, ax)),
        "后仰 extension": segment(flex, -1, (lat, ax), hi=8, lo=3, op=12),
        "左倾 lat-L": segment(lat, +1, (flex, ax)),
        "右倾 lat-R": segment(lat, -1, (flex, ax)),
        "左转 twist-L": segment(ax, +1, (flex, lat)),
        "右转 twist-R": segment(ax, -1, (flex, lat)),
    }

    common = dict(layout_preset="validation1", filter="vqf", **{r: i for i, r in PM.items()})
    res = fiv.run_pipeline(RAW, fiv.make_args(**common)); ti = res.t_s
    sw_s = res.relations["sacrum_to_t3"].swing_deg      # S1 parent bend
    sw_t = res.relations["t12_to_t3"].swing_deg          # T12 parent bend
    tw_s = res.relations["sacrum_to_sternum"].twist_deg  # S1 parent twist
    tw_t = res.relations["t12_to_sternum"].twist_deg     # T12 parent twist

    # movement -> (mocap signal, OLD imu, NEW imu, signed)
    PLAN = {
        "前倾 flexion":   (flex, sw_s, sw_s, False),   # S1 both (unchanged)
        "后仰 extension": (flex, sw_s, sw_t, False),   # T12 new
        "左倾 lat-L":     (lat,  sw_s, sw_t, False),
        "右倾 lat-R":     (lat,  sw_s, sw_t, False),
        "左转 twist-L":   (ax,   tw_s, tw_t, True),
        "右转 twist-R":   (ax,   tw_s, tw_t, True),
    }

    print("=== movement-dependent reference (flexion=S1, others=T12) vs OLD all-sacrum ===")
    print(f"{'movement':16s} | {'OLD (S1 ref)':38s} | {'NEW (T12 ref)':38s}")
    for nm, (msig, old, new, sg) in PLAN.items():
        so = score(tm, msig, ti, old, MOVES[nm], sg); sn = score(tm, msig, ti, new, MOVES[nm], sg)
        tag = "  (same)" if old is new else ""
        print(f"{nm:16s} | {fmt(so):38s} | {fmt(sn):38s}{tag}")

    print("\n=== held-out (leave-one-rep-out) on the NEW movement-dependent reference ===")
    for nm, (msig, old, new, sg) in PLAN.items():
        lo = loro(tm, msig, ti, new, MOVES[nm], sg)
        if lo is None:
            print(f"  {nm:16s} (need >=2 reps)"); continue
        rmse, gains, rom = lo
        print(f"  {nm:16s} held-out RMSE={rmse:4.1f}  acc={max(0,1-rmse/max(rom,1e-6))*100:3.0f}%  gains={'/'.join(f'{g:.2f}' for g in gains)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
