from __future__ import annotations

"""2026-06-29 re-take (4-marker cluster per IMU): MoCap-vs-IMU using robust cluster CENTROIDS.

Finding from diag_cluster.py: the 4-marker clusters are NOT rigid on the stretch garment (Kabsch
rigid-fit residual 4-6mm on a 41mm cluster, + occlusion spikes), so a clean rigid-body ORIENTATION
can't be recovered from them -- the per-cluster 'patch orientation' we hoped for is corrupted.
BUT the cluster CENTROID (mean of 4 markers) is robust (deformation averages out), giving a much
cleaner segment point than a single marker. So this script redoes the validation2 chord/tangent
comparison with cluster centroids, on the new take, and reports the gain + leave-one-rep-out held-out.

Cluster<->IMU map: S1=sacrum U1, T12=U2, T6=U3, T3=U4, Sternum=U5. Run with .venv/Scripts/python.exe.
"""

import csv
import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import five_imu_fusion as fiv

MOC = HERE / "data/mocap/Take 2026-06-29 06.13.46 PM.csv"
LOG = HERE / "data/twist_trial_20260629_181244.log"
PM = {"IMU0": "sacrum", "IMU1": "t12", "IMU2": "t6", "IMU3": "t3", "IMU4": "sternum"}
PRE = "Trunk 2:"
CLUSTERS = {
    "sacrum": ["S1U", "S1B", "S1L", "S1R"], "t12": ["T12U", "T12B", "T12L", "T12R"],
    "t6": ["T6U", "T6B", "T6L", "T6R"], "t3": ["T3u", "T3B", "T3L", "T3R"],
    "sternum": ["SternumU", "SternumB", "SternumL", "SternumR"],
}
PELVIS = ["LASIS", "RASIS", "LSIS", "RSIS"]
A, B = 1.006, 63.36  # set by align_swing at runtime


def parse(path: Path):
    rows = list(csv.reader(path.open(encoding="utf-8", errors="replace")))
    name_row = next(r for r in rows if len(r) > 1 and r[1] == "Name")
    data = [r for r in rows if r and r[0].isdigit()]
    ncol = max(len(r) for r in data)
    mat = np.full((len(data), ncol), np.nan)
    for k, r in enumerate(data):
        for j, v in enumerate(r):
            if v:
                try:
                    mat[k, j] = float(v)
                except ValueError:
                    pass
    cols: dict[str, list[int]] = {}
    for j in range(2, len(name_row)):
        if name_row[j]:
            cols.setdefault(name_row[j], []).append(j)
    t = mat[:, 1] - mat[0, 1]
    return t, {nm: mat[:, idx[:3]] for nm, idx in cols.items()}


def fill(a):
    a = a.astype(float).copy(); idx = np.arange(a.shape[0])
    for c in range(a.shape[1]):
        good = np.isfinite(a[:, c])
        if good.sum() >= 2:
            a[:, c] = np.interp(idx, idx[good], a[good, c])
    return a


def _u(v):
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.where(n == 0, np.nan, n)


def uw(x):
    return np.degrees(np.unwrap(np.radians(x)))


def cen(mk, name):
    return np.mean([mk[PRE + n] for n in CLUSTERS[name]], axis=0)


def pelvis_basis(mk):
    L, R, Lp, Rp = mk[PRE + "LASIS"], mk[PRE + "RASIS"], mk[PRE + "LSIS"], mk[PRE + "RSIS"]
    ml = _u(R - L)
    up = _u(np.cross(ml, (L + R) / 2 - (Lp + Rp) / 2))
    fwd = _u(np.cross(up, ml))
    return fwd, up, ml


def chord_tilt(p_par, p_chi, fwd, up, ml, still):
    v = p_chi - p_par
    flex = uw(np.degrees(np.arctan2(np.sum(v * fwd, axis=1), np.sum(v * up, axis=1))))
    lat = uw(np.degrees(np.arctan2(np.sum(v * ml, axis=1), np.sum(v * up, axis=1))))
    return flex - np.median(flex[still]), lat - np.median(lat[still])


def axial(ant, post, fwd, ml, still):
    fr = ant - post
    ax = uw(np.degrees(np.arctan2(np.sum(fr * ml, axis=1), np.sum(fr * fwd, axis=1))))
    return ax - np.median(ax[still])


def align_swing(tm, m, ti, i):
    def corr(a, b):
        g0 = max(tm[0], (ti[0] - b) / a); g1 = min(tm[-1], (ti[-1] - b) / a)
        if g1 - g0 < 20:
            return -2.0
        g = np.arange(g0, g1, 0.05); M = np.interp(g, tm, m); I = np.interp(a * g + b, ti, i)
        if np.std(I) < 1e-6 or np.std(M) < 1e-6:
            return -2.0
        return float(np.corrcoef(I, M)[0, 1])
    Bs = np.arange(0.0, 150.0, 0.5); B0 = Bs[int(np.argmax([corr(1.0, b) for b in Bs]))]
    best = (-2.0, 1.0, B0)
    for a in np.arange(0.994, 1.006, 0.0004):
        for b in np.arange(B0 - 3, B0 + 3, 0.04):
            c = corr(a, b)
            if c > best[0]:
                best = (c, a, b)
    return best[1], best[2], best[0]


def runs(tm, mask, md=0.4):
    d = np.diff(mask.astype(int)); s = np.where(d == 1)[0] + 1; e = np.where(d == -1)[0] + 1
    if mask[0]: s = np.r_[0, s]
    if mask[-1]: e = np.r_[e, len(mask)]
    return [(i, j) for i, j in zip(s, e) if tm[j - 1] - tm[i] >= md]


def segment(tm, sig, sign, others, hi=18, lo=5, opurity=16, frac=0.30):
    s = sign * sig; out = []
    for i, j in runs(tm, s > lo):
        pk = float(s[i:j].max())
        if pk < hi:
            continue
        k = i + int(np.argmax(s[i:j]))
        if abs(others[0][k]) > opurity or abs(others[1][k]) > opurity:
            continue
        thr = frac * pk; a = k; b = k
        while a > i and s[a] > thr: a -= 1
        while b < j - 1 and s[b] > thr: b += 1
        out.append((tm[a], tm[b]))
    return out


def per_bout(tm, msig, ti, isig, bouts, signed):
    out = []
    for a, b in bouts:
        g = np.arange(a, b, 0.01)
        best, lag = -np.inf, 0.0
        for L in np.arange(-0.5, 0.5 + 1e-9, 0.01):
            I = np.interp(A * g + B + L, ti, isig); M = np.interp(g, tm, msig if signed else np.abs(msig))
            if np.std(I) < 1e-6 or np.std(M) < 1e-6:
                continue
            c = np.corrcoef(I, M)[0, 1]
            if abs(c) > best:
                best, lag = abs(c), float(L)
        hi = a - 0.2; lo = hi - 1.0; m0 = i0 = 0.0
        if hi > tm[0]:
            g0 = np.arange(max(tm[0], lo), hi, 0.01)
            if len(g0) >= 5:
                m0 = float(np.mean(np.interp(g0, tm, msig))); i0 = float(np.mean(np.interp(A * g0 + B + lag, ti, isig)))
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
            float(np.sqrt(np.mean((M - I) ** 2))), float(np.sqrt(np.mean((M - (slope * I + intc)) ** 2))),
            float(np.ptp(M)))


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


def prow(label, s):
    if s is None:
        print(f"  {label:22s}  (no data)"); return
    r, sl, raw, cal, rom = s
    print(f"  {label:22s} gain={sl:4.2f} r={r:+4.2f} | UNGAINED rmse={raw:5.1f} acc={max(0,1-raw/max(rom,1e-6))*100:3.0f}% "
          f"| gained rmse={cal:4.1f} acc={max(0,1-cal/max(rom,1e-6))*100:3.0f}% | ROM={rom:4.0f}")


def lrow(label, s):
    if s is None:
        print(f"  {label:22s}  (need >=2 reps)"); return
    rmse, gains, rom = s
    print(f"  {label:22s} held-out RMSE={rmse:4.1f} acc={max(0,1-rmse/max(rom,1e-6))*100:3.0f}%  per-fold gain={'/'.join(f'{g:.2f}' for g in gains)}")


def main() -> int:
    global A, B
    tm, mk = parse(MOC); mk = {k: fill(v) for k, v in mk.items()}; still = tm <= 8.0
    print(f"MoCap {MOC.name}: {len(tm)} frames, {tm[-1]:.0f}s  (cluster CENTROIDS; orientation unusable, see diag)")
    fwd, up, ml = pelvis_basis(mk)
    C = {n: cen(mk, n) for n in CLUSTERS}
    flex, lat = chord_tilt(C["sacrum"], C["t3"], fwd, up, ml, still)        # big-span bend
    ax_end = axial(C["sternum"], C["sacrum"], fwd, ml, still)               # twist, endpoints
    ax_t6 = axial(C["sternum"], C["t6"], fwd, ml, still)                    # twist, matched level

    common = dict(layout_preset="validation1", filter="vqf", **{r: i for i, r in PM.items()})
    res = fiv.run_pipeline(LOG, fiv.make_args(**common)); ti = res.t_s
    imu_sw = res.relations["sacrum_to_t3"].swing_deg
    imu_tw = res.relations["sacrum_to_sternum"].twist_deg

    A, B, q = align_swing(tm, np.hypot(flex, lat), ti, imu_sw)
    print(f"clock: t_imu = {A:.6f}*t_mocap + {B:.3f}  (align r={q:.2f})")

    MOVES = {
        "前倾 flexion": segment(tm, flex, +1, (lat, ax_end)),
        "后仰 extension": segment(tm, flex, -1, (lat, ax_end), hi=8, lo=3, opurity=12),
        "左倾 lat-L": segment(tm, lat, +1, (flex, ax_end)),
        "右倾 lat-R": segment(tm, lat, -1, (flex, ax_end)),
        "左转 twist-L": segment(tm, ax_end, +1, (flex, lat)),
        "右转 twist-R": segment(tm, ax_end, -1, (flex, lat)),
    }
    print("bouts:", {k: len(v) for k, v in MOVES.items()})

    print("\n=== BENDING (cluster-centroid chord vs IMU sacrum_to_t3) ===")
    for nm in ("前倾 flexion", "后仰 extension", "左倾 lat-L", "右倾 lat-R"):
        src = flex if "flex" in nm or "ext" in nm else lat
        prow(nm, score(tm, src, ti, imu_sw, MOVES[nm]))
    print("=== TWIST (cluster-centroid, vs IMU sacrum_to_sternum) ===")
    tw_bouts = MOVES["左转 twist-L"] + MOVES["右转 twist-R"]
    prow("twist XIP-sacrum", score(tm, ax_end, ti, imu_tw, tw_bouts, signed=True))
    prow("twist XIP-t6", score(tm, ax_t6, ti, imu_tw, tw_bouts, signed=True))

    # robust pooled bending: magnitude vs magnitude, no sign/segmentation needed
    bendmag = np.hypot(flex, lat)
    bw = [(tm[i], tm[j - 1]) for i, j in runs(tm, bendmag > 15)]
    print(f"\n=== POOLED bending magnitude vs IMU swing (no segmentation; {len(bw)} active windows) ===")
    prow("bend |mag|", score(tm, bendmag, ti, imu_sw, bw))
    lrow("bend |mag| held-out", loro(tm, bendmag, ti, imu_sw, bw))
    print("=== held-out twist (cluster centroid, XIP-sacrum) ===")
    lrow("twist pooled", loro(tm, ax_end, ti, imu_tw, tw_bouts, signed=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
