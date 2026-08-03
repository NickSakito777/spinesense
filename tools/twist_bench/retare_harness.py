from __future__ import annotations
"""Shared harness for the twist re-tare experiment.

Runs the expensive VQF pipeline ONCE, caches per-role q_segment (sensor->world) to npz.
Exposes the MoCap twist reference + clock + IMU-covered twist bouts + scoring, so each
candidate re-tare strategy is a small, comparable, NON-CIRCULAR script:

    import retare_harness as H
    d = H.load()
    q_rel = H.qrel(d, "mid", "sternum")          # raw relative quaternion, per sample
    twist = H.decompose(q_rel, tare_idx)         # your re-tare -> twist_deg (already sign-flipped +L)
    print(H.score(d, twist))                     # left/right pooled r + held-out RMSE vs MoCap

Non-circularity contract: MoCap is used ONLY to score. Re-tare windows must be chosen from
IMU stillness (gyro) or protocol timing, never fit to the MoCap angle. `still_before` picks the
quietest IMU sub-window before a time anchor purely from gyro magnitude.
"""

import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import five_imu_fusion as fiv
import signed_diagnostic as sd
import twist_bench_fusion as pf
import validation3_cluster_orientation as v3

# Session inputs. Override with --mocap / --log to point at your own recording.
MOC = HERE / "data/mocap/session_mocap.csv"
LOG = HERE / "data/sessions/session.log"
CACHE = HERE / "retare_cache.npz"
ROLES = ("sacrum", "lower", "mid", "upper", "sternum")
AXIS = pf.SEGMENT_TWIST_AXIS


def ensure_cache():
    if CACHE.exists():
        return
    res = fiv.run_pipeline(LOG, fiv.make_args(layout_preset="t01", filter="vqf"))
    d = {"t_imu": res.t_s}
    for r in ROLES:
        d[f"q_{r}"] = res.sensors[r].q_segment
        d[f"gyr_{r}"] = res.sensors[r].gyr_cal_dps
    np.savez_compressed(CACHE, **d)
    print(f"cached {CACHE}")


def load():
    ensure_cache()
    z = np.load(CACHE)
    ti = z["t_imu"]
    q = {r: z[f"q_{r}"] for r in ROLES}
    gyr = {r: z[f"gyr_{r}"] for r in ROLES}

    # ---- MoCap reference (identical to validation_p02) ----
    tm, mkraw = v3.parse(MOC)
    mk = {k: v3.fill(v) for k, v in mkraw.items()}
    still = tm <= 8.0
    m = lambda n: mk["Trunk:" + n]
    mid = lambda a, b: (m(a) + m(b)) / 2
    fwd, up, ml = sd.pelvis_basis(mk)
    P = {"upper": mid("C7 (2)", "T2"),
         "sacrum": (m("L3") + m("S2") + m("LPSIS_B") + m("RPSIS_B")) / 4}
    flex, lat = v3.chord_tilt(P["sacrum"], P["upper"], fwd, up, ml, still)
    twist = v3.axial(mid("JN", "XP"), P["sacrum"], fwd, ml, still)
    ld = (np.abs(lat) > 15) & (np.abs(flex) < 10)
    if ld.sum() > 50:
        twist = twist - np.polyfit(lat[ld], twist[ld], 1)[0] * lat

    # ---- clock from IMU bend swing (sacrum->upper) ----
    q_su = qrel_arr(q["sacrum"], q["upper"])
    tare0 = ti <= 8.0
    q_su_t = pf.qmul(pf.qconj(pf.quat_average(q_su[tare0]))[None, :], q_su)
    _, sw = pf.swing_twist_deg(q_su_t, AXIS)
    v3.A, v3.B, _ = v3.align_swing(tm, np.hypot(flex, lat), ti, sw)

    cov = (ti[-1] - v3.B) / v3.A
    boutsL = [b for b in v3.segment(tm, twist, +1, (flex, lat)) if b[1] <= cov - 0.3]
    boutsR = [b for b in v3.segment(tm, twist, -1, (flex, lat)) if b[1] <= cov - 0.3]

    return dict(ti=ti, q=q, gyr=gyr, tm=tm, twist=twist, flex=flex, lat=lat,
                A=v3.A, B=v3.B, cov=cov, boutsL=boutsL, boutsR=boutsR)


def qrel_arr(qp, qc):
    return pf.qmul(pf.qconj(qp), qc)


def qrel(d, parent, child):
    return qrel_arr(d["q"][parent], d["q"][child])


def decompose(q_rel, tare_idx, axis=AXIS, flip=True):
    """Tare q_rel by the reference orientation averaged over tare_idx, decompose, sign-flip to +L."""
    q0 = pf.quat_average(q_rel[tare_idx])
    q_t = pf.qmul(pf.qconj(q0)[None, :], q_rel)
    tw, _ = pf.swing_twist_deg(q_t, axis)
    tw = pf.unwrap_deg(tw)
    return -tw if flip else tw


def still_before(d, t_anchor_mocap, roles=("mid", "sternum"), look=2.5, win=0.5):
    """IMU-side still window (index mask) ending just before a MoCap time anchor.
    Picks the `win`-second sub-window in the `look` seconds before the anchor with the
    lowest summed gyro magnitude across `roles`. Pure IMU, no MoCap angle used."""
    ti = d["ti"]
    a = d["A"] * t_anchor_mocap + d["B"]
    lo, hi = a - look, a - 0.15
    seg = np.where((ti >= lo) & (ti < hi))[0]
    if len(seg) < 5:
        m = np.zeros(len(ti), dtype=bool)
        m[seg] = True
        return m if seg.size else (ti <= 8.0)
    mag = sum(np.linalg.norm(d["gyr"][r], axis=1) for r in roles)
    n = max(3, int(win / np.median(np.diff(ti))))
    csum = np.cumsum(np.insert(mag[seg], 0, 0.0))
    best = min(range(len(seg) - n + 1), key=lambda k: csum[k + n] - csum[k])
    m = np.zeros(len(ti), dtype=bool)
    m[seg[best:best + n]] = True
    return m


def score(d, twist_imu, signed=True):
    """Return dict of left/right pooled r, gain, and held-out RMSE/acc vs MoCap covered bouts."""
    ti = d["ti"]
    out = {}
    for tag, bouts in (("L", d["boutsL"]), ("R", d["boutsR"])):
        s = v3.score(d["tm"], d["twist"], ti, twist_imu, bouts, signed=signed)
        lo = v3.loro(d["tm"], d["twist"], ti, twist_imu, bouts, signed=signed)
        if s is None:
            out[tag] = None
            continue
        r, sl, raw, cal, rom = s
        rec = dict(n=len(bouts), r=r, gain=sl, cal_rmse=cal, rom=rom)
        if lo is not None:
            rmse, gains, romL = lo
            rec.update(ho_rmse=rmse, ho_acc=max(0, 1 - rmse / max(romL, 1e-6)),
                       gain_spread=(min(gains), max(gains)))
        out[tag] = rec
    return out


def fmt(out):
    lines = []
    for tag in ("L", "R"):
        rec = out.get(tag)
        if rec is None:
            lines.append(f"  twist-{tag}: no data"); continue
        ho = f"held-out {rec['ho_rmse']:.1f}° ({rec['ho_acc']*100:.0f}%)" if "ho_rmse" in rec else "held-out n/a"
        gs = f" gain[{rec['gain_spread'][0]:.1f}-{rec['gain_spread'][1]:.1f}]" if "gain_spread" in rec else ""
        lines.append(f"  twist-{tag} (n={rec['n']}): pooled r={rec['r']:+.2f} gain={rec['gain']:.2f} | {ho}{gs}")
    return "\n".join(lines)


if __name__ == "__main__":
    d = load()
    print(f"clock t_imu={d['A']:.4f}*t+{d['B']:.1f}  coverage to MoCap {d['cov']:.0f}s")
    print(f"covered bouts: L={len(d['boutsL'])} R={len(d['boutsR'])}")
    # baseline: global tare (t<=8s), mid->sternum, fixed axis -- must reproduce r_L~0.52 r_R~0.95
    q_ms = qrel(d, "mid", "sternum")
    base = decompose(q_ms, d["ti"] <= 8.0)
    print("\n[baseline] global tare, mid->sternum, fixed axis:")
    print(fmt(score(d, base)))
