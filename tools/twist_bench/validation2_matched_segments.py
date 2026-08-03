from __future__ import annotations

"""Marker-MATCHED MoCap segments vs IMU relations (Validation1 P01 re-analysis).

Tests the hypothesis (Fox, 2026-06-29): the pre-gain magnitude error (esp. bending over-read ~2x)
is an ARTEFACT of the MoCap truth being whole-thorax-vs-whole-pelvis while the IMU spans specific
sensor sites. Now that the IMU<->marker placement is known, rebuild the MoCap truth on the SAME
spans and see whether the gain (slope in  MoCap = slope*IMU + b)  moves toward 1.

Placement (Fox, 2026-06-29), IMU -> bracketing markers -> matched MoCap point:
  U5 sternum = Xiphoid (anterior)              -> P5 = Xiphoid
  U4 t3      = between T1.2 and T3             -> P4 = mid(T1.2, T3)
  U3 t6      = between T7 and T12              -> P3 = mid(T7, T12)
  U2 t12     = between T12 and L1              -> P2 = mid(T12, L1)
  U1 sacrum  = between L5/S2/LPSIS-B/RPSIS-B   -> P1 = centroid(L5, S2, LPSIS-B, RPSIS-B)

Granularity (both swept):
  big-span    : IMU sacrum_to_t3   vs MoCap chord P1->P4
  per-segment : IMU sacrum_to_t12  vs P1->P2 | t12_to_t6 vs P2->P3 | t6_to_t3 vs P3->P4
Baseline for comparison: the OLD thorax-vs-pelvis truth (mocap_adapter.trunk_angles) vs sacrum_to_t3.

Twist anterior/posterior anchors (both/all swept), all vs IMU sacrum_to_sternum twist:
  T0 orig : (IJ+PX)/2          minus  mean(T1,T3,T7,T12)
  T1      : Xiphoid            minus  mid(T7,T12)   [matched to U5 level]
  T2      : Xiphoid            minus  T12
  T3      : Xiphoid            minus  mid(T12,L1)
  T4      : Xiphoid            minus  P1 (sacrum)   [exact U1<->U5 endpoints]

Bouts are auto-detected from the robust trunk-level signal (same as validation1_full_accuracy), then
every variant is evaluated inside the SAME windows with the same per-bout relock + local re-tare, so
only the segment DEFINITION differs across rows. N=1 P01 pilot. Run with .venv/Scripts/python.exe.
"""

import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import five_imu_fusion as fiv
import mocap_adapter as mc

# additive, runtime: let the parser also return the lumbar markers (originals left untouched on disk)
mc.MK.update({"L1": "Trunk:L1", "L3": "Trunk:L3", "L5": "Trunk:L5"})

A, B = 1.002402, 61.889  # drift-corrected clock map t_imu = A*t_mocap + B (P01 pilot)
RAW = HERE / "data/twist_trial_20260626_170028.log"
MOC = HERE / "data/mocap/Testing_01(1).csv"
PM = {"IMU0": "sacrum", "IMU1": "t12", "IMU2": "t6", "IMU3": "t3", "IMU4": "sternum"}
RETARE_S, RETARE_GAP_S, MAXLAG = 1.0, 0.2, 0.5

MOVES_ORDER = ("前倾 flexion", "后仰 extension", "左倾 lat-L", "右倾 lat-R")


def _u(v):
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.where(n == 0, np.nan, n)


def uw(x):
    return np.degrees(np.unwrap(np.radians(x)))


def pelvis_basis(m):
    asis_mid = (m["LASIS"] + m["RASIS"]) / 2.0
    psis_mid = (m["LPSIS"] + m["RPSIS"]) / 2.0
    ml = _u(m["RASIS"] - m["LASIS"])                 # L->R = subject right
    up = _u(np.cross(ml, asis_mid - psis_mid))       # right x anterior = up
    fwd = _u(np.cross(up, ml))                        # re-orthogonalised anterior
    return fwd, up, ml


def seg_points(m):
    return {
        "P1": (m["L5"] + m["S2"] + m["LPSIS"] + m["RPSIS"]) / 4.0,
        "P2": (m["T12"] + m["L1"]) / 2.0,
        "P3": (m["T7"] + m["T12"]) / 2.0,
        "P4": (m["T1"] + m["T3"]) / 2.0,   # MK["T1"] -> "Trunk:T1.2"
        "P5": m["PX"],
    }


def chord_bend(p_par, p_chi, fwd, up, ml, still):
    """flex/lateral tilt (deg) of the SECANT p_par->p_chi in the pelvis frame, tared on `still`.
    This is the straight chord (kept only to contrast with the local-tangent definition)."""
    return dir_tilt(_u(p_chi - p_par), fwd, up, ml, still)


def dir_tilt(d, fwd, up, ml, still):
    """Absolute flex/lateral tilt (deg) of a unit direction `d` in the pelvis frame, tared on `still`.
    Sign is irrelevant downstream (bending is scored unsigned)."""
    flex = uw(np.degrees(np.arctan2(np.sum(d * fwd, axis=1), np.sum(d * up, axis=1))))
    lat = uw(np.degrees(np.arctan2(np.sum(d * ml, axis=1), np.sum(d * up, axis=1))))
    return flex - np.median(flex[still]), lat - np.median(lat[still])


def axial_anchor(anterior, posterior, fwd, ml, still):
    """transverse-plane rotation (deg) of the posterior->anterior vector in the pelvis frame, tared."""
    fr = anterior - posterior
    ax = uw(np.degrees(np.arctan2(np.sum(fr * ml, axis=1), np.sum(fr * fwd, axis=1))))
    return ax - np.median(ax[still])


# ---------- bout auto-segmentation (verbatim shape from validation1_full_accuracy) ----------
def detect_bouts(tm, flex, lat, ax):
    def runs(mask, min_dur=0.4):
        d = np.diff(mask.astype(int)); s = np.where(d == 1)[0] + 1; e = np.where(d == -1)[0] + 1
        if mask[0]: s = np.r_[0, s]
        if mask[-1]: e = np.r_[e, len(mask)]
        return [(i, j) for i, j in zip(s, e) if tm[j - 1] - tm[i] >= min_dur]

    def segment(sig, sign, others, hi=20, lo=5, opurity=18, frac=0.30):
        s = sign * sig; out = []
        for i, j in runs(s > lo):
            pk = float(s[i:j].max())
            if pk < hi: continue
            k = i + int(np.argmax(s[i:j]))
            if abs(others[0][k]) > opurity or abs(others[1][k]) > opurity: continue
            thr = frac * pk; a = k; b = k
            while a > i and s[a] > thr: a -= 1
            while b < j - 1 and s[b] > thr: b += 1
            out.append((tm[a], tm[b], float(sig[k])))
        return out

    return {
        "前倾 flexion":   segment(flex, +1, (lat, ax)),
        "后仰 extension": segment(flex, -1, (lat, ax), hi=8, lo=3, opurity=12),
        "左倾 lat-L":     segment(lat,  +1, (flex, ax)),
        "右倾 lat-R":     segment(lat,  -1, (flex, ax)),
        "左转 twist-L":   segment(ax,   +1, (flex, lat)),
        "右转 twist-R":   segment(ax,   -1, (flex, lat)),
    }


# ---------- per-bout relock + local re-tare + calibrated stats (matches headline method) ----------
def relock(tm, msig, ti, isig, win):
    g = np.arange(win[0], win[1], 0.01); M = np.interp(g, tm, msig)
    best_abs, best_lag = -np.inf, 0.0
    for lag in np.arange(-MAXLAG, MAXLAG + 1e-9, 0.01):
        I = np.interp(A * g + B + lag, ti, isig)
        if np.std(I) < 1e-6 or np.std(M) < 1e-6: continue
        c = np.corrcoef(I, M)[0, 1]
        if abs(c) > best_abs: best_abs, best_lag = abs(c), float(lag)
    return best_lag


def local_offsets(tm, msig, ti, isig, a, lag):
    hi = a - RETARE_GAP_S; lo = hi - RETARE_S
    if hi <= tm[0]: return 0.0, 0.0
    g0 = np.arange(max(tm[0], lo), hi, 0.01)
    if len(g0) < 5: return 0.0, 0.0
    return float(np.mean(np.interp(g0, tm, msig))), float(np.mean(np.interp(A * g0 + B + lag, ti, isig)))


def score(tm, msig, ti, isig, bouts, signed=False):
    """Return (r, slope, raw_rmse, cal_rmse, rom). slope = gain in MoCap = slope*IMU + b (->1 is matched).
    raw_rmse = error with NO gain (the 'pre-gain degrees are off' number). cal_rmse = after the affine fit."""
    Iall, Mall = [], []
    for a, b, _ in bouts:
        lag = relock(tm, msig if signed else np.abs(msig), ti, isig, (a, b))
        m0, i0 = local_offsets(tm, msig, ti, isig, a, lag)
        g = np.arange(a, b, 0.01)
        M = np.interp(g, tm, msig) - m0
        I = np.interp(A * g + B + lag, ti, isig) - i0
        if not signed: M = np.abs(M)
        Iall.append(I); Mall.append(M)
    if not Iall:
        return None
    I = np.concatenate(Iall); M = np.concatenate(Mall)
    if np.std(I) < 1e-6 or np.std(M) < 1e-6:
        return None
    slope, intc = np.polyfit(I, M, 1)
    r = float(np.corrcoef(I, M)[0, 1])
    raw_rmse = float(np.sqrt(np.mean((M - I) ** 2)))
    cal_rmse = float(np.sqrt(np.mean((M - (slope * I + intc)) ** 2)))
    return r, float(slope), raw_rmse, cal_rmse, float(np.ptp(M))


def per_bout_IM(tm, msig, ti, isig, bouts, signed):
    out = []
    for a, b, _ in bouts:
        lag = relock(tm, msig if signed else np.abs(msig), ti, isig, (a, b))
        m0, i0 = local_offsets(tm, msig, ti, isig, a, lag)
        g = np.arange(a, b, 0.01)
        M = np.interp(g, tm, msig) - m0
        I = np.interp(A * g + B + lag, ti, isig) - i0
        if not signed:
            M = np.abs(M)
        out.append((I, M))
    return out


def loro(tm, msig, ti, isig, bouts, signed=False):
    """Leave-one-rep-out held-out calibration: fit gain+offset on the OTHER reps, test on the held-out
    rep (never used to fit). Non-circular -> if held-out RMSE ~ in-sample RMSE, the gain is a stable
    CALIBRATION, not 'copying the answer'. Also returns the per-fold gains to show stability."""
    pb = per_bout_IM(tm, msig, ti, isig, bouts, signed)
    if len(pb) < 2:
        return None
    gains, sse, n = [], 0.0, 0
    rom_all = float(np.ptp(np.concatenate([m for _, m in pb])))
    for k in range(len(pb)):
        I_tr = np.concatenate([pb[j][0] for j in range(len(pb)) if j != k])
        M_tr = np.concatenate([pb[j][1] for j in range(len(pb)) if j != k])
        slope, intc = np.polyfit(I_tr, M_tr, 1)
        gains.append(float(slope))
        I_te, M_te = pb[k]
        sse += float(np.sum((M_te - (slope * I_te + intc)) ** 2)); n += len(M_te)
    rmse = float(np.sqrt(sse / max(n, 1)))
    acc = max(0.0, 1 - rmse / max(rom_all, 1e-6)) * 100
    return rmse, acc, gains, rom_all


def lrow(label, s):
    if s is None:
        print(f"  {label:26s}   (need >=2 reps)")
        return
    rmse, acc, gains, rom = s
    gtxt = "/".join(f"{g:.2f}" for g in gains)
    print(f"  {label:26s} held-out RMSE={rmse:4.1f}  acc={acc:3.0f}%  ROM={rom:4.0f}  | per-fold gain={gtxt}")


def prow(label, s):
    if s is None:
        print(f"  {label:30s}   (no bouts / flat)")
        return
    r, slope, raw, cal, rom = s
    raw_acc = max(0.0, 1 - raw / max(rom, 1e-6)) * 100
    cal_acc = max(0.0, 1 - cal / max(rom, 1e-6)) * 100
    print(f"  {label:30s} gain={slope:4.2f} r={r:+4.2f} | UNGAINED rmse={raw:5.1f} acc={raw_acc:3.0f}% "
          f"| gained rmse={cal:4.1f} acc={cal_acc:3.0f}% | ROM={rom:4.0f}")


def main() -> int:
    # ---- MoCap ----
    tm, mk, nanf = mc.parse_motive_csv(MOC); tm = tm - tm[0]
    print(f"MoCap {MOC.name}: {len(tm)} frames, {tm[-1]:.0f}s")
    print("new-marker NaN frac:", ", ".join(f"{k}={nanf[k]:.2f}" for k in ("L1", "L3", "L5", "S2", "LPSIS", "RPSIS", "T1", "T3", "T7", "T12", "PX")))
    mk = {k: mc._fill_gaps(v) for k, v in mk.items()}
    still = tm <= 8.0

    # trunk-level signals for bout detection + the OLD baseline truth (sign as in headline script)
    ang = mc.trunk_angles(mk)
    flex_tr = -(uw(ang["flex"]) - np.median(uw(ang["flex"])[still]))
    lat_tr = -(uw(ang["lateral"]) - np.median(uw(ang["lateral"])[still]))
    ax_tr = uw(ang["axial"]) - np.median(uw(ang["axial"])[still])
    MOVES = detect_bouts(tm, flex_tr, lat_tr, ax_tr)
    print("\n=== auto-detected bouts (MoCap s : peak deg) ===")
    for nm, bts in MOVES.items():
        print(f"  {nm:16s}: " + "  ".join(f"{a:.0f}-{b:.0f}s({p:+.0f})" for a, b, p in bts))

    # matched segment definitions + twist anchors
    fwd_p, up_p, ml_p = pelvis_basis(mk)
    P = seg_points(mk)

    # SECANT big span (straight chord P1->P4) -- kept only to contrast with the tangent definition
    sec_flex, sec_lat = chord_bend(P["P1"], P["P4"], fwd_p, up_p, ml_p, still)

    # LOCAL TANGENT at each IMU site (what the sensor patch actually senses). Central differences along
    # the posterior chain; sacrum site == pelvis up (its absolute tilt is 0 by construction).
    fa1, la1 = dir_tilt(up_p, fwd_p, up_p, ml_p, still)                  # U1 sacrum  (~0)
    fa2, la2 = dir_tilt(_u(P["P3"] - P["P1"]), fwd_p, up_p, ml_p, still)  # U2 T12  tangent
    fa3, la3 = dir_tilt(_u(P["P4"] - P["P2"]), fwd_p, up_p, ml_p, still)  # U3 T6   tangent
    fa4, la4 = dir_tilt(_u(P["P4"] - P["P3"]), fwd_p, up_p, ml_p, still)  # U4 T3   tangent
    bs_flex, bs_lat = fa4, la4                                            # big span sacrum->t3 == T3 tangent vs pelvis
    # per-segment matched = RELATIVE tilt between adjacent site tangents (matches IMU local relations)
    seg_chords = {
        "U1->U2 (sacrum_to_t12)": ((fa2 - fa1, la2 - la1), "sacrum_to_t12"),
        "U2->U3 (t12_to_t6)":     ((fa3 - fa2, la3 - la2), "t12_to_t6"),
        "U3->U4 (t6_to_t3)":      ((fa4 - fa3, la4 - la3), "t6_to_t3"),
    }
    spine_cent = (mk["T1"] + mk["T3"] + mk["T7"] + mk["T12"]) / 4.0
    twist_anchors = {
        "T0 (IJ+PX)/2 - spine4": axial_anchor((mk["IJ"] + mk["PX"]) / 2.0, spine_cent, fwd_p, ml_p, still),
        "T1 XIP - mid(T7,T12)":  axial_anchor(mk["PX"], P["P3"], fwd_p, ml_p, still),
        "T2 XIP - T12":          axial_anchor(mk["PX"], mk["T12"], fwd_p, ml_p, still),
        "T3 XIP - mid(T12,L1)":  axial_anchor(mk["PX"], P["P2"], fwd_p, ml_p, still),
        "T4 XIP - sacrum(P1)":   axial_anchor(mk["PX"], P["P1"], fwd_p, ml_p, still),
    }

    # ---- IMU pipeline (VQF, fixed twist axis) ----
    common = dict(layout_preset="validation1", filter="vqf", **{r: i for i, r in PM.items()})
    res = fiv.run_pipeline(RAW, fiv.make_args(**common))
    ti = res.t_s
    sw = {r: res.relations[r].swing_deg for r in ("sacrum_to_t3", "sacrum_to_t12", "t12_to_t6", "t6_to_t3")}
    tw = res.relations["sacrum_to_sternum"].twist_deg

    # ================= BENDING =================
    print("\n=== BENDING gain (MoCap = gain*IMU + b; gain->1 == matched). unsigned. ===")
    print("--- baseline: OLD thorax-vs-pelvis truth  vs IMU sacrum_to_t3 ---")
    for nm in MOVES_ORDER:
        src = flex_tr if nm in ("前倾 flexion", "后仰 extension") else lat_tr
        prow(f"OLD {nm}", score(tm, src, ti, sw["sacrum_to_t3"], MOVES[nm]))

    print("--- contrast SECANT big-span: straight chord P1->P4  vs IMU sacrum_to_t3 ---")
    for nm in MOVES_ORDER:
        src = sec_flex if nm in ("前倾 flexion", "后仰 extension") else sec_lat
        prow(f"SECANT {nm}", score(tm, src, ti, sw["sacrum_to_t3"], MOVES[nm]))

    print("--- matched TANGENT big-span: local T3 tangent vs pelvis  vs IMU sacrum_to_t3 ---")
    for nm in MOVES_ORDER:
        src = bs_flex if nm in ("前倾 flexion", "后仰 extension") else bs_lat
        prow(f"TAN {nm}", score(tm, src, ti, sw["sacrum_to_t3"], MOVES[nm]))

    print("--- matched per-segment: chord Pi->Pj  vs the SAME-span IMU relation ---")
    for label, ((cf, cl), rel) in seg_chords.items():
        for nm in MOVES_ORDER:
            src = cf if nm in ("前倾 flexion", "后仰 extension") else cl
            prow(f"{label} | {nm}", score(tm, src, ti, sw[rel], MOVES[nm]))

    # ================= TWIST =================
    twL, twR = MOVES["左转 twist-L"], MOVES["右转 twist-R"]
    print("\n=== TWIST  vs IMU sacrum_to_sternum (signed). OLD anchor T0 vs matched anchor T4 ===")
    for aname in ("T0 (IJ+PX)/2 - spine4", "T4 XIP - sacrum(P1)"):
        axv = twist_anchors[aname]
        prow(f"{aname} | 左转", score(tm, axv, ti, tw, twL, signed=True))
        prow(f"{aname} | 右转", score(tm, axv, ti, tw, twR, signed=True))
        prow(f"{aname} | pooled", score(tm, axv, ti, tw, twL + twR, signed=True))

    print("\n--- (all twist anchors, pooled, for the anchor sweep) ---")
    for label, axv in twist_anchors.items():
        prow(label, score(tm, axv, ti, tw, twL + twR, signed=True))

    # ===== held-out calibration: is the gain a real CALIBRATION or 'copying the answer'? =====
    print("\n=== LEAVE-ONE-REP-OUT held-out calibration (gain fit on 2 reps, tested on the unseen rep) ===")
    print("    if held-out RMSE ~ gained RMSE above, the gain GENERALISES across reps (not circular).")
    lrow("前倾 flexion (matched)", loro(tm, bs_flex, ti, sw["sacrum_to_t3"], MOVES["前倾 flexion"]))
    lrow("左倾 lat-L (matched)", loro(tm, bs_lat, ti, sw["sacrum_to_t3"], MOVES["左倾 lat-L"]))
    lrow("右倾 lat-R (matched)", loro(tm, bs_lat, ti, sw["sacrum_to_t3"], MOVES["右倾 lat-R"]))
    lrow("后仰 extension (matched)", loro(tm, bs_flex, ti, sw["sacrum_to_t3"], MOVES["后仰 extension"]))
    lrow("扭转 pooled (T4 anchor)", loro(tm, twist_anchors["T4 XIP - sacrum(P1)"], ti, tw, twL + twR, signed=True))
    lrow("左转 twist-L (T4)", loro(tm, twist_anchors["T4 XIP - sacrum(P1)"], ti, tw, twL, signed=True))
    lrow("右转 twist-R (T4)", loro(tm, twist_anchors["T4 XIP - sacrum(P1)"], ti, tw, twR, signed=True))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
