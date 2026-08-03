from __future__ import annotations

"""All-movement (6-class, signed) IMU-vs-MoCap accuracy for the Validation1 P01 pilot.

Supersedes the 3-class accuracy windows (flexion/lateral/twist) which silently dropped back-extension
and merged left/right. This tool:
  - sign-corrects MoCap to the SUBJECT-ANATOMICAL convention (anchored on forward flexion = physically +,
    plus the recorded protocol order): 前倾+ 后仰- | 左倾+ 右倾- | 左转+ 右转-.
    Empirically that means flex and lateral are negated vs mocap_adapter's raw output, axial unchanged.
  - AUTO-segments all 6 movements from the corrected signals (no hardcoded windows), 3 reps each.
  - per-bout LOCAL RELOCK (+-0.5 s xcorr) on top of the global affine clock map, fixing the residual
    drift that grows across rounds (esp. R3).
  - protocol-assisted local re-tare before each bout, reducing cross-round yaw offset without claiming
    absolute yaw recovery.
  - reports per-movement accuracy; twist both per-direction and pooled, fixed vs functional axis.

Run from tools/twist_bench:  .venv/Scripts/python.exe validation1_full_accuracy.py
Filter defaults to VQF and fails fast if the wheel is absent; use --filter madgwick for diagnostics only.
N=1 subject pilot; acc% = 1 - calRMSE/ROM (harsh for small-ROM movements -> read absolute RMSE too).
"""

import argparse
import csv
import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import five_imu_fusion as fiv
import mocap_adapter as mc

A, B = 1.002402, 61.889  # drift-corrected clock map t_imu = A*t_mocap + B
RAW = HERE / "data/twist_trial_20260626_170028.log"
MOC = HERE / "data/mocap/Testing_01(1).csv"
PM = {"IMU0": "sacrum", "IMU1": "t12", "IMU2": "t6", "IMU3": "t3", "IMU4": "sternum"}

MOVE_FLEX = "前倾 flexion"
MOVE_EXT = "后仰 extension"
MOVE_LAT_L = "左倾 lat-L"
MOVE_LAT_R = "右倾 lat-R"
MOVE_TWIST_L = "左转 twist-L"
MOVE_TWIST_R = "右转 twist-R"
BEND_MOVES = (MOVE_FLEX, MOVE_EXT, MOVE_LAT_L, MOVE_LAT_R)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--filter", choices=("vqf", "madgwick", "sflp"), default="vqf",
                   help="Orientation filter. VQF is the thesis mainline; SFLP is on-chip cross-check; Madgwick is diagnostic only.")
    p.add_argument("--allow-fallback", action="store_true",
                   help="Allow VQF import failure to fall back to Madgwick. Disabled by default for headline runs.")
    p.add_argument("--max-relock-s", type=float, default=0.5,
                   help="Local cross-correlation search window in seconds for the diagnostic relock table.")
    p.add_argument("--local-retare-s", type=float, default=1.0,
                   help="Seconds before each bout used as a protocol-assisted local zero. Default: 1.0.")
    p.add_argument("--local-retare-gap-s", type=float, default=0.2,
                   help="Gap between local zero window and bout start. Default: 0.2 s.")
    p.add_argument("--no-local-retare", action="store_true",
                   help="Disable local per-bout zeroing; use only the initial neutral tare.")
    p.add_argument("--out-aligned-csv", type=Path,
                   help="Write 100 Hz global-affine aligned MoCap/IMU channels for frame-level audit.")
    p.add_argument("--imu", type=Path, default=RAW, help="IMU raw(+SFLP) serial log. Default: the P01 pilot log.")
    p.add_argument("--mocap", type=Path, default=MOC, help="OptiTrack/Motive marker CSV. Default: the P01 pilot CSV.")
    p.add_argument("--clock-a", type=float, default=A,
                   help="Clock-map slope a in t_imu = a*t_mocap + b. Re-derive per recording with align_drift_regress.py.")
    p.add_argument("--clock-b", type=float, default=B,
                   help="Clock-map offset b in t_imu = a*t_mocap + b (seconds the IMU started before MoCap).")
    return p


def resolve_filter(args: argparse.Namespace) -> str:
    if args.filter == "sflp":
        return args.filter
    if args.filter != "vqf":
        print("[warn] filter=madgwick is diagnostic; do not use it for thesis headline VQF results.")
        return args.filter
    try:
        import vqf  # noqa: F401
    except Exception as exc:
        if args.allow_fallback:
            print(f"[warn] vqf unavailable ({exc!r}) -> filter=madgwick")
            return "madgwick"
        raise SystemExit(
            "VQF is unavailable in this interpreter. Run with "
            ".venv/Scripts/python.exe, or pass --filter madgwick only for a diagnostic run."
        ) from exc
    return "vqf"


def main() -> int:
    args = build_parser().parse_args()
    flt = resolve_filter(args)
    global A, B
    A, B = args.clock_a, args.clock_b   # clock map t_imu = A*t_mocap + B (per-recording; see align_drift_regress.py)
    print(f"inputs: imu={args.imu.name}  mocap={args.mocap.name}  clock: t_imu = {A:.6f}*t_mocap + {B:.3f}")

    # ---- MoCap: sign-correct to subject-anatomical convention + tare on neutral ----
    tm, mk, _ = mc.parse_motive_csv(args.mocap); tm = tm - tm[0]
    mk = {k: mc._fill_gaps(v) for k, v in mk.items()}
    ang = mc.trunk_angles(mk); st = tm <= 8
    uw = lambda x: np.degrees(np.unwrap(np.radians(x)))  # noqa: E731
    flex = -(uw(ang["flex"]) - np.median(uw(ang["flex"])[st]))        # 前倾+ 后仰-
    lat = -(uw(ang["lateral"]) - np.median(uw(ang["lateral"])[st]))   # 左倾+ 右倾-
    ax = (uw(ang["axial"]) - np.median(uw(ang["axial"])[st]))         # 左转+ 右转-

    # ---- auto-segment 6 signed movements (full hump, purity-checked) ----
    def runs(mask, min_dur=0.4):
        d = np.diff(mask.astype(int)); s = np.where(d == 1)[0]+1; e = np.where(d == -1)[0]+1
        if mask[0]: s = np.r_[0, s]
        if mask[-1]: e = np.r_[e, len(mask)]
        return [(i, j) for i, j in zip(s, e) if tm[j-1]-tm[i] >= min_dur]

    def segment(sig, sign, others, hi=20, lo=5, opurity=18, frac=0.30):
        s = sign*sig; out = []
        for i, j in runs(s > lo):
            pk = float(s[i:j].max())
            if pk < hi: continue
            k = i + int(np.argmax(s[i:j]))
            if abs(others[0][k]) > opurity or abs(others[1][k]) > opurity: continue
            thr = frac*pk
            a = k
            while a > i and s[a] > thr: a -= 1
            b = k
            while b < j-1 and s[b] > thr: b += 1
            out.append((tm[a], tm[b], float(sig[k])))
        return out

    MOVES = {
        "前倾 flexion":   segment(flex, +1, (lat, ax)),
        "后仰 extension": segment(flex, -1, (lat, ax), hi=8, lo=3, opurity=12),
        "左倾 lat-L":     segment(lat,  +1, (flex, ax)),
        "右倾 lat-R":     segment(lat,  -1, (flex, ax)),
        "左转 twist-L":   segment(ax,   +1, (flex, lat)),
        "右转 twist-R":   segment(ax,   -1, (flex, lat)),
    }
    print(f"=== auto-detected signed bouts (filter={flt}, MoCap s : peak deg) ===")
    for nm, bts in MOVES.items():
        print(f"  {nm:16s}: " + "  ".join(f"{a:.0f}-{b:.0f}s({p:+.0f})" for a, b, p in bts))

    # ---- IMU pipeline: fixed + functional (calib = the bend bouts) ----
    calib = [[A*a+B, A*b+B] for nm in ("前倾 flexion", "左倾 lat-L", "右倾 lat-R") for a, b, _ in MOVES[nm]]
    common = dict(layout_preset="validation1", filter=flt, **{r: i for i, r in PM.items()})
    fix = fiv.run_pipeline(args.imu, fiv.make_args(**common))
    fn = fiv.run_pipeline(args.imu, fiv.make_args(twist_axis_mode="functional", twist_calib_window=calib, **common))
    ti = fix.t_s
    sw_t3 = fix.relations["sacrum_to_t3"].swing_deg
    tw_fix = fix.relations["sacrum_to_sternum"].twist_deg
    tw_fn = fn.relations["sacrum_to_sternum"].twist_deg

    # ---- per-bout local relock + calibrated accuracy ----
    def relock(msig, isig, win, maxlag=0.5):
        g = np.arange(win[0], win[1], 0.01); M = np.interp(g, tm, msig)
        best_abs, best_lag = -np.inf, 0.0
        for lag in np.arange(-maxlag, maxlag+1e-9, 0.01):
            I = np.interp(A*g+B+lag, ti, isig)
            if np.std(I) < 1e-6 or np.std(M) < 1e-6: continue
            c = np.corrcoef(I, M)[0, 1]
            if abs(c) > best_abs:
                best_abs, best_lag = abs(c), float(lag)
        return best_lag

    def local_offsets(msig, isig, a, lag):
        if args.no_local_retare:
            return 0.0, 0.0
        hi = a - args.local_retare_gap_s
        lo = hi - args.local_retare_s
        if hi <= tm[0]:
            return 0.0, 0.0
        g0 = np.arange(max(tm[0], lo), hi, 0.01)
        if len(g0) < 5:
            return 0.0, 0.0
        return (
            float(np.mean(np.interp(g0, tm, msig))),
            float(np.mean(np.interp(A*g0+B+lag, ti, isig))),
        )

    def acc(msig, isig, bouts, signed=False):
        Iall, Mall = [], []
        for a, b, _ in bouts:
            lag = relock(msig if signed else np.abs(msig), isig, (a, b), args.max_relock_s)
            m0, i0 = local_offsets(msig, isig, a, lag)
            g = np.arange(a, b, 0.01)
            M = np.interp(g, tm, msig) - m0
            I = np.interp(A*g+B+lag, ti, isig) - i0
            if not signed: M = np.abs(M)
            Iall.append(I); Mall.append(M)
        I = np.concatenate(Iall); M = np.concatenate(Mall)
        aa, bb = np.polyfit(I, M, 1)
        rmse = float(np.sqrt(np.mean((M-(aa*I+bb))**2)))
        return float(np.corrcoef(I, M)[0, 1]), rmse, max(0., 1-rmse/max(np.ptp(M), 1e-6))*100, float(np.ptp(M))

    retare_txt = "local re-tare ON" if not args.no_local_retare else "local re-tare OFF"
    print(f"\n=== ALL-MOVEMENT ACCURACY (signed, per-bout relocked, calibrated, {retare_txt}) ===")
    print(f"{'movement':16s} {'IMU chan':12s} {'r':>6} {'RMSE':>6} {'acc':>5} {'ROM':>5}")
    for nm in ("前倾 flexion", "后仰 extension", "左倾 lat-L", "右倾 lat-R"):
        src = flex if nm in ("前倾 flexion", "后仰 extension") else lat
        r, rmse, a_, rom = acc(src, sw_t3, MOVES[nm])
        print(f"{nm:16s} {'swing t3':12s} {r:6.2f} {rmse:6.1f} {a_:4.0f}% {rom:5.0f}")
    twL, twR = MOVES["左转 twist-L"], MOVES["右转 twist-R"]
    for nm, bts in (("左转 twist-L", twL), ("右转 twist-R", twR), ("扭转 pooled", twL+twR)):
        for an, tw in (("fixed", tw_fix), ("func", tw_fn)):
            r, rmse, a_, rom = acc(ax, tw, bts, signed=True)
            print(f"{nm:16s} {('twist '+an):12s} {r:6.2f} {rmse:6.1f} {a_:4.0f}% {rom:5.0f}")
    r, rmse, a_, rom = acc(ax, tw_fix, [(65.78, 88.05, 36.0)], signed=True)
    print(f"{'  twist R1-only':16s} {'twist fixed':12s} {r:6.2f} {rmse:6.1f} {a_:4.0f}% {rom:5.0f}  (anchor vs prior 93%)")

    print("\n=== CROSS-TALK (IMU twist during pure bending; true axial ~0, lower=better) ===")
    for nm in ("前倾 flexion", "后仰 extension"):
        f_, n_, m_ = [], [], []
        for a, b, _ in MOVES[nm]:
            g = np.arange(a, b, 0.01)
            f_.append(np.interp(A*g+B, ti, tw_fix)); n_.append(np.interp(A*g+B, ti, tw_fn)); m_.append(np.interp(g, tm, ax))
        fr = np.sqrt(np.mean(np.concatenate(f_)**2)); nr = np.sqrt(np.mean(np.concatenate(n_)**2)); mr = np.sqrt(np.mean(np.concatenate(m_)**2))
        print(f"  {nm:16s}: fixed twist RMS={fr:4.1f}  functional={nr:4.1f}  (true axial RMS={mr:.1f})")
    out_aligned = args.out_aligned_csv or (HERE / "plots" / f"validation1_full_accuracy_{flt}_aligned.csv")
    write_aligned_csv(out_aligned, MOVES, tm, flex, lat, ax, ti, sw_t3, tw_fix, tw_fn)
    print(f"\nwrote aligned frame table: {out_aligned}")
    return 0


def write_aligned_csv(path: Path, moves: dict, tm, flex, lat, ax, ti, sw_t3, tw_fix, tw_fn) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    t0 = max(float(tm[0]), float((ti[0] - B) / A))
    t1 = min(float(tm[-1]), float((ti[-1] - B) / A))
    grid = np.arange(t0, t1, 0.01)

    def label(t: float) -> str:
        for name, bouts in moves.items():
            for a, b, _ in bouts:
                if a <= t <= b:
                    return name
        return ""

    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow([
            "t_mocap_s", "t_imu_global_s", "movement",
            "mocap_flex_deg", "mocap_lateral_deg", "mocap_axial_deg",
            "imu_swing_sacrum_to_t3_deg", "imu_twist_sternum_fixed_deg", "imu_twist_sternum_functional_deg",
        ])
        for t in grid:
            t_imu = A * t + B
            writer.writerow([
                f"{t:.4f}", f"{t_imu:.4f}", label(float(t)),
                f"{np.interp(t, tm, flex):.6f}",
                f"{np.interp(t, tm, lat):.6f}",
                f"{np.interp(t, tm, ax):.6f}",
                f"{np.interp(t_imu, ti, sw_t3):.6f}",
                f"{np.interp(t_imu, ti, tw_fix):.6f}",
                f"{np.interp(t_imu, ti, tw_fn):.6f}",
            ])


if __name__ == "__main__":
    raise SystemExit(main())
