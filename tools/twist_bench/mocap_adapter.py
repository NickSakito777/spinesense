from __future__ import annotations

"""MoCap (OptiTrack/Motive marker CSV) -> trunk thorax-relative-to-pelvis angles.

Builds an ISB-style pelvis frame (ASIS/PSIS/S2) and thorax frame (jugular notch + xiphoid + T-spine),
then expresses the thorax forward axis in the pelvis frame to get the trunk's transverse-plane AXIAL
rotation (the twist ground truth), plus sagittal (flexion+/extension-) for context. Marker-based, so
no rigid-body export needed. Motive is Y-up, meters, global space.

Output: <stem>_mocap_trunk.csv  (time_s, axial_deg, flex_deg) tared on the neutral window.
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

# Motive marker names (Trunk: prefix in this export).
MK = {
    "IJ": "Trunk:JugularNotch", "PX": "Trunk:Xiphoid",
    "T1": "Trunk:T1.2", "T3": "Trunk:T3", "T7": "Trunk:T7", "T12": "Trunk:T12",
    "LASIS": "Trunk:LASIS-F", "RASIS": "Trunk:RASIS-F",
    "LPSIS": "Trunk:LPSIS-B", "RPSIS": "Trunk:RPSIS-B", "S2": "Trunk:S2",
}


def _is_int(s: str) -> bool:
    try:
        int(s)
        return True
    except ValueError:
        return False


def parse_motive_csv(path: Path) -> tuple[np.ndarray, dict[str, np.ndarray], dict[str, float]]:
    rows = list(csv.reader(path.open(encoding="utf-8", errors="replace")))
    name_row = next(r for r in rows if len(r) > 1 and r[1] == "Name")
    data_rows = [r for r in rows if r and _is_int(r[0])]
    # marker name -> its 3 consecutive column indices (X,Y,Z)
    cols: dict[str, list[int]] = {}
    for j in range(2, len(name_row)):
        nm = name_row[j]
        if nm:
            cols.setdefault(nm, []).append(j)

    n = len(data_rows)
    ncol = max(len(r) for r in data_rows)
    mat = np.full((n, ncol), np.nan)
    for k, r in enumerate(data_rows):
        for j, val in enumerate(r):
            if val:
                try:
                    mat[k, j] = float(val)
                except ValueError:
                    pass
    t = mat[:, 1].copy()
    markers: dict[str, np.ndarray] = {}
    nan_frac: dict[str, float] = {}
    for key, full in MK.items():
        if full in cols and len(cols[full]) >= 3:
            xyz = mat[:, cols[full][:3]]
            markers[key] = xyz
            nan_frac[key] = float(np.mean(~np.isfinite(xyz).all(axis=1)))
        else:
            markers[key] = np.full((n, 3), np.nan)
            nan_frac[key] = 1.0
    return t, markers, nan_frac


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.where(n == 0, np.nan, n)


def _fill_gaps(a: np.ndarray) -> np.ndarray:
    """Linearly interpolate short marker-occlusion gaps (NaN) along time, per axis. Without this the
    few-percent scattered occlusions poison downstream cross products / unwrap."""
    a = a.astype(float).copy()
    idx = np.arange(a.shape[0])
    for c in range(a.shape[1]):
        good = np.isfinite(a[:, c])
        if good.sum() >= 2:
            a[:, c] = np.interp(idx, idx[good], a[good, c])
    return a


def trunk_angles(markers: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    m = markers
    # --- pelvis frame: fwd (anterior), up (cranial), ml (subject right) ---
    asis_mid = (m["LASIS"] + m["RASIS"]) / 2.0
    psis_mid = (m["LPSIS"] + m["RPSIS"]) / 2.0
    ml_p = _unit(m["RASIS"] - m["LASIS"])           # L->R = subject right
    up_p = _unit(np.cross(ml_p, asis_mid - psis_mid))  # right x anterior = up
    fwd_p = _unit(np.cross(up_p, ml_p))             # re-orthogonalize anterior

    # --- thorax frame from midline markers + AP offset ---
    up_t = _unit(((m["IJ"] + m["T1"]) / 2.0) - ((m["PX"] + m["T12"]) / 2.0))  # cranial
    spine = (m["T1"] + m["T3"] + m["T7"] + m["T12"]) / 4.0
    stern = (m["IJ"] + m["PX"]) / 2.0
    fwd_raw = stern - spine                          # posterior->anterior
    fwd_t = _unit(fwd_raw - np.sum(fwd_raw * up_t, axis=1, keepdims=True) * up_t)

    # thorax UP axis in pelvis basis -> sagittal (flexion+) and frontal (lateral) tilt
    u_fwd = np.sum(up_t * fwd_p, axis=1)
    u_up = np.sum(up_t * up_p, axis=1)
    u_ml = np.sum(up_t * ml_p, axis=1)
    flex = np.degrees(np.arctan2(u_fwd, u_up))       # thorax up tilts forward = + flexion (- extension)
    lateral = np.degrees(np.arctan2(u_ml, u_up))     # side tilt = lateral bending
    # thorax FWD axis in pelvis basis -> transverse-plane AXIAL rotation (twist)
    f_fwd = np.sum(fwd_t * fwd_p, axis=1)
    f_ml = np.sum(fwd_t * ml_p, axis=1)
    axial = np.degrees(np.arctan2(f_ml, f_fwd))
    return {"axial": axial, "flex": flex, "lateral": lateral, "up_p": up_p}


def _tare(x: np.ndarray, still: np.ndarray) -> np.ndarray:
    ref = np.nanmedian(x[still]) if np.any(still) else np.nanmedian(x[:50])
    return x - ref


def main() -> int:
    ap = argparse.ArgumentParser(description="Motive marker CSV -> thorax-vs-pelvis trunk angles.")
    ap.add_argument("--input", type=Path, required=True, help="Motive marker-export CSV.")
    ap.add_argument("--neutral-s", type=float, default=8.0, help="Neutral window (s) for tare.")
    ap.add_argument("--out", type=Path, help="Output CSV (default plots/<stem>_mocap_trunk.csv).")
    args = ap.parse_args()

    t, markers, nan_frac = parse_motive_csv(args.input)
    t = t - t[0]
    ts_med = float(np.median(np.diff(t)))
    print(f"MoCap {args.input.name}: {len(t)} frames, {t[-1]:.1f}s, ~{1/ts_med:.0f} Hz")
    print("marker NaN fraction (occlusion):", ", ".join(f"{k}={v:.2f}" for k, v in nan_frac.items()))

    markers = {k: _fill_gaps(v) for k, v in markers.items()}  # fill occlusion gaps before frame math
    ang = trunk_angles(markers)
    still = t <= args.neutral_s
    axial = np.degrees(np.unwrap(np.radians(_tare(ang["axial"], still))))
    flex = _tare(ang["flex"], still)
    lateral = _tare(ang["lateral"], still)

    print(f"\ntrunk AXIAL (twist ground truth): range {np.ptp(axial):.1f} deg  [{axial.min():.1f} .. {axial.max():.1f}]")
    print(f"trunk FLEXION(+)/EXT(-):          range {np.ptp(flex):.1f} deg  [{flex.min():.1f} .. {flex.max():.1f}]")
    print(f"trunk LATERAL bend:               range {np.ptp(lateral):.1f} deg  [{lateral.min():.1f} .. {lateral.max():.1f}]")

    out = args.out or (Path(__file__).resolve().parent / "plots" / f"{args.input.stem}_mocap_trunk.csv")
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["time_s", "axial_deg", "flex_deg", "lateral_deg"])
        for i in range(len(t)):
            w.writerow([f"{t[i]:.4f}", f"{axial[i]:.4f}", f"{flex[i]:.4f}", f"{lateral[i]:.4f}"])
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
