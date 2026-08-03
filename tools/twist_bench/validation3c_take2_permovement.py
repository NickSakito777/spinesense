from __future__ import annotations

"""Take-2 (cluster re-take) FULL per-movement table using ONLY U/B brackets per IMU.

Drops L/R (non-rigid, useless for orientation). Uses U/B midpoints as clean IMU points. Builds the
anatomical frame from the sternum-vs-spine direction (sternum is unambiguously anterior), avoiding
this take's inconsistent pelvis L/R marker labels. Reports all 6 movements: gain, r, ungained acc,
gained acc, and leave-one-rep-out held-out -- the Take-2 equivalent of the validation1 table.

Run with .venv/Scripts/python.exe.
"""

import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import five_imu_fusion as fiv
import validation3_cluster_orientation as v3

UB = {"sacrum": ("S1U", "S1B"), "t12": ("T12U", "T12B"), "t6": ("T6U", "T6B"),
      "t3": ("T3u", "T3B"), "sternum": ("SternumU", "SternumB")}
UPW = np.array([0.0, 1.0, 0.0])


def tared(x, still):
    return v3.uw(x) - np.median(v3.uw(x)[still])


def main() -> int:
    tm, mk = v3.parse(v3.MOC); mk = {k: v3.fill(v) for k, v in mk.items()}; still = tm <= 8.0
    mid = {imu: (mk[v3.PRE + u] + mk[v3.PRE + b]) / 2 for imu, (u, b) in UB.items()}

    # anatomical frame (fixed at neutral): up=world Y; anterior from sternum-vs-T6; ml = up x ap
    ap0 = np.mean((mid["sternum"] - mid["t6"])[still], axis=0)
    ap0 = ap0 - ap0[1] * UPW; ap = ap0 / np.linalg.norm(ap0)
    ml = np.cross(UPW, ap); ml = ml / np.linalg.norm(ml)
    print(f"anterior axis (world) = {ap.round(2)} ; ml = {ml.round(2)}")

    # big-span trunk bend (t3 vs sacrum) decomposed; twist from chest anterior-dir rotation
    c = v3._u(mid["t3"] - mid["sacrum"])
    flex = tared(np.degrees(np.arctan2(c @ ap, c @ UPW)), still)     # 前倾+/后仰-
    lat = tared(np.degrees(np.arctan2(c @ ml, c @ UPW)), still)      # 倾 +/-
    a = mid["sternum"] - mid["t6"]
    ax = tared(np.degrees(np.arctan2(a @ ml, a @ ap)), still)        # 转 +/-
    lean = tared(np.degrees(np.arccos(np.clip(c @ UPW, -1, 1))), still)  # frame-free, for alignment

    common = dict(layout_preset="validation1", filter="vqf", **{r: i for i, r in v3.PM.items()})
    res = fiv.run_pipeline(v3.LOG, fiv.make_args(**common)); ti = res.t_s
    imu_sw = res.relations["sacrum_to_t3"].swing_deg
    imu_tw = res.relations["sacrum_to_sternum"].twist_deg

    v3.A, v3.B, qa = v3.align_swing(tm, lean, ti, imu_sw)
    print(f"clock: t_imu = {v3.A:.6f}*t_mocap + {v3.B:.3f}  (align r={qa:.2f})")

    MOVES = {
        "前倾 flexion": v3.segment(tm, flex, +1, (lat, ax)),
        "后仰 extension": v3.segment(tm, flex, -1, (lat, ax), hi=8, lo=3, opurity=12),
        "左倾 lat-L": v3.segment(tm, lat, +1, (flex, ax)),
        "右倾 lat-R": v3.segment(tm, lat, -1, (flex, ax)),
        "左转 twist-L": v3.segment(tm, ax, +1, (flex, lat)),
        "右转 twist-R": v3.segment(tm, ax, -1, (flex, lat)),
    }
    print("bouts:", {k: len(v) for k, v in MOVES.items()})

    print("\n=== TAKE-2 per-movement (U/B brackets, big-span bend vs IMU sacrum_to_t3 / twist vs sternum) ===")
    for nm in ("前倾 flexion", "后仰 extension", "左倾 lat-L", "右倾 lat-R"):
        src = flex if "flex" in nm or "ext" in nm else lat
        v3.prow(nm, v3.score(tm, src, ti, imu_sw, MOVES[nm]))
    twb = MOVES["左转 twist-L"] + MOVES["右转 twist-R"]
    for nm, b in (("左转 twist-L", MOVES["左转 twist-L"]), ("右转 twist-R", MOVES["右转 twist-R"]), ("扭转 pooled", twb)):
        v3.prow(nm, v3.score(tm, ax, ti, imu_tw, b, signed=True))

    print("\n=== held-out (leave-one-rep-out) ===")
    for nm in ("前倾 flexion", "后仰 extension", "左倾 lat-L", "右倾 lat-R"):
        src = flex if "flex" in nm or "ext" in nm else lat
        v3.lrow(nm, v3.loro(tm, src, ti, imu_sw, MOVES[nm]))
    v3.lrow("扭转 pooled", v3.loro(tm, ax, ti, imu_tw, twb, signed=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
