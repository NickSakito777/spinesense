from __future__ import annotations

"""U/B-bracket-only analysis on the take-2 data (drop L/R clusters, keep just the markers that
sandwich each IMU above/below). Tests Fox's proposal: validation1-style, but with markers placed to
bracket each IMU tightly. Uses ONLY S?U/S?B per IMU -> a clean local point (midpoint) and a local
long-axis tangent (U-B). Frame-free (tilt from world vertical), so no pelvis-frame calibration needed.

Question: does tight bracketing beat validation1's anatomical-landmark markers, and does the bending
gain move toward 1?  Run with .venv/Scripts/python.exe.
"""

import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import five_imu_fusion as fiv
import validation3_cluster_orientation as v3

UB = {  # IMU -> (upper marker, lower marker)
    "sacrum": ("S1U", "S1B"), "t12": ("T12U", "T12B"), "t6": ("T6U", "T6B"),
    "t3": ("T3u", "T3B"), "sternum": ("SternumU", "SternumB"),
}
VERT = np.array([0.0, 1.0, 0.0])  # Motive world up


def tared(x, still):
    return v3.uw(x) - np.median(v3.uw(x)[still])


def main() -> int:
    tm, mk = v3.parse(v3.MOC); mk = {k: v3.fill(v) for k, v in mk.items()}; still = tm <= 8.0
    P = {n: v3.PRE + n for n in sum([list(v) for v in UB.values()], [])}
    mid = {imu: (mk[P[u]] + mk[P[b]]) / 2 for imu, (u, b) in UB.items()}      # IMU position
    tan = {imu: v3._u(mk[P[u]] - mk[P[b]]) for imu, (u, b) in UB.items()}     # local long-axis (up)

    def chord_lean(par, chi):  # tilt of the par->chi midpoint line from vertical (total bend magnitude)
        d = v3._u(mid[chi] - mid[par])
        return tared(np.degrees(np.arccos(np.clip(np.sum(d * VERT, axis=1), -1, 1))), still)

    def tangent_interangle(par, chi):  # change in angle between the two local long-axes = inter-seg bend
        return tared(np.degrees(np.arccos(np.clip(np.sum(tan[par] * tan[chi], axis=1), -1, 1))), still)

    bend_chord = chord_lean("sacrum", "t3")
    bend_tan = tangent_interangle("sacrum", "t3")

    common = dict(layout_preset="validation1", filter="vqf", **{r: i for i, r in v3.PM.items()})
    res = fiv.run_pipeline(v3.LOG, fiv.make_args(**common)); ti = res.t_s
    imu_sw = res.relations["sacrum_to_t3"].swing_deg

    v3.A, v3.B, qa = v3.align_swing(tm, bend_chord, ti, imu_sw)
    print(f"clock: t_imu = {v3.A:.6f}*t_mocap + {v3.B:.3f}  (align r={qa:.2f})")

    bw = [(tm[i], tm[j - 1]) for i, j in v3.runs(tm, bend_chord > 12)]
    print(f"bending active windows: {len(bw)}\n")

    print("=== BIG-SPAN bending, U/B brackets (take-2) vs IMU sacrum_to_t3 ===")
    v3.prow("midpoint-chord lean", v3.score(tm, bend_chord, ti, imu_sw, bw))
    v3.prow("U-B tangent inter-ang", v3.score(tm, bend_tan, ti, imu_sw, bw))

    print("\n=== per-segment, U-B tangent inter-angle vs same-span IMU relation ===")
    for par, chi, rel in [("sacrum", "t12", "sacrum_to_t12"), ("t12", "t6", "t12_to_t6"), ("t6", "t3", "t6_to_t3")]:
        sig = tangent_interangle(par, chi)
        sw = res.relations[rel].swing_deg
        seg_bw = [(tm[i], tm[j - 1]) for i, j in v3.runs(tm, sig > 6)]
        v3.prow(f"{rel}", v3.score(tm, sig, ti, sw, seg_bw))

    print("\n=== held-out (leave-one-rep-out), big-span U/B brackets ===")
    v3.lrow("midpoint-chord", v3.loro(tm, bend_chord, ti, imu_sw, bw))
    v3.lrow("U-B tangent", v3.loro(tm, bend_tan, ti, imu_sw, bw))
    print(f"\n(ref) midpoint-chord bend ptp={np.ptp(bend_chord):.0f}  IMU swing ptp={np.ptp(imu_sw):.0f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
