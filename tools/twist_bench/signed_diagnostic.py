from __future__ import annotations

"""Signed MoCap + IMU diagnostic plot for manual sync review."""

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import five_imu_fusion as fiv
import sync_audit as sa
import twist_bench_fusion as pf
import validation3_cluster_orientation as v3


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--subject", default="04")
    p.add_argument("--manual-a", type=float, required=True)
    p.add_argument("--manual-b", type=float, required=True)
    p.add_argument("--chunk-s", type=float, default=120.0)
    p.add_argument("--filter", choices=("sflp", "vqf", "madgwick"), default="sflp")
    p.add_argument("--out-dir", type=Path, default=HERE / "plots" / "sync_audit")
    return p.parse_args()


def marker(mk: dict[str, np.ndarray], name: str) -> np.ndarray:
    key = "Trunk:" + name
    if key not in mk:
        raise SystemExit(f"Missing marker {key}")
    return mk[key]


def midpoint(mk: dict[str, np.ndarray], a: str, b: str) -> np.ndarray:
    return (marker(mk, a) + marker(mk, b)) / 2.0


def pelvis_basis(mk: dict[str, np.ndarray]):
    """Pelvis frame from the 4 PSIS markers (front + back).

    ml = right-left, ap = front-back (anterior), up = right x anterior (cranial),
    fwd re-orthogonalised. Returns ``(fwd, up, ml)``.
    """
    Lf, Rf = marker(mk, "LPSIS_F"), marker(mk, "RPSIS_F")
    Lb, Rb = marker(mk, "LPSIS_B"), marker(mk, "RPSIS_B")
    ml = v3._u((Rf + Rb) / 2.0 - (Lf + Lb) / 2.0)          # subject right - left
    ap = (Lf + Rf) / 2.0 - (Lb + Rb) / 2.0                 # front - back = anterior
    up = v3._u(np.cross(ml, ap))                           # right x anterior = up (cranial)
    fwd = v3._u(np.cross(up, ml))                          # re-orthogonalised anterior
    return fwd, up, ml


def first_marker(mk: dict[str, np.ndarray], names: tuple[str, ...]) -> np.ndarray:
    for name in names:
        key = "Trunk:" + name
        if key in mk:
            return mk[key]
    raise SystemExit(f"Missing all markers: {names}")


def mocap_signed(mocap: Path):
    tm, mk = sa.parse_motive_markers(mocap, gap_fill=True)
    fwd, up, ml = pelvis_basis(mk)
    still = tm <= 8.0
    upper = midpoint(mk, "C7 (2)", "T2")
    sternum = midpoint(mk, "JN", "XP")
    sacral = first_marker(mk, ("S2", "S1"))
    sacrum = (marker(mk, "L3") + sacral + marker(mk, "LPSIS_B") + marker(mk, "RPSIS_B")) / 4.0
    flex, lat = v3.chord_tilt(sacrum, upper, fwd, up, ml, still)
    axial = v3.axial(sternum, sacrum, fwd, ml, still)
    lat_dominant = (np.abs(lat) > 15.0) & (np.abs(flex) < 10.0)
    if int(np.sum(lat_dominant)) > 50:
        axial = axial - np.polyfit(lat[lat_dominant], axial[lat_dominant], 1)[0] * lat
    return tm, flex, lat, axial


def imu_signed_bend(rel) -> tuple[np.ndarray, np.ndarray]:
    d = pf.quat_rotate(rel.q_tared, np.array([0.0, 0.0, 1.0]))
    return (
        np.degrees(np.arctan2(d[:, 0], d[:, 2])),
        np.degrees(np.arctan2(d[:, 1], d[:, 2])),
    )


def diagnostic_bend_map(tm, flex, lat, axial, ti_mocap, bend_x, bend_y):
    pure_flex = (np.abs(axial) < 12.0) & (np.abs(flex) > 15.0) & (np.abs(lat) < 12.0)
    pure_lat = (np.abs(axial) < 12.0) & (np.abs(lat) > 15.0) & (np.abs(flex) < 12.0)
    pure = pure_flex | pure_lat
    if int(np.sum(pure)) < 20:
        return bend_x, bend_y, np.eye(2)
    x = np.column_stack([
        np.interp(tm[pure], ti_mocap, bend_x),
        np.interp(tm[pure], ti_mocap, bend_y),
    ])
    y = np.column_stack([flex[pure], lat[pure]])
    u, _, vt_ = np.linalg.svd(x.T @ y)
    r = u @ vt_
    x_all = np.column_stack([
        np.interp(tm, ti_mocap, bend_x),
        np.interp(tm, ti_mocap, bend_y),
    ])
    mapped_all = x_all @ r
    for col, target, mask in ((0, flex, pure_flex), (1, lat, pure_lat)):
        if int(np.sum(mask)) > 5 and np.corrcoef(mapped_all[mask, col], target[mask])[0, 1] < 0.0:
            r[:, col] *= -1.0
    mapped = np.column_stack([bend_x, bend_y]) @ r
    return mapped[:, 0], mapped[:, 1], r


def write_plot(path: Path, tm, flex, lat, axial, ti_mocap, bend_x, bend_y, bend_flex, bend_lat, twist,
               chunk_s: float, a: float, b: float, bend_map: np.ndarray) -> None:
    import matplotlib.pyplot as plt

    start = max(float(tm[0]), float(ti_mocap[0]))
    end = min(float(tm[-1]), float(ti_mocap[-1]))
    chunks = max(1, int(np.ceil((end - start) / chunk_s)))
    fig, axes = plt.subplots(chunks * 3, 1, figsize=(28, max(5, chunks * 5.8)), constrained_layout=True)
    axes = np.atleast_1d(axes)

    for k in range(chunks):
        lo = start + k * chunk_s
        hi = min(end, lo + chunk_s)
        g = np.arange(lo, hi, 0.03)

        ax0 = axes[3 * k]
        ax0.plot(g, np.interp(g, tm, flex), label="MoCap flex signed", lw=0.9)
        ax0.plot(g, np.interp(g, tm, lat), label="MoCap lateral signed", lw=0.9)
        ax0.plot(g, np.interp(g, tm, axial), label="MoCap axial signed", lw=0.9)
        ax0.axhline(0, color="black", lw=0.5, alpha=0.4)
        ax0.grid(True, alpha=0.2)
        ax0.set_ylabel(f"MoCap\n{lo:.0f}-{hi:.0f}s")
        if k == 0:
            ax0.set_title(
                f"Signed diagnostic: t_imu = {a:.6f} * t_mocap + {b:.3f} | bend map "
                f"[[{bend_map[0,0]:+.2f},{bend_map[0,1]:+.2f}], [{bend_map[1,0]:+.2f},{bend_map[1,1]:+.2f}]]"
            )
            ax0.legend(loc="upper right", ncol=3)

        ax1 = axes[3 * k + 1]
        ax1.plot(g, np.interp(g, ti_mocap, bend_flex), label="IMU bend mapped to flex", lw=0.9)
        ax1.plot(g, np.interp(g, ti_mocap, bend_lat), label="IMU bend mapped to lateral", lw=0.9)
        ax1.plot(g, np.interp(g, ti_mocap, twist), label="IMU -sacrum->sternum twist signed", lw=0.9)
        ax1.axhline(0, color="black", lw=0.5, alpha=0.4)
        ax1.grid(True, alpha=0.2)
        ax1.set_ylabel("IMU mapped deg")
        ax1.set_xlim(lo, hi)
        if k == 0:
            ax1.legend(loc="upper right", ncol=3)

        ax2 = axes[3 * k + 2]
        ax2.plot(g, np.interp(g, ti_mocap, bend_x), label="IMU raw bend x", lw=0.9)
        ax2.plot(g, np.interp(g, ti_mocap, bend_y), label="IMU raw bend y", lw=0.9)
        ax2.axhline(0, color="black", lw=0.5, alpha=0.4)
        ax2.grid(True, alpha=0.2)
        ax2.set_ylabel("IMU raw deg")
        ax2.set_xlim(lo, hi)
        ax0.set_xlim(lo, hi)
        if k == 0:
            ax2.legend(loc="upper right", ncol=2)
    axes[-1].set_xlabel("MoCap time (s)")
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    imu, mocap = sa.default_paths(args.subject)
    tm, flex, lat, axial = mocap_signed(mocap)
    res = fiv.run_pipeline(imu, fiv.make_args(layout_preset="t01", filter=args.filter))
    ti_mocap = (res.t_s - args.manual_b) / args.manual_a
    bend_x, bend_y = imu_signed_bend(res.relations["sacrum_to_upper"])
    bend_flex, bend_lat, bend_map = diagnostic_bend_map(tm, flex, lat, axial, ti_mocap, bend_x, bend_y)
    twist = -res.relations["sacrum_to_sternum"].twist_deg
    args.out_dir.mkdir(parents=True, exist_ok=True)
    out = args.out_dir / f"T{int(args.subject):02d}_P{int(args.subject):02d}_signed_diagnostic.png"
    write_plot(out, tm, flex, lat, axial, ti_mocap, bend_x, bend_y, bend_flex, bend_lat, twist,
               args.chunk_s, args.manual_a, args.manual_b, bend_map)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
