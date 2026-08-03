from __future__ import annotations

"""IMU/MoCap sync audit before accuracy analysis.

This does not edit raw data. It compares a generic IMU gyro-activity envelope
against a generic MoCap marker-speed envelope, then writes a small JSON + PNG
that can be manually adjusted with --manual-b.

Example:
  .venv/Scripts/python.exe sync_audit.py --subject 03
  .venv/Scripts/python.exe sync_audit.py --subject 03 --manual-b 42.5
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import twist_bench_v0 as v0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--subject", default="03", help="Subject number, e.g. 03 for T03/P03.")
    p.add_argument("--imu", type=Path, help="IMU .log path. Defaults to data/sessions/<id>.log.")
    p.add_argument("--mocap", type=Path, help="MoCap .csv path. Defaults to data/mocap/Pxx_all.csv.")
    p.add_argument("--manual-a", type=float, default=1.0, help="Manual clock slope for t_imu = a*t_mocap + b.")
    p.add_argument("--manual-b", type=float, help="Manual offset b: IMU seconds at MoCap t=0.")
    p.add_argument("--b-min", type=float, default=-120.0)
    p.add_argument("--b-max", type=float, default=240.0)
    p.add_argument("--neutral-keep-s", type=float, default=8.0, help="Recommended IMU seconds to keep before overlap.")
    p.add_argument("--crop-mocap-start", type=float, help="Override clean/analysis MoCap start time after sync.")
    p.add_argument("--crop-mocap-end", type=float, help="Override clean/analysis MoCap end time after sync.")
    p.add_argument("--out-dir", type=Path, default=HERE / "plots" / "sync_audit")
    p.add_argument("--write-clean", action="store_true", help="Write aligned/cropped derived CSVs under data_clean/.")
    p.add_argument("--clean-dir", type=Path, default=HERE / "data_clean")
    p.add_argument("--detail-chunk-s", type=float, default=0.0, help="Also write a stacked detail plot split into N-second chunks.")
    return p.parse_args()


def default_paths(subject: str) -> tuple[Path, Path]:
    s = f"{int(subject):02d}"
    return (
        HERE / "data" / "sessions" / f"{s}.log",
        HERE / "data" / "mocap" / f"P{s}_all.csv",
    )


def smooth(t: np.ndarray, x: np.ndarray, win_s: float = 0.7) -> np.ndarray:
    if len(t) < 3:
        return x
    dt = float(np.nanmedian(np.diff(t)))
    w = max(1, int(win_s / max(dt, 1e-6)))
    return np.convolve(x, np.ones(w) / w, mode="same")


def zscore(x: np.ndarray) -> np.ndarray:
    x = np.asarray(x, dtype=float)
    return (x - np.nanmean(x)) / (np.nanstd(x) + 1e-9)


def imu_activity(path: Path) -> tuple[np.ndarray, np.ndarray]:
    text = path.read_text(encoding="utf-8", errors="replace")
    recs = v0.parse_serial_text(text) or v0.parse_long_table_rows(v0.read_dict_rows(text))
    if not recs:
        raise SystemExit(f"No IMU rows parsed from {path}")

    by_t: dict[float, list[float]] = {}
    for r in recs:
        mag = (r.gx_dps * r.gx_dps + r.gy_dps * r.gy_dps + r.gz_dps * r.gz_dps) ** 0.5
        by_t.setdefault(r.t_s, []).append(mag)
    t = np.array(sorted(by_t), dtype=float)
    a = np.array([float(np.mean(by_t[x])) for x in t], dtype=float)
    t -= t[0]
    return t, smooth(t, a)


def parse_motive_markers(path: Path, gap_fill: bool = True) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    rows = list(csv.reader(path.open(encoding="utf-8", errors="replace")))
    name_row = next(r for r in rows if len(r) > 1 and r[1] == "Name")
    data = [r for r in rows if r and r[0].isdigit()]
    ncol = max(len(r) for r in data)
    mat = np.full((len(data), ncol), np.nan)
    for i, row in enumerate(data):
        for j, val in enumerate(row):
            if not val:
                continue
            try:
                mat[i, j] = float(val)
            except ValueError:
                pass

    cols: dict[str, list[int]] = {}
    for j in range(2, len(name_row)):
        if name_row[j]:
            cols.setdefault(name_row[j], []).append(j)

    t = mat[:, 1].copy()
    t -= t[0]
    markers: dict[str, np.ndarray] = {}
    for name, idx in cols.items():
        if len(idx) >= 3:
            xyz = mat[:, idx[:3]]
            markers[name] = fill_gaps(xyz) if gap_fill else xyz
    return t, markers


def fill_gaps(x: np.ndarray) -> np.ndarray:
    x = x.astype(float).copy()
    idx = np.arange(x.shape[0])
    for c in range(x.shape[1]):
        good = np.isfinite(x[:, c])
        if good.sum() >= 2:
            x[:, c] = np.interp(idx, idx[good], x[good, c])
    return x


def mocap_activity(path: Path) -> tuple[np.ndarray, np.ndarray, int]:
    t, markers = parse_motive_markers(path)
    usable = [v for v in markers.values() if np.isfinite(v).any()]
    if not usable:
        raise SystemExit(f"No marker positions parsed from {path}")

    # Median marker speed is marker-name agnostic and survives S1/S2/T7/T8 naming changes.
    speeds = []
    for xyz in usable:
        vel = np.gradient(xyz, t, axis=0)
        speeds.append(np.linalg.norm(vel, axis=1))
    a = np.nanmedian(np.vstack(speeds), axis=0)
    return t, smooth(t, a), len(usable)


def corr_for(ti: np.ndarray, ia: np.ndarray, tm: np.ndarray, ma: np.ndarray, a: float, b: float) -> float:
    g0 = max(tm[0], (ti[0] - b) / a)
    g1 = min(tm[-1], (ti[-1] - b) / a)
    if g1 - g0 < 30.0:
        return -2.0
    g = np.arange(g0, g1, 0.05)
    i = np.interp(a * g + b, ti, ia)
    m = np.interp(g, tm, ma)
    if np.std(i) < 1e-6 or np.std(m) < 1e-6:
        return -2.0
    return float(np.corrcoef(i, m)[0, 1])


def auto_align(ti: np.ndarray, ia: np.ndarray, tm: np.ndarray, ma: np.ndarray, b_min: float, b_max: float):
    coarse = []
    for b in np.arange(b_min, b_max + 1e-9, 0.5):
        coarse.append((corr_for(ti, ia, tm, ma, 1.0, b), 1.0, float(b)))
    coarse.sort(reverse=True)
    b0 = coarse[0][2]

    best = coarse[0]
    for a in np.arange(0.994, 1.006 + 1e-12, 0.00025):
        for b in np.arange(b0 - 3.0, b0 + 3.0 + 1e-12, 0.05):
            c = corr_for(ti, ia, tm, ma, float(a), float(b))
            if c > best[0]:
                best = (c, float(a), float(b))
    return best, coarse[:10]


def overlap(ti: np.ndarray, tm: np.ndarray, a: float, b: float) -> tuple[float, float]:
    return max(tm[0], (ti[0] - b) / a), min(tm[-1], (ti[-1] - b) / a)


def write_plot(path: Path, ti, ia, tm, ma, a: float, b: float, coarse: list[tuple[float, float, float]]) -> None:
    import matplotlib.pyplot as plt

    g0, g1 = overlap(ti, tm, a, b)
    g = np.arange(g0, g1, 0.05)
    im = np.interp(a * g + b, ti, zscore(ia))
    mo = np.interp(g, tm, zscore(ma))

    fig, ax = plt.subplots(2, 1, figsize=(14, 7), constrained_layout=True)
    ax[0].plot(g, mo, label="MoCap marker speed", lw=1.0)
    ax[0].plot(g, im, label="IMU gyro activity mapped to MoCap time", lw=1.0, alpha=0.85)
    ax[0].set_title(f"Overlay: t_imu = {a:.6f} * t_mocap + {b:.3f}")
    ax[0].set_xlabel("MoCap time (s)")
    ax[0].set_ylabel("z-scored activity")
    ax[0].legend(loc="upper right")

    bs = [x[2] for x in coarse]
    cs = [x[0] for x in coarse]
    ax[1].bar(range(len(bs)), cs)
    ax[1].set_xticks(range(len(bs)), [f"{x:.1f}" for x in bs], rotation=45)
    ax[1].set_title("Top coarse offsets b (A fixed at 1.0)")
    ax[1].set_xlabel("b = IMU seconds at MoCap t=0")
    ax[1].set_ylabel("corr")
    fig.savefig(path, dpi=160)
    plt.close(fig)


def write_detail_plot(path: Path, ti, ia, tm, ma, a: float, b: float, chunk_s: float) -> None:
    import matplotlib.pyplot as plt

    g0, g1 = overlap(ti, tm, a, b)
    n = max(1, int(np.ceil((g1 - g0) / chunk_s)))
    fig, axes = plt.subplots(n, 1, figsize=(22, max(2.2, 2.0 * n)), constrained_layout=True)
    if n == 1:
        axes = [axes]

    for k, ax in enumerate(axes):
        lo = g0 + k * chunk_s
        hi = min(g1, lo + chunk_s)
        g = np.arange(lo, hi, 0.03)
        im = np.interp(a * g + b, ti, zscore(ia))
        mo = np.interp(g, tm, zscore(ma))
        ax.plot(g, mo, label="MoCap", lw=0.9)
        ax.plot(g, im, label="IMU", lw=0.9, alpha=0.85)
        ax.set_xlim(lo, hi)
        ax.set_ylim(-2.5, 6.0)
        ax.grid(True, alpha=0.2)
        ax.set_ylabel(f"{lo:.0f}-{hi:.0f}s")
        if k == 0:
            ax.set_title(f"Detailed sync overlay: t_imu = {a:.6f} * t_mocap + {b:.3f}")
            ax.legend(loc="upper right")
    axes[-1].set_xlabel("MoCap time (s)")
    fig.savefig(path, dpi=140)
    plt.close(fig)


def write_clean_outputs(
    clean_dir: Path,
    rec: dict[str, object],
    imu_path: Path,
    mocap_path: Path,
    a: float,
    b: float,
    subject: str,
) -> dict[str, object]:
    out_dir = clean_dir / f"T{subject}_P{subject}"
    out_dir.mkdir(parents=True, exist_ok=True)
    overlap_rec = rec["overlap_s"]
    crop_rec = rec["recommended_crop_s"]
    imu_start = float(crop_rec["imu_start_keep_neutral"])
    imu_end = float(crop_rec["imu_end"])
    mocap_start = float(crop_rec["mocap_start"])
    mocap_end = float(crop_rec["mocap_end"])

    imu_out = out_dir / f"T{subject}_imu_aligned.csv"
    mocap_out = out_dir / f"P{subject}_mocap_markers_aligned.csv"
    manifest_out = out_dir / "sync_manifest.json"

    imu_in_rows, imu_out_rows, first_t = write_clean_imu(imu_path, imu_out, a, b, imu_start, imu_end)
    mocap_in_rows, mocap_out_rows, marker_count = write_clean_mocap(mocap_path, mocap_out, mocap_start, mocap_end)

    manifest = {
        **rec,
        "clean_outputs": {
            "folder": str(out_dir),
            "imu_aligned_csv": str(imu_out),
            "mocap_markers_aligned_csv": str(mocap_out),
            "manifest": str(manifest_out),
        },
        "clean_modifications": {
            "raw_files_untouched": True,
            "imu": {
                "source_first_timestamp_s": first_t,
                "crop_original_relative_s": [imu_start, imu_end],
                "rows_in": imu_in_rows,
                "rows_out": imu_out_rows,
                "added_columns": ["mocap_time_s", "imu_rel_s", "orig_t_ms"],
                "value_changes": "sensor values copied as-is; timestamps are converted to the shared MoCap time base",
            },
            "mocap": {
                "crop_mocap_s": [mocap_start, mocap_end],
                "rows_in": mocap_in_rows,
                "rows_out": mocap_out_rows,
                "markers": marker_count,
                "gap_filling": False,
                "value_changes": "marker coordinates copied as-is; Motive header is flattened to one CSV row",
            },
            "alignment": {
                "formula": "mocap_time_s = (imu_rel_s - b) / a",
                "inverse_formula": "imu_rel_s = a*mocap_time_s + b",
                "a": a,
                "b": b,
                "overlap_mocap_s": [float(overlap_rec["mocap_start"]), float(overlap_rec["mocap_end"])],
                "overlap_imu_rel_s": [float(overlap_rec["imu_start"]), float(overlap_rec["imu_end"])],
            },
        },
    }
    manifest_out.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def write_clean_imu(path: Path, out: Path, a: float, b: float, start_s: float, end_s: float) -> tuple[int, int, float]:
    parsed: list[tuple[float, list[str], list[float], list[float | str]]] = []
    with path.open(encoding="utf-8", errors="replace") as fh:
        for line in fh:
            parts = line.strip().split()
            if len(parts) < 10 or not parts[1].upper().startswith("IMU"):
                continue
            try:
                t_ms = float(parts[0])
                vals = [float(x) for x in parts[4:10]]
                quat = [float(x) for x in parts[10:14]] if len(parts) >= 14 else ["", "", "", ""]
            except ValueError:
                continue
            parsed.append((t_ms / 1000.0, parts, vals, quat))

    if not parsed:
        raise SystemExit(f"No IMU rows parsed from {path}")

    first_t = min(row[0] for row in parsed)
    rows: list[list[str]] = []
    for t_s, parts, vals, quat in sorted(parsed, key=lambda row: (row[0], row[1][1])):
        rel = t_s - first_t
        if rel < start_s or rel > end_s:
            continue
        mocap_t = (rel - b) / a
        rows.append([
            f"{mocap_t:.6f}", f"{rel:.6f}", f"{t_s * 1000.0:.3f}",
            parts[1], parts[2], parts[3],
            *[f"{x:.6f}" for x in vals],
            *[f"{x:.6f}" if isinstance(x, float) else "" for x in quat],
        ])

    with out.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow([
            "mocap_time_s", "imu_rel_s", "orig_t_ms", "imu", "position", "addr",
            "ax_mg", "ay_mg", "az_mg", "gx_dps", "gy_dps", "gz_dps",
            "qw", "qx", "qy", "qz",
        ])
        writer.writerows(rows)
    return len(parsed), len(rows), float(first_t)


def write_clean_mocap(path: Path, out: Path, start_s: float, end_s: float) -> tuple[int, int, int]:
    t, markers = parse_motive_markers(path, gap_fill=False)
    names = sorted(markers)
    keep = (t >= start_s) & (t <= end_s)
    with out.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        header = ["mocap_time_s"]
        for name in names:
            header.extend([f"{name}_x", f"{name}_y", f"{name}_z"])
        writer.writerow(header)
        for i in np.where(keep)[0]:
            row: list[str] = [f"{t[i]:.6f}"]
            for name in names:
                row.extend("" if not np.isfinite(v) else f"{v:.9f}" for v in markers[name][i])
            writer.writerow(row)
    return len(t), int(np.sum(keep)), len(names)


def main() -> int:
    args = parse_args()
    default_imu, default_mocap = default_paths(args.subject)
    imu = args.imu or default_imu
    mocap = args.mocap or default_mocap
    if not imu.exists():
        raise SystemExit(f"IMU log not found: {imu}")
    if not mocap.exists():
        raise SystemExit(f"MoCap CSV not found: {mocap}")

    ti, ia = imu_activity(imu)
    tm, ma, n_markers = mocap_activity(mocap)
    ia, ma = zscore(ia), zscore(ma)

    if args.manual_b is None:
        (corr, a, b), coarse = auto_align(ti, ia, tm, ma, args.b_min, args.b_max)
        mode = "auto"
    else:
        a, b = args.manual_a, args.manual_b
        corr = corr_for(ti, ia, tm, ma, a, b)
        coarse = [(corr_for(ti, ia, tm, ma, 1.0, x), 1.0, float(x)) for x in np.arange(b - 5, b + 5.5, 0.5)]
        coarse.sort(reverse=True)
        mode = "manual"

    m0, m1 = overlap(ti, tm, a, b)
    if args.crop_mocap_start is not None:
        m0 = max(m0, float(args.crop_mocap_start))
    if args.crop_mocap_end is not None:
        m1 = min(m1, float(args.crop_mocap_end))
    i0, i1 = a * m0 + b, a * m1 + b
    rec = {
        "subject": f"{int(args.subject):02d}",
        "mode": mode,
        "imu": str(imu),
        "mocap": str(mocap),
        "mocap_markers_used": n_markers,
        "clock": {"a": a, "b": b, "corr": corr, "formula": "t_imu = a*t_mocap + b"},
        "durations_s": {"imu": float(ti[-1]), "mocap": float(tm[-1])},
        "overlap_s": {
            "mocap_start": float(m0),
            "mocap_end": float(m1),
            "imu_start": float(i0),
            "imu_end": float(i1),
            "duration": float(max(0.0, m1 - m0)),
        },
        "recommended_crop_s": {
            "imu_start_keep_neutral": float(max(ti[0], i0 - args.neutral_keep_s)),
            "imu_end": float(i1),
            "mocap_start": float(m0),
            "mocap_end": float(m1),
        },
        "top_coarse_offsets": [
            {"b": x[2], "corr": x[0]} for x in coarse[:10]
        ],
    }

    args.out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"T{int(args.subject):02d}_P{int(args.subject):02d}_{mode}"
    out_json = args.out_dir / f"{stem}_sync.json"
    out_png = args.out_dir / f"{stem}_sync.png"
    out_detail = args.out_dir / f"{stem}_sync_detail.png"
    out_json.write_text(json.dumps(rec, indent=2) + "\n", encoding="utf-8")
    write_plot(out_png, ti, ia, tm, ma, a, b, coarse[:10])
    if args.detail_chunk_s and args.detail_chunk_s > 0:
        write_detail_plot(out_detail, ti, ia, tm, ma, a, b, args.detail_chunk_s)
    clean_manifest = None
    if args.write_clean:
        clean_manifest = write_clean_outputs(
            args.clean_dir, rec, imu, mocap, a, b, f"{int(args.subject):02d}"
        )

    print(f"{stem}: corr={corr:.3f}  t_imu={a:.6f}*t_mocap+{b:.3f}")
    print(f"  overlap MoCap {m0:.1f}-{m1:.1f}s ({m1-m0:.1f}s), IMU {i0:.1f}-{i1:.1f}s")
    print(f"  keep IMU from {rec['recommended_crop_s']['imu_start_keep_neutral']:.1f}s to {i1:.1f}s")
    print(f"  wrote {out_json}")
    print(f"  wrote {out_png}")
    if args.detail_chunk_s and args.detail_chunk_s > 0:
        print(f"  wrote {out_detail}")
    if clean_manifest:
        paths = clean_manifest["clean_outputs"]
        print(f"  wrote clean folder {paths['folder']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
