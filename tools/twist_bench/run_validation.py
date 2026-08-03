from __future__ import annotations

"""CLICK-TO-RUN validation. No arguments needed.

1. Drop your new IMU log into   tools/twist_bench/data/<name>.log
2. Drop your new MoCap CSV into  tools/twist_bench/data/mocap/<name>.csv
3. Open this file in VS Code and press Run (or F5).  <-- that's it.

It auto-picks the NEWEST .log and .csv, auto-aligns the two clocks (no manual offset),
runs the preflight data check, then the full 6-movement IMU-vs-MoCap accuracy (VQF).

Must run with the venv interpreter (it has VQF): the VS Code workspace already selects
.venv\\Scripts\\python.exe, so just pressing Run uses the right one.
"""

import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import twist_bench_v0 as v0
import mocap_adapter as mc


def newest(folder: Path, pattern: str) -> Path:
    files = list(folder.glob(pattern))
    if not files:
        raise SystemExit(f"No {pattern} found in {folder}. Put your file there first.")
    return max(files, key=lambda p: p.stat().st_mtime)


def imu_activity(log: Path):
    """IMU movement envelope on IMU time: mean |gyro| over all IMUs at each timestamp. No fusion needed."""
    text = log.read_text(encoding="utf-8", errors="replace")
    recs = v0.parse_serial_text(text) or v0.parse_long_table_rows(v0.read_dict_rows(text))
    if not recs:
        raise SystemExit(f"No IMU records parsed from {log}.")
    by_t: dict[float, list[float]] = {}
    for r in recs:
        by_t.setdefault(r.t_s, []).append((r.gx_dps**2 + r.gy_dps**2 + r.gz_dps**2) ** 0.5)
    t = np.array(sorted(by_t))
    a = np.array([np.mean(by_t[x]) for x in t])
    return t - t[0], a


def mocap_activity(csv: Path):
    """MoCap movement envelope on MoCap time: speed of the total trunk angle."""
    tm, mk, _ = mc.parse_motive_csv(csv)
    tm = tm - tm[0]
    mk = {k: mc._fill_gaps(v) for k, v in mk.items()}
    ang = mc.trunk_angles(mk)
    uw = lambda x: np.degrees(np.unwrap(np.radians(x)))  # noqa: E731
    tot = np.sqrt(uw(ang["axial"]) ** 2 + uw(ang["flex"]) ** 2 + uw(ang["lateral"]) ** 2)
    return tm, np.abs(np.gradient(tot, tm))


def auto_align(ti, ia, tm, ma):
    """Robust clock map t_imu = a*t_mocap + b. Coarse GLOBAL offset (full-signal corr, drift=1) then
    refine drift, so it cannot mis-lock the way a per-window search can."""
    def smooth(t, x, win_s=0.5):
        dt = float(np.median(np.diff(t)))
        w = max(1, int(win_s / dt))
        return np.convolve(x, np.ones(w) / w, mode="same")
    # smooth to the movement-burst envelope (raw gyro magnitude vs angle-speed correlate poorly otherwise)
    ia = smooth(ti, ia); ma = smooth(tm, ma)
    ia = (ia - ia.mean()) / (ia.std() + 1e-9)
    ma = (ma - ma.mean()) / (ma.std() + 1e-9)

    def corr(a, b):
        g0 = max(tm[0], (ti[0] - b) / a)
        g1 = min(tm[-1], (ti[-1] - b) / a)
        if g1 - g0 < 20:
            return -2.0
        g = np.arange(g0, g1, 0.05)
        I = np.interp(a * g + b, ti, ia)
        M = np.interp(g, tm, ma)
        if np.std(I) < 1e-6 or np.std(M) < 1e-6:
            return -2.0
        return float(np.corrcoef(I, M)[0, 1])

    # coarse global offset (whole-signal pattern only matches at the true offset -> robust)
    Bs = np.arange(0.0, 150.0, 0.5)
    B0 = Bs[int(np.argmax([corr(1.0, b) for b in Bs]))]
    # joint refine of drift a and offset b around B0
    best = (-2.0, 1.0, B0)
    for a in np.arange(0.994, 1.006, 0.0002):
        for b in np.arange(B0 - 2.0, B0 + 2.0, 0.04):
            c = corr(a, b)
            if c > best[0]:
                best = (c, a, b)
    return best[1], best[2], best[0]


def main() -> int:
    log = newest(HERE / "data", "*.log")
    csv = newest(HERE / "data" / "mocap", "*.csv")
    print("=" * 70)
    print(f"newest IMU log : {log.name}")
    print(f"newest MoCap   : {csv.name}")
    print("=" * 70)

    print("\n[1/3] preflight data check ...")
    subprocess.run([sys.executable, str(HERE / "validation1_preflight.py"), "--input", str(log)])

    print("\n[2/3] auto-aligning the two clocks (no manual offset) ...")
    ti, ia = imu_activity(log)
    tm, ma = mocap_activity(csv)
    a, b, r = auto_align(ti, ia, tm, ma)
    print(f"      => t_imu = {a:.6f} * t_mocap + {b:.3f}   (alignment quality r={r:.2f})")
    if r < 0.5:
        print("      [!] LOW alignment quality. Check the files are the same session, or that there is")
        print("          a clear movement burst near the start. You can still inspect the table below.")

    print("\n[3/3] full 6-movement IMU-vs-MoCap accuracy (VQF) ...\n")
    subprocess.run([
        sys.executable, str(HERE / "validation1_full_accuracy.py"),
        "--imu", str(log), "--mocap", str(csv),
        "--clock-a", f"{a:.6f}", "--clock-b", f"{b:.3f}",
    ])
    print("\nDone. (twist channel: read 'protocol-assisted, calibrated, n=1' — see chat for the claim wording.)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
