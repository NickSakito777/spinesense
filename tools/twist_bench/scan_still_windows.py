"""Scan a twist-bench serial log for still windows usable as bias/tare segments."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from twist_bench_fusion import load_fusion_pairs


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--parent", default="IMU1")
    parser.add_argument("--child", default="IMU2")
    parser.add_argument("--window-s", type=float, default=3.0)
    parser.add_argument("--still-threshold-dps", type=float, default=1.0)
    args = parser.parse_args()

    p = load_fusion_pairs(args.input, args.parent, args.child)
    t = p.t_s
    found_still = False
    for w0 in np.arange(0.0, max(0.0, t[-1] - args.window_s) + 1e-9, 1.0):
        sel = (t >= w0) & (t < w0 + args.window_s)
        if sel.sum() < 10:
            continue
        ps = float(np.abs(p.parent_gyr_dps[sel] - p.parent_gyr_dps[sel].mean(0)).max())
        cs = float(np.abs(p.child_gyr_dps[sel] - p.child_gyr_dps[sel].mean(0)).max())
        still = max(ps, cs) < args.still_threshold_dps
        found_still = found_still or still
        flag = "STILL" if still else ""
        print(f"t={w0:5.1f}-{w0 + args.window_s:5.1f}s  parent_maxdev={ps:6.2f}  child_maxdev={cs:6.2f} dps  {flag}")
    if not found_still:
        print(f"NO still window below {args.still_threshold_dps} dps found; "
              "capture a trial with a quiet neutral hold before motion.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
