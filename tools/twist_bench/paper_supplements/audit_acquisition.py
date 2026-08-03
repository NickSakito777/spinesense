#!/usr/bin/env python3
"""Acquisition audit for a fourteen-field five-IMU streaming log.

Computes the observables listed in Dissertation Table 5.3 (acquisition test)
under the definitions of Appendix A.13: session duration, observed frame rate,
the three completeness proportions, and the failure-record counts.

Layering (Appendix A.13):
  * software frame  -- all records sharing one millisecond stamp t_ms
  * record slot     -- one expected (t_ms, interface) pair; a frame expects five

Analysis set: records are read in file order; the final frame is dropped
because acquisition stops mid-cycle and it holds fewer than five records. A
truncated leading row and the periodically re-emitted column header are not
records and are counted separately.

Observed frame rate over a segment of N frames, first stamp t_1, last t_N:

    f_obs = 1000 (N - 1) / (t_N - t_1)   Hz

Usage:
    python3 audit_acquisition.py <log> [--json out.json]
"""
import argparse
import json
import sys
from collections import defaultdict

N_INTERFACES = 5
N_FIELDS = 14
NON_FINITE = {"nan", "-nan", "inf", "-inf", "+inf"}


def audit(path):
    frame_interfaces = defaultdict(set)     # t_ms -> {interface}
    frame_finite_q = defaultdict(int)       # t_ms -> records with finite quaternion
    order, seen = [], set()
    slots = set()
    read_fail = duplicate_slots = non_finite_q = 0
    header_rows = malformed_rows = 0

    with open(path) as fh:
        for line in fh:
            parts = line.rstrip("\n").split("\t")
            if len(parts) == 4 and "READ_FAIL" in line:
                read_fail += 1                        # raw-read failure bypass (5.5)
                continue
            if len(parts) == N_FIELDS and parts[1] == "imu":
                header_rows += 1                      # periodic column header
                continue
            if len(parts) != N_FIELDS or not parts[1].startswith("IMU"):
                malformed_rows += 1                   # e.g. truncated leading row
                continue
            try:
                t = int(parts[0])
            except ValueError:
                malformed_rows += 1
                continue
            interface = parts[1]
            if (t, interface) in slots:
                duplicate_slots += 1
            slots.add((t, interface))
            frame_interfaces[t].add(interface)
            if all(v.strip().lower() not in NON_FINITE for v in parts[10:14]):
                frame_finite_q[t] += 1
            else:
                non_finite_q += 1
            if t not in seen:
                seen.add(t)
                order.append(t)

    if len(order) < 2:
        raise SystemExit(f"{path}: too few frames to audit")

    stamps = order[:-1]                               # drop the final partial frame
    n = len(stamps)
    span_ms = stamps[-1] - stamps[0]
    monotonic = all(b > a for a, b in zip(stamps, stamps[1:]))
    intervals = [b - a for a, b in zip(stamps, stamps[1:])]
    interval_counts = {v: intervals.count(v) for v in sorted(set(intervals))}

    filled = sum(len(frame_interfaces[t]) for t in stamps)
    expected = N_INTERFACES * n
    full_records = sum(1 for t in stamps
                       if len(frame_interfaces[t]) == N_INTERFACES)
    full_finite_q = sum(1 for t in stamps
                        if frame_finite_q[t] == N_INTERFACES)

    return {
        "log": path,
        "frames": n,
        "duration_s": round(span_ms / 1000.0, 1),
        "observed_rate_hz": round(1000.0 * (n - 1) / span_ms, 4),
        "mean_interval_ms": round(span_ms / (n - 1), 4),
        "interval_counts_ms": interval_counts,
        "monotonic_no_gaps": monotonic and max(interval_counts) <= 9,
        "record_slots": {"filled": filled, "expected": expected,
                         "pct": round(100.0 * filled / expected, 4)},
        "frames_five_interface_complete": {
            "n": full_records, "pct": round(100.0 * full_records / n, 4)},
        "frames_five_finite_quaternion": {
            "n": full_finite_q, "pct": round(100.0 * full_finite_q / n, 4)},
        "read_fail_records": read_fail,
        "non_finite_quaternion_records": non_finite_q,
        "duplicate_slots": duplicate_slots,
        "header_rows": header_rows,
        "malformed_rows": malformed_rows,
        # Every incomplete frame should be explained by a logged failure record.
        "shortfall_explained": (n - full_records) == read_fail,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--json", help="also write the report as JSON")
    args = ap.parse_args()

    r = audit(args.log)
    w = 34
    print(f"{'frames (analysis set)':<{w}} {r['frames']:,}")
    print(f"{'duration (s)':<{w}} {r['duration_s']}")
    print(f"{'observed frame rate (Hz)':<{w}} {r['observed_rate_hz']}")
    print(f"{'mean inter-frame interval (ms)':<{w}} {r['mean_interval_ms']}")
    print(f"{'inter-frame intervals (ms)':<{w}} "
          + ", ".join(f"{k}:{v:,}" for k, v in r["interval_counts_ms"].items()))
    print(f"{'strictly increasing, no gaps':<{w}} {r['monotonic_no_gaps']}")
    rs = r["record_slots"]
    print(f"{'record-slot completeness':<{w}} {rs['filled']:,}/{rs['expected']:,} "
          f"= {rs['pct']}%")
    fr = r["frames_five_interface_complete"]
    print(f"{'five-interface complete frames':<{w}} {fr['n']:,} = {fr['pct']}%")
    fq = r["frames_five_finite_quaternion"]
    print(f"{'five-interface finite-q frames':<{w}} {fq['n']:,} = {fq['pct']}%")
    print(f"{'READ_FAIL records':<{w}} {r['read_fail_records']:,}")
    print(f"{'non-finite quaternion records':<{w}} {r['non_finite_quaternion_records']:,}")
    print(f"{'duplicate record slots':<{w}} {r['duplicate_slots']:,}")
    print(f"{'header rows / malformed rows':<{w}} {r['header_rows']:,} / "
          f"{r['malformed_rows']:,}")
    print(f"{'shortfall explained by failures':<{w}} {r['shortfall_explained']}")

    if args.json:
        with open(args.json, "w") as fh:
            json.dump(r, fh, indent=2)
        print(f"\nwrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
