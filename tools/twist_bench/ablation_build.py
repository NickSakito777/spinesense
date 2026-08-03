"""Per-channel feature builder for the five-sensor ablation (Meeting11 action item).

Shadow-only builder. It reuses ``dataset_adapter.subject_blocks`` as the frozen
authority for raw segments, clocks, protocol windows and detected bouts, exactly like
``corrected_regional_qc.py``. Nothing here re-detects a bout or repairs a clock.

WHY THIS EXISTS
---------------
The frozen 13-feature classifier draws every feature from two relations only --
``sacrum_to_upper`` (bend) and ``sacrum_to_sternum`` (twist). That is three physical
sensors: IMU1/sacrum, IMU4/upper, IMU0/sternum. ``lower`` (IMU2) and ``mid`` (IMU3)
never enter the model matrix. The supervisor asked (Meeting11 section 2) for a
sensor-subset ablation to establish which placements are necessary and which are
redundant. Answering that needs features defined per CHANNEL, not one fused block.

CHANNEL DEFINITION
------------------
A channel is one quaternion time series, locally re-tared per bout on the same
``pre_mask`` window the frozen pipeline uses, then decomposed by swing-twist into three
signed scalars (swing-x, swing-y, twist). The same 13 scale-invariant, axis-agnostic
features the frozen pipeline computes are then emitted from those three scalars.

  solo_<role>            q_segment of one sensor, re-tared against its own neutral.
                         World/gravity referenced: no differential yaw cancellation.
                         This is what a genuine single-sensor system measures.
  pair_<parent>_<child>  qrel(parent, child): the child expressed in the parent frame.
                         Differential: shared heading error partially cancels.

Fifteen channels are built once (5 solo + 10 pairs). Every one of the 31 non-empty
sensor subsets is then a column selection over that wide table -- no rebuild per subset.

SUBSET -> CHANNEL RULE (fixed-root, k-1 channels)
-------------------------------------------------
Roles are ranked ``sacrum < lower < mid < upper < sternum``; the lowest-ranked member of
a subset is its root, and every other member is taken relative to that root. This mirrors
the frozen pipeline (sacrum is already the root) and keeps feature dimension linear in k.

REFERENCE-FRAME CONFOUND -- must be stated in the dissertation
--------------------------------------------------------------
Single-sensor subsets are world-referenced; multi-sensor subsets are differential. A
solo-vs-pair gap therefore mixes two causes: sensor count and measurement principle. The
ablation cannot separate them, and the write-up must not claim otherwise.

LEGACY CHANNEL
--------------
``legacy`` reproduces the frozen fused block exactly (swing from ``sacrum_to_upper``,
twist from ``sacrum_to_sternum``) so the rebuilt row set can be checked against the
frozen ``features_model.csv`` and its published LOSO accuracy.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent          # .../twist_bench
sys.path.insert(0, str(HERE))

import dataset_adapter as bd              # noqa: E402  (frozen bout/clock authority)
import placement_maps as pm             # noqa: E402
import session_recipe as t06        # noqa: E402  (qrel + pre_mask)
import signed_diagnostic as sd          # noqa: E402
import twist_bench_fusion as pf         # noqa: E402

EPS = bd.EPS

# Ranked inferior -> superior along the posterior chain, sternum last (anterior terminal).
# The lowest-ranked member of a subset becomes its reference frame.
ROOT_ORDER = ["sacrum", "lower", "mid", "upper", "sternum"]

FEAT_NAMES = [
    "frac_x", "frac_y", "frac_tw", "logr_xy", "logr_tw_bend",
    "sign_x", "sign_y", "sign_tw", "dir_x", "dir_y", "dir_tw",
    "n_sign_changes", "tw_dominance",
]


def channel_names() -> list[str]:
    names = [f"solo_{r}" for r in ROOT_ORDER]
    for i, parent in enumerate(ROOT_ORDER):
        for child in ROOT_ORDER[i + 1:]:
            names.append(f"pair_{parent}_{child}")
    names.append("legacy")
    return names


def subset_channels(subset: tuple[str, ...]) -> list[str]:
    """Fixed-root channel list for one sensor subset."""
    ordered = [r for r in ROOT_ORDER if r in subset]
    if not ordered:
        raise ValueError("empty subset")
    if len(ordered) == 1:
        return [f"solo_{ordered[0]}"]
    root = ordered[0]
    return [f"pair_{root}_{c}" for c in ordered[1:]]


def channel_raw(res, name: str) -> tuple[np.ndarray, np.ndarray]:
    """Untared source quaternions for one channel: (swing source, twist source).

    Both are the same series except for ``legacy``, which reproduces the frozen split of
    swing from ``sacrum_to_upper`` and twist from ``sacrum_to_sternum``. Computed once per
    block and reused across that block's bouts.
    """
    if name == "legacy":
        return t06.qrel(res, "sacrum", "upper"), t06.qrel(res, "sacrum", "sternum")
    if name.startswith("solo_"):
        q = res.sensors[name[len("solo_"):]].q_segment
        return q, q
    if name.startswith("pair_"):
        parent, child = name[len("pair_"):].split("_", 1)
        q = t06.qrel(res, parent, child)
        return q, q
    raise ValueError(f"unknown channel {name!r}")


def _retare_masked(q: np.ndarray, tare: np.ndarray, m: np.ndarray) -> np.ndarray:
    """Re-tare on the pre_mask window, then keep only the bout window.

    Tare and evaluation windows are both slices of the same series, so restricting the
    output to ``m`` is equivalent to tarng the full series and slicing afterwards. Twist
    unwrapping stays self-consistent because ``m`` is a contiguous interval and every
    feature is computed after subtracting the window's first sample.
    """
    q0 = pf.quat_average(q[tare])
    return pf.qmul(pf.qconj(q0)[None, :], q[m])


def channel_scalars(raw: tuple[np.ndarray, np.ndarray], tare: np.ndarray,
                    m: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Three signed scalars (swing-x, swing-y, twist) over the bout window, in degrees."""
    q_sw, q_tw = raw
    qt_sw = _retare_masked(q_sw, tare, m)
    rv = bd.swing_rotvec_deg(qt_sw)
    qt_tw = qt_sw if q_tw is q_sw else _retare_masked(q_tw, tare, m)
    twist, _ = pf.swing_twist_deg(qt_tw, pf.SEGMENT_TWIST_AXIS)
    return rv[:, 0], rv[:, 1], -pf.unwrap_deg(twist)


def features_from_scalars(px: np.ndarray, py: np.ndarray, pw: np.ndarray) -> dict[str, float]:
    """The frozen 13 scale-invariant features, computed from one channel's three scalars.

    Mirrors the reference bout-feature builder term for term; only the source of the three
    scalars differs.
    """
    px = px - px[0]
    py = py - py[0]
    pw = pw - pw[0]

    Ex, Ey, Ew = float(np.var(px)), float(np.var(py)), float(np.var(pw))
    tot = Ex + Ey + Ew + EPS
    peak_x = bd._peak_signed(px)
    peak_y = bd._peak_signed(py)
    peak_w = bd._peak_signed(pw)
    bend_amp = abs(peak_x) + abs(peak_y) + EPS
    all_amp = abs(peak_x) + abs(peak_y) + abs(peak_w) + EPS

    dom = int(np.argmax([Ex, Ey, Ew]))
    dom_series = [px, py, pw][dom]

    return {
        "frac_x": Ex / tot,
        "frac_y": Ey / tot,
        "frac_tw": Ew / tot,
        "logr_xy": float(np.log((Ex + EPS) / (Ey + EPS))),
        "logr_tw_bend": float(np.log((Ew + EPS) / (Ex + Ey + EPS))),
        "sign_x": float(np.sign(peak_x)),
        "sign_y": float(np.sign(peak_y)),
        "sign_tw": float(np.sign(peak_w)),
        "dir_x": peak_x / bend_amp,
        "dir_y": peak_y / bend_amp,
        "dir_tw": peak_w / all_amp,
        "n_sign_changes": bd._n_sign_changes(dom_series),
        "tw_dominance": abs(peak_w) / all_amp,
    }


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Build the per-channel ablation feature table.")
    p.add_argument("--out-dir", type=Path, required=True,
                   help="New or empty output directory.")
    p.add_argument("--analysis-mode", choices=("mapping_only",), default="mapping_only",
                   help="Ablation runs on the mapping_only cohort, matching the headline.")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    out_dir = args.out_dir.resolve()
    if out_dir.exists() and any(out_dir.iterdir()):
        raise SystemExit(f"--out-dir must be new or empty; refusing to overwrite {out_dir}")
    out_dir.mkdir(parents=True, exist_ok=True)

    chans = channel_names()
    rows: list[dict] = []

    for s in bd.SUBJECTS:
        tmoc, mflex, mlat, maxial = sd.mocap_signed(bd.mocap_path(s))
        msig = {"flex": mflex, "lat": mlat, "axial": maxial}
        n_before = len(rows)

        for bid, block, res, a, b, bouts in bd.subject_blocks(s, tmoc, msig):
            label = bd._canon_label(
                bd.BLOCK_OVERRIDES.get(s, {}).get(bid, {}).get("label", block["label"])
            )
            if label not in bd.LABELS:
                continue
            quality = bd.quality_for(s, bid, block)
            ti = res.t_s
            raws = {c: channel_raw(res, c) for c in chans}

            for bout_index, (lo, hi) in enumerate(bouts):
                lo, hi = float(lo), float(hi)
                imu_lo, imu_hi = a * lo + b, a * hi + b
                m = (ti >= imu_lo) & (ti <= imu_hi)
                n = int(np.count_nonzero(m))
                if n < 5:
                    # Same rejection rule as the frozen builder, so the row set matches.
                    continue
                tare = t06.pre_mask(ti, a, b, lo)

                provenance = res.summary["placement_provenance"]
                row = {
                    "subject": f"T{s}",
                    "trial_id": provenance["trial_id"],
                    "mapping_version": provenance["mapping_version"],
                    "mapping_sha256": provenance["mapping_sha256"],
                    "block": bid,
                    "label": label,
                    "y": bd.LABELS[label],
                    "quality": quality,
                    "bout_index": int(bout_index),
                    "bout_start_s": round(lo, 6),
                    "bout_end_s": round(hi, 6),
                    "n_samples": n,
                    "dur_s": round(float(imu_hi - imu_lo), 3),
                }
                for cname in chans:
                    px, py, pw = channel_scalars(raws[cname], tare, m)
                    for k, v in features_from_scalars(px, py, pw).items():
                        row[f"{cname}__{k}"] = v
                rows.append(row)

        print(f"T{s}: {len(rows) - n_before} bouts", flush=True)

    meta_cols = ["subject", "trial_id", "mapping_version", "mapping_sha256",
                 "block", "label", "y", "quality", "bout_index",
                 "bout_start_s", "bout_end_s", "n_samples", "dur_s"]
    feat_cols = [f"{c}__{f}" for c in chans for f in FEAT_NAMES]
    cols = meta_cols + feat_cols

    out_csv = out_dir / "channel_features.csv"
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in cols})

    manifest = {
        "run_type": "sensor_subset_ablation_channels",
        "analysis_mode": args.analysis_mode,
        "purpose": "Meeting11 action item: per-sensor and per-subset 6-class ablation.",
        "frozen_authority": "dataset_adapter.subject_blocks (bouts, clocks, windows)",
        "root_order": ROOT_ORDER,
        "channels": chans,
        "features_per_channel": FEAT_NAMES,
        "n_rows": len(rows),
        "n_subjects": len(bd.SUBJECTS),
        "subjects": [f"T{s}" for s in bd.SUBJECTS],
        "mapping_registry": str(pm.DEFAULT_CONFIG_PATH),
        "mapping_registry_sha256": sha256_file(Path(pm.DEFAULT_CONFIG_PATH)),
        "builder_sha256": sha256_file(Path(__file__).resolve()),
        "outputs": {"channel_features.csv": sha256_file(out_csv)},
        "environment": {"python": platform.python_version(), "numpy": np.__version__},
        "reference_frame_confound": (
            "solo_* channels are world/gravity referenced; pair_* channels are differential. "
            "A solo-vs-pair difference mixes sensor count with measurement principle and "
            "cannot attribute the gap to either alone."
        ),
    }
    (out_dir / "ablation_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(f"\nwrote {out_csv}  rows={len(rows)}  channels={len(chans)}  cols={len(cols)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
