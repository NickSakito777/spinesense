from __future__ import annotations

"""Merge subject-level seven-source profiles into one cohort profile.

Each cohort point is the median of the available subject-level medians for the
same action/source. Whiskers are the subject-level Q1--Q3 range, so every
participant has equal weight regardless of their number of scored bouts.
"""

import argparse
import csv
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ACTIONS = ["flexion", "extension", "left_bend", "right_bend", "left_twist", "right_twist"]
ACTION_LABELS = {
    "flexion": "Flexion",
    "extension": "Extension",
    "left_bend": "Left bend",
    "right_bend": "Right bend",
    "left_twist": "Left twist",
    "right_twist": "Right twist",
}
SOURCES = [
    "mocap_thorax",
    "mocap_pelvis",
    "IMU0_sternum",
    "IMU1_sacrum",
    "IMU2_lower",
    "IMU3_mid",
    "IMU4_upper",
]
SOURCE_LABELS = {
    "mocap_thorax": "MoCap thorax",
    "mocap_pelvis": "MoCap pelvis",
    "IMU0_sternum": "IMU0 sternum",
    "IMU1_sacrum": "IMU1 sacrum/S1",
    "IMU2_lower": "IMU2 lower",
    "IMU3_mid": "IMU3 mid",
    "IMU4_upper": "IMU4 upper",
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def merge(rows: list[dict[str, str]]) -> list[dict[str, Any]]:
    subjects = sorted({row["subject"] for row in rows})
    if len(subjects) != 13:
        raise ValueError(f"expected 13 subjects, got {len(subjects)}")
    lookup = {(row["subject"], row["action"], row["source"]): row for row in rows}
    expected = len(subjects) * len(ACTIONS) * len(SOURCES)
    if len(lookup) != expected:
        raise ValueError(f"expected {expected} unique subject/action/source rows, got {len(lookup)}")

    out: list[dict[str, Any]] = []
    for action in ACTIONS:
        for source in SOURCES:
            values = [
                float(lookup[(subject, action, source)]["median_excursion_deg"])
                for subject in subjects
                if lookup[(subject, action, source)]["median_excursion_deg"]
            ]
            if not values:
                raise ValueError(f"no cohort values for {action}/{source}")
            q1, median, q3 = np.percentile(np.asarray(values), [25.0, 50.0, 75.0])
            out.append({
                "action": action,
                "action_label": ACTION_LABELS[action],
                "source": source,
                "source_label": SOURCE_LABELS[source],
                "cohort_median_deg": round(float(median), 6),
                "subject_q1_deg": round(float(q1), 6),
                "subject_q3_deg": round(float(q3), 6),
                "n_subjects": len(values),
            })
    if len(out) != 42:
        raise ValueError(f"expected 42 merged rows, got {len(out)}")
    return out


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot(path: Path, rows: list[dict[str, Any]]) -> None:
    lookup = {(row["source"], row["action"]): row for row in rows}
    colors = {
        "mocap_thorax": "#111111",
        "mocap_pelvis": "#777777",
        "IMU0_sternum": "#d62728",
        "IMU1_sacrum": "#1f77b4",
        "IMU2_lower": "#2ca02c",
        "IMU3_mid": "#9467bd",
        "IMU4_upper": "#ff7f0e",
    }
    markers = {
        "mocap_thorax": "o",
        "mocap_pelvis": "s",
        "IMU0_sternum": "D",
        "IMU1_sacrum": "v",
        "IMU2_lower": "P",
        "IMU3_mid": "X",
        "IMU4_upper": "^",
    }
    x = np.arange(len(ACTIONS))
    fig, ax = plt.subplots(figsize=(13.2, 7.4), constrained_layout=True)
    for source in SOURCES:
        sr = [lookup[(source, action)] for action in ACTIONS]
        y = np.asarray([row["cohort_median_deg"] for row in sr], dtype=float)
        q1 = np.asarray([row["subject_q1_deg"] for row in sr], dtype=float)
        q3 = np.asarray([row["subject_q3_deg"] for row in sr], dtype=float)
        ax.errorbar(
            x,
            y,
            yerr=np.vstack([y - q1, q3 - y]),
            label=SOURCE_LABELS[source],
            color=colors[source],
            marker=markers[source],
            markersize=6.0,
            linewidth=2.5 if source.startswith("mocap") else 1.7,
            linestyle="-" if source.startswith("mocap") else "--",
            capsize=3.5,
            alpha=0.96,
        )
    ax.set_xticks(x, [ACTION_LABELS[action] for action in ACTIONS])
    ax.set_ylabel("Neutral-referenced excursion (deg)")
    ax.set_xlabel("Prescribed action")
    ax.set_title(
        "All subjects merged: seven-source six-action profile\n"
        "Point = median of subject medians; whisker = subject Q1–Q3 (n=11–13)"
    )
    ax.set_ylim(bottom=0.0)
    ax.grid(axis="y", alpha=0.22)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=4, frameon=False)
    ax.text(
        0.995,
        0.01,
        "Bend = swing magnitude; twist = short-window tared axial excursion (not absolute yaw)",
        transform=ax.transAxes,
        ha="right",
        va="bottom",
        fontsize=8,
        color="#555555",
    )
    fig.savefig(path.with_suffix(".png"), dpi=180)
    fig.savefig(path.with_suffix(".svg"))
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--out-prefix", type=Path, required=True)
    args = parser.parse_args()
    input_path = args.input.resolve()
    prefix = args.out_prefix.resolve()
    outputs = [
        prefix.with_suffix(".png"),
        prefix.with_suffix(".svg"),
        prefix.parent / "cohort_merged_subject_summary.csv",
        prefix.parent / "cohort_merged_subject_summary.json",
        prefix.parent / "cohort_merged_manifest.json",
    ]
    existing = [str(path) for path in outputs if path.exists()]
    if existing:
        raise SystemExit(f"refusing to overwrite existing outputs: {existing}")
    prefix.parent.mkdir(parents=True, exist_ok=True)

    merged = merge(read_rows(input_path))
    plot(prefix, merged)
    csv_path = prefix.parent / "cohort_merged_subject_summary.csv"
    json_path = prefix.parent / "cohort_merged_subject_summary.json"
    write_csv(csv_path, merged)
    json_path.write_text(json.dumps(merged, indent=2) + "\n", encoding="utf-8")

    produced = [prefix.with_suffix(".png"), prefix.with_suffix(".svg"), csv_path, json_path]
    manifest = {
        "schema_version": 1,
        "run_type": "cohort_merged_action_source_profile_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "input": {
            "path": str(input_path),
            "sha256": sha256_file(input_path),
            "size_bytes": input_path.stat().st_size,
        },
        "generator": {
            "path": str(Path(__file__).resolve()),
            "sha256": sha256_file(Path(__file__).resolve()),
        },
        "definition": {
            "unit_of_analysis": "subject",
            "point": "median of available subject-level action/source medians",
            "whisker": "Q1 to Q3 of available subject-level medians",
            "subject_weighting": "equal",
            "n_subjects_range": [11, 13],
        },
        "counts": {"merged_rows": len(merged), "actions": 6, "sources": 7},
        "outputs": {
            str(path.relative_to(prefix.parent)): {
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
            for path in produced
        },
    }
    manifest_path = prefix.parent / "cohort_merged_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest["counts"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
