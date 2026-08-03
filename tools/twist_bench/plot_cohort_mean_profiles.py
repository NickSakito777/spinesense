from __future__ import annotations

"""Create a seven-line cohort-mean action profile with equal subject weighting."""

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
from scipy.stats import t as student_t


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
COLORS = {
    "mocap_thorax": "#111111",
    "mocap_pelvis": "#7f7f7f",
    "IMU0_sternum": "#d62728",
    "IMU1_sacrum": "#1f77b4",
    "IMU2_lower": "#2ca02c",
    "IMU3_mid": "#9467bd",
    "IMU4_upper": "#ff7f0e",
}
MARKERS = {
    "mocap_thorax": "o",
    "mocap_pelvis": "s",
    "IMU0_sternum": "D",
    "IMU1_sacrum": "v",
    "IMU2_lower": "P",
    "IMU3_mid": "X",
    "IMU4_upper": "^",
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


def aggregate(rows: list[dict[str, str]]) -> list[dict[str, Any]]:
    subjects = sorted({row["subject"] for row in rows})
    if len(subjects) != 13:
        raise ValueError(f"expected 13 subjects, got {len(subjects)}")
    lookup = {(row["subject"], row["action"], row["source"]): row for row in rows}
    if len(lookup) != len(subjects) * len(ACTIONS) * len(SOURCES):
        raise ValueError("subject/action/source table is incomplete")

    out: list[dict[str, Any]] = []
    for action in ACTIONS:
        for source in SOURCES:
            values = np.asarray([
                float(lookup[(subject, action, source)]["median_excursion_deg"])
                for subject in subjects
                if lookup[(subject, action, source)]["median_excursion_deg"]
            ], dtype=float)
            if len(values) < 2:
                raise ValueError(f"insufficient subject values for {action}/{source}")
            mean = float(np.mean(values))
            sd = float(np.std(values, ddof=1))
            sem = sd / np.sqrt(len(values))
            ci95 = float(student_t.ppf(0.975, df=len(values) - 1) * sem)
            out.append({
                "action": action,
                "action_label": ACTION_LABELS[action],
                "source": source,
                "source_label": SOURCE_LABELS[source],
                "mean_of_subject_medians_deg": round(mean, 6),
                "subject_sd_deg": round(sd, 6),
                "subject_sem_deg": round(sem, 6),
                "ci95_low_deg": round(mean - ci95, 6),
                "ci95_high_deg": round(mean + ci95, 6),
                "n_subjects": int(len(values)),
            })
    if len(out) != 42:
        raise ValueError(f"expected 42 cohort mean rows, got {len(out)}")
    return out


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot(prefix: Path, rows: list[dict[str, Any]], dpi: int) -> list[Path]:
    lookup = {(row["source"], row["action"]): row for row in rows}
    x = np.arange(len(ACTIONS), dtype=float)
    width_in, height_in = 30.0, 17.5
    matplotlib.rcParams["pdf.fonttype"] = 42
    matplotlib.rcParams["svg.fonttype"] = "none"
    fig, ax = plt.subplots(figsize=(width_in, height_in), constrained_layout=True)

    for source in SOURCES:
        source_rows = [lookup[(source, action)] for action in ACTIONS]
        mean = np.asarray([row["mean_of_subject_medians_deg"] for row in source_rows], dtype=float)
        low = np.asarray([row["ci95_low_deg"] for row in source_rows], dtype=float)
        high = np.asarray([row["ci95_high_deg"] for row in source_rows], dtype=float)
        ax.errorbar(
            x,
            mean,
            yerr=np.vstack([mean - low, high - mean]),
            label=SOURCE_LABELS[source],
            color=COLORS[source],
            marker=MARKERS[source],
            markersize=15.0,
            linewidth=4.5 if source.startswith("mocap") else 3.2,
            linestyle="-" if source.startswith("mocap") else "--",
            capsize=9.0,
            capthick=2.4,
            elinewidth=2.2,
            alpha=0.97,
        )

    ax.set_xticks(x, [ACTION_LABELS[action] for action in ACTIONS])
    ax.set_ylim(bottom=0.0)
    ax.set_xlabel("Prescribed action", fontsize=22)
    ax.set_ylabel("Across-subject mean neutral-referenced excursion (deg)", fontsize=22)
    ax.set_title(
        "Seven-source trend after averaging all available subjects\n"
        "Point = equal-weight mean of subject-level medians; whisker = 95% confidence interval",
        fontsize=26,
        pad=22,
    )
    ax.tick_params(axis="both", labelsize=18)
    ax.grid(axis="y", linewidth=0.8, alpha=0.23)
    ax.legend(
        loc="upper center",
        bbox_to_anchor=(0.5, -0.09),
        ncol=4,
        frameon=False,
        fontsize=18,
        handlelength=4.0,
        columnspacing=2.1,
    )
    ax.text(
        0.995,
        0.012,
        "n=11–13 per point; bend = swing magnitude; twist = short-window tared axial excursion (not absolute yaw)",
        transform=ax.transAxes,
        ha="right",
        va="bottom",
        fontsize=14,
        color="#555555",
    )

    png = prefix.with_suffix(".png")
    svg = prefix.with_suffix(".svg")
    pdf = prefix.with_suffix(".pdf")
    fig.savefig(svg)
    fig.savefig(pdf)
    fig.savefig(png, dpi=dpi)
    plt.close(fig)
    return [png, svg, pdf]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--out-prefix", type=Path, required=True)
    parser.add_argument("--dpi", type=int, default=400)
    args = parser.parse_args()
    input_path = args.input.resolve()
    prefix = args.out_prefix.resolve()
    csv_path = prefix.with_name(prefix.name + "_summary.csv")
    json_path = prefix.with_name(prefix.name + "_summary.json")
    manifest_path = prefix.with_name(prefix.name + "_manifest.json")
    targets = [prefix.with_suffix(ext) for ext in (".png", ".svg", ".pdf")]
    targets.extend([csv_path, json_path, manifest_path])
    if args.dpi < 300:
        raise SystemExit("high-resolution output requires dpi >= 300")
    existing = [str(path) for path in targets if path.exists()]
    if existing:
        raise SystemExit(f"refusing to overwrite existing outputs: {existing}")
    prefix.parent.mkdir(parents=True, exist_ok=True)

    cohort_rows = aggregate(read_rows(input_path))
    write_csv(csv_path, cohort_rows)
    json_path.write_text(json.dumps(cohort_rows, indent=2) + "\n", encoding="utf-8")
    plot_paths = plot(prefix, cohort_rows, args.dpi)
    produced = [*plot_paths, csv_path, json_path]
    manifest = {
        "schema_version": 1,
        "run_type": "cohort_mean_action_source_profile_v1",
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
            "point": "arithmetic mean of available subject-level action/source medians",
            "subject_weighting": "equal",
            "whisker": "two-sided 95% Student-t confidence interval across subjects",
            "n_subjects_range": [11, 13],
        },
        "counts": {"subjects_available": 13, "actions": 6, "sources": 7, "mean_points": 42},
        "png_dimensions_px": [round(30.0 * args.dpi), round(17.5 * args.dpi)],
        "outputs": {
            path.name: {"sha256": sha256_file(path), "size_bytes": path.stat().st_size}
            for path in produced
        },
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"counts": manifest["counts"], "png_dimensions_px": manifest["png_dimensions_px"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
