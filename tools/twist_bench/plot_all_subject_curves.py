from __future__ import annotations

"""Overlay every subject/source action profile in one ultra-high-resolution plot."""

import argparse
import csv
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
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
COLORS = {
    "mocap_thorax": "#111111",
    "mocap_pelvis": "#858585",
    "IMU0_sternum": "#d62728",
    "IMU1_sacrum": "#1f77b4",
    "IMU2_lower": "#2ca02c",
    "IMU3_mid": "#9467bd",
    "IMU4_upper": "#ff7f0e",
}
LINESTYLES = {
    "mocap_thorax": "-",
    "mocap_pelvis": "-",
    "IMU0_sternum": (0, (6, 2)),
    "IMU1_sacrum": (0, (4, 2)),
    "IMU2_lower": (0, (2, 2)),
    "IMU3_mid": (0, (7, 2, 2, 2)),
    "IMU4_upper": (0, (9, 2)),
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


def plot(input_path: Path, output_prefix: Path, dpi: int) -> dict[str, object]:
    rows = read_rows(input_path)
    subjects = sorted({row["subject"] for row in rows})
    if subjects != ["T02", "T03", "T04", "T05", "T06", "T08", "T09", "T10", "T11", "T12", "T13", "T14", "T15"]:
        raise ValueError(f"unexpected subjects: {subjects}")
    lookup = {(row["subject"], row["action"], row["source"]): row for row in rows}
    if len(lookup) != len(subjects) * len(ACTIONS) * len(SOURCES):
        raise ValueError("subject/action/source table is incomplete")

    matplotlib.rcParams["pdf.fonttype"] = 42
    matplotlib.rcParams["svg.fonttype"] = "none"
    width_in, height_in = 30.0, 17.5
    fig, ax = plt.subplots(figsize=(width_in, height_in), constrained_layout=True)
    base_x = np.arange(len(ACTIONS), dtype=float)
    offsets = np.linspace(-0.18, 0.18, len(subjects))

    for source in SOURCES:
        for subject, offset in zip(subjects, offsets):
            values = []
            for action in ACTIONS:
                value = lookup[(subject, action, source)]["median_excursion_deg"]
                values.append(np.nan if value == "" else float(value))
            ax.plot(
                base_x + offset,
                values,
                color=COLORS[source],
                linestyle=LINESTYLES[source],
                linewidth=1.25 if source.startswith("mocap") else 1.05,
                alpha=0.62,
                marker=f"${subject[1:]}$",
                markersize=11.5,
                markeredgewidth=0.5,
                zorder=3 if source.startswith("mocap") else 2,
            )

    ax.set_xticks(base_x, [ACTION_LABELS[action] for action in ACTIONS])
    ax.set_xlim(-0.42, 5.42)
    ax.set_ylim(0.0, 105.0)
    ax.set_yticks(np.arange(0.0, 106.0, 5.0))
    ax.set_xlabel("Prescribed action", fontsize=20)
    ax.set_ylabel("Neutral-referenced excursion (deg)", fontsize=20)
    ax.set_title(
        "All 91 subject × source curves in one graph\n"
        "Marker digits identify subject (02–15); horizontal jitter separates participants within each action",
        fontsize=24,
        pad=20,
    )
    ax.tick_params(axis="both", labelsize=16)
    ax.grid(axis="y", linewidth=0.65, alpha=0.23)
    ax.grid(axis="x", linewidth=0.45, alpha=0.12)
    legend = [
        Line2D(
            [],
            [],
            color=COLORS[source],
            linestyle=LINESTYLES[source],
            linewidth=3.0,
            label=SOURCE_LABELS[source],
        )
        for source in SOURCES
    ]
    legend.append(
        Line2D([], [], color="#333333", linestyle="none", marker="$02$", markersize=15, label="Marker digits = subject ID")
    )
    ax.legend(
        handles=legend,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.075),
        ncol=4,
        frameon=False,
        fontsize=16,
        handlelength=4.0,
        columnspacing=2.0,
    )
    ax.text(
        0.995,
        0.012,
        "Subject-level medians; bend = swing magnitude; twist = short-window tared axial excursion (not absolute yaw)",
        transform=ax.transAxes,
        ha="right",
        va="bottom",
        fontsize=13,
        color="#555555",
    )

    png = output_prefix.with_suffix(".png")
    svg = output_prefix.with_suffix(".svg")
    pdf = output_prefix.with_suffix(".pdf")
    fig.savefig(svg)
    fig.savefig(pdf)
    fig.savefig(png, dpi=dpi)
    plt.close(fig)
    return {
        "subjects": subjects,
        "curves": len(subjects) * len(SOURCES),
        "points_expected": len(subjects) * len(SOURCES) * len(ACTIONS),
        "png_dimensions_px": [round(width_in * dpi), round(height_in * dpi)],
        "outputs": [png, svg, pdf],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--out-prefix", type=Path, required=True)
    parser.add_argument("--dpi", type=int, default=400)
    args = parser.parse_args()
    input_path = args.input.resolve()
    prefix = args.out_prefix.resolve()
    paths = [prefix.with_suffix(ext) for ext in (".png", ".svg", ".pdf")]
    manifest_path = prefix.with_name(prefix.name + "_manifest.json")
    if args.dpi < 300:
        raise SystemExit("ultra-high-resolution output requires dpi >= 300")
    existing = [str(path) for path in [*paths, manifest_path] if path.exists()]
    if existing:
        raise SystemExit(f"refusing to overwrite existing outputs: {existing}")
    prefix.parent.mkdir(parents=True, exist_ok=True)

    result = plot(input_path, prefix, args.dpi)
    manifest = {
        "schema_version": 1,
        "run_type": "all_subject_all_source_overlay_v1",
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
            "curve": "one subject-level six-action profile for one source",
            "value": "subject/action/source median excursion from canonical scored bouts",
            "source_encoding": "color plus line style",
            "subject_encoding": "numeric marker text plus horizontal jitter",
            "error_bars": "omitted to preserve legibility across 91 simultaneous curves",
        },
        "counts": {
            "subjects": len(result["subjects"]),
            "sources": len(SOURCES),
            "curves": result["curves"],
            "categorical_points_expected": result["points_expected"],
        },
        "png_dimensions_px": result["png_dimensions_px"],
        "outputs": {
            path.name: {"sha256": sha256_file(path), "size_bytes": path.stat().st_size}
            for path in result["outputs"]
        },
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"counts": manifest["counts"], "png_dimensions_px": manifest["png_dimensions_px"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
