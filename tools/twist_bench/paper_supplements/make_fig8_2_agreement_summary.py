#!/usr/bin/env python3
"""Figure 8.2 — per-action agreement before and after within-block conversion.

Replaces the eight-column table: bias and limits of agreement are lengths on a
common axis, so the compression from raw scale (a) to converted residual (b) is
read directly rather than reconstructed from numbers. Panel c carries the gain
that explains why the compression works.

Every value is read from the frozen run, never restated here, so the figure
cannot drift from the appendix table.

Typography and palette follow the Chapter 5 figure family (figstyle.py).
"""

from __future__ import annotations

import csv
import statistics as st
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[2]
RUN = (PROJECT / "tools" / "twist_bench" / "runs"
       / "corrected_bland_altman_2026-07-14_final2")

# same-hue pair: both series are the system, light-to-dark marks the processing
# stage. Reusing blue/orange here would read as system-vs-reference (Figure 8.1).
NATIVE = "#56B4E9"   # Okabe-Ito sky blue — raw scale
CONV = "#0072B2"     # Okabe-Ito blue — after within-block conversion
INK = "#1A1A1A"
GUIDE = "#B8B8B8"

plt.rcParams.update({
    "font.family": "Times New Roman",
    "mathtext.fontset": "stix",
    "svg.fonttype": "none",
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
    "font.size": 9.5,
    "axes.linewidth": 0.8,
    "axes.edgecolor": INK,
    "axes.labelcolor": INK,
    "text.color": INK,
    "xtick.color": INK,
    "ytick.color": INK,
    "legend.frameon": False,
    "figure.facecolor": "white",
    "axes.facecolor": "white",
})

MM = 1 / 25.4
# 88 mm rather than 76: at the larger type size the value labels of the two
# rotation rows collided with the dark interval of the row above them
FIG_W, FIG_H = 175 * MM, 88 * MM

ACTIONS = [
    ("flexion", "Flexion"),
    ("extension", "Extension"),
    ("left_bend", "Left lateral bend"),
    ("right_bend", "Right lateral bend"),
    ("left_twist", "Left axial rotation"),
    ("right_twist", "Right axial rotation"),
    ("overall_absolute_amplitude", "All actions (absolute)"),
]
POOLED = "overall_absolute_amplitude"
ZOOM = (-7.0, 6.0)  # panel b range, marked on panel a
LABEL_X = -0.475    # row-label column start, in panel-a axes fraction


def load() -> list[dict]:
    stats = {
        (r["mode"], r["action"]): r
        for r in csv.DictReader((RUN / "bland_altman_stats.csv").open(encoding="utf-8"))
        if r["analysis_set"] == "all_scored"
    }
    peaks = list(csv.DictReader((RUN / "paired_bout_peaks.csv").open(encoding="utf-8")))

    rows = []
    for key, label in ACTIONS:
        gains = [
            float(r["loro_gain"]) for r in peaks
            if r["loro_gain"] not in ("", "nan")
            and (key == POOLED or r["action"] == key)
        ]
        row = {"key": key, "label": label, "beta": st.median(gains)}
        for mode, tag in (("native", "nat"), ("loro", "conv")):
            s = stats[(mode, key)]
            row[f"{tag}_bias"] = float(s["bias_deg"])
            row[f"{tag}_lo"] = float(s["lower_loa_deg"])
            row[f"{tag}_hi"] = float(s["upper_loa_deg"])
            row[f"{tag}_n"] = int(s["n_bouts"])
        rows.append(row)
    return rows


def fmt(value: float, digits: int = 2, sign: bool = True) -> str:
    """Format with a real minus sign (U+2212), matching the axis ticks.

    matplotlib renders tick labels with U+2212 but an f-string emits an ASCII
    hyphen, which puts two different minus glyphs in one figure.
    """
    text = f"{value:+.{digits}f}" if sign else f"{value:.{digits}f}"
    return text.replace("-", "−")


def interval(ax, y, lo, hi, bias, colour, cap=0.15, lw=2.0, ms=4.4) -> None:
    ax.hlines(y, lo, hi, color=colour, lw=lw, zorder=2)
    for x in (lo, hi):
        ax.plot([x, x], [y - cap, y + cap], color=colour, lw=0.9, zorder=2)
    ax.plot([bias], [y], "o", color=colour, ms=ms,
            mec="white", mew=0.7, zorder=3)


def style(ax, rows, xlim, xlabel, zero=True) -> None:
    if zero:
        ax.axvline(0, color=GUIDE, lw=0.7, ls=(0, (3, 2)), zorder=1)
    ax.set_xlim(*xlim)
    ax.set_ylim(-len(rows) + 0.4, 0.75)
    ax.set_yticks([])
    ax.set_xlabel(xlabel, fontsize=9.1, fontweight="bold")
    ax.tick_params(length=2.4, pad=1.6, labelsize=8.5)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    # the pooled row mixes action categories; separate it from the per-action
    # rows. Sits above the midpoint because each row carries its label on top,
    # so the free space between rows is not centred between them.
    ax.axhline(-len(rows) + 1.66, color=GUIDE, lw=0.6, ls=(0, (2, 2)), zorder=1)


def main() -> int:
    rows = load()
    ys = [-i for i in range(len(rows))]

    fig = plt.figure(figsize=(FIG_W, FIG_H))
    grid = fig.add_gridspec(
        1, 3, width_ratios=[1.18, 0.60, 0.86],
        left=0.175, right=0.985, top=0.86, bottom=0.145, wspace=0.18,
    )
    # order follows the narrative of 9.2: the raw-vs-converted contrast, then the
    # gain that produced it, then the magnified residuals
    ax_nat, ax_beta, ax_conv = (fig.add_subplot(grid[0, i]) for i in range(3))

    # panel a carries both series on one scale, so the compression is a change in
    # bar length rather than something the reader must infer across panels
    for row, y in zip(rows, ys):
        interval(ax_nat, y + 0.19, row["nat_lo"], row["nat_hi"], row["nat_bias"],
                 NATIVE, cap=0.11)
        ax_nat.annotate(fmt(row["nat_bias"]), (row["nat_bias"], y + 0.34),
                        ha="center", va="bottom", fontsize=8.1, color=INK)
        interval(ax_nat, y - 0.19, row["conv_lo"], row["conv_hi"], row["conv_bias"],
                 CONV, cap=0.11, lw=1.8, ms=3.4)

        interval(ax_conv, y, row["conv_lo"], row["conv_hi"], row["conv_bias"], CONV)
        ax_conv.annotate(fmt(row["conv_bias"]), (row["conv_bias"], y + 0.20),
                         ha="center", va="bottom", fontsize=8.1, color=INK)

        pooled = row["key"] == POOLED
        ax_beta.plot([row["beta"]], [y], "o", ms=4.6, color=CONV,
                     mfc="white" if pooled else CONV, mew=1.0, zorder=3)
        ax_beta.annotate(fmt(row["beta"], digits=3, sign=False), (row["beta"], y + 0.20),
                         ha="center", va="bottom", fontsize=8.1, color=INK)

    # the shaded band marks the span panel c magnifies; the cross-panel guide
    # lines are dropped because the two panels are no longer adjacent
    ax_nat.axvspan(*ZOOM, color=CONV, alpha=0.10, lw=0, zorder=0)

    style(ax_nat, rows, (-26, 70), "Difference: system − reference (°)")
    style(ax_conv, rows, ZOOM, "Converted residual (°)")
    style(ax_beta, rows, (0.16, 1.35), r"Conversion gain $\beta$", zero=False)
    ax_beta.axvline(1.0, color=GUIDE, lw=0.7, ls=(0, (3, 2)), zorder=1)
    ax_beta.set_xticks([0.2, 0.5, 1.0])

    # beta is the quantity Section 7.3.4 defines, but 0.195 says nothing on its
    # own; the mirrored axis gives the same points in "how many times larger"
    top = ax_beta.secondary_xaxis("top", functions=(lambda b: 1.0 / np.maximum(b, 1e-6),
                                                    lambda m: 1.0 / np.maximum(m, 1e-6)))
    top.set_xticks([5, 2, 1])
    top.set_xticklabels(["5×", "2×", "1×"])
    top.set_xlabel("System reading / reference", fontsize=8.7, labelpad=3)
    top.tick_params(length=2.4, pad=1.4, labelsize=8.5, colors=INK)
    top.spines["top"].set_color(INK)

    handles = [
        Line2D([], [], color=NATIVE, lw=2.0, marker="o", ms=4.4,
               mec="white", mew=0.7, label="Raw scale"),
        Line2D([], [], color=CONV, lw=1.8, marker="o", ms=3.4,
               mec="white", mew=0.7, label="After conversion"),
    ]
    # the legend sits where panel a's title would go: with both series on one
    # axis, naming the two colours says more than a title could
    ax_nat.legend(handles=handles, loc="lower left", bbox_to_anchor=(0.0, 1.0),
                  ncol=2, fontsize=8.8, handlelength=1.9, columnspacing=1.6,
                  borderaxespad=0.15, handletextpad=0.5)

    # left-aligned: the labels share a start, so the column reads as a list
    # rather than as ragged text pushed up against the axis
    for row, y in zip(rows, ys):
        ax_nat.annotate(row["label"], (LABEL_X, y), xycoords=("axes fraction", "data"),
                        ha="left", va="center", fontsize=8.9, fontweight="bold",
                        color=INK)

    ax_conv.set_title("After within-block conversion", fontsize=9.1, pad=9, color=INK)
    # panel b's header is the mirrored axis itself, so it takes no title
    # one height for all three: panel b's mirrored axis is taller than a legend
    # or a title, and letters at different heights read as a layout error
    for ax, tag in ((ax_nat, "a"), (ax_beta, "b"), (ax_conv, "c")):
        ax.text(0.0, 1.20, tag, transform=ax.transAxes, fontsize=11.5,
                fontweight="bold", ha="left", va="bottom", color=INK)

    ax_conv.annotate("magnified from a", xy=(0.985, 1.005), xycoords="axes fraction",
                     ha="right", va="bottom", fontsize=8.1, color="#606060")

    stem = HERE / "fig8_2_agreement_summary"
    for ext, kw in (("svg", {}), ("pdf", {}), ("tiff", {"dpi": 600}), ("png", {"dpi": 400})):
        fig.savefig(f"{stem}.{ext}", bbox_inches="tight", facecolor="white", **kw)

    print(f"{'action':26s} {'native bias':>12s} {'native LoA':>19s} "
          f"{'conv bias':>10s} {'conv LoA':>17s} {'beta':>7s}  n")
    for r in rows:
        print(f"{r['label']:26s} {r['nat_bias']:+12.2f} "
              f"{r['nat_lo']:+8.2f}..{r['nat_hi']:+8.2f} {r['conv_bias']:+10.2f} "
              f"{r['conv_lo']:+7.2f}..{r['conv_hi']:+7.2f} {r['beta']:7.3f}  "
              f"{r['nat_n']}/{r['conv_n']}")
    off = [r["label"] for r in rows
           if not (ZOOM[0] <= r["conv_lo"] and r["conv_hi"] <= ZOOM[1])]
    if off:
        raise SystemExit(f"panel b window clips: {off}")
    print("saved", stem, "(svg/pdf/tiff@600/png@400)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
