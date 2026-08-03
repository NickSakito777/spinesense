from __future__ import annotations

import argparse
import html
import sys
from pathlib import Path

from twist_bench_v0 import integrate_twist, load_pairs, parse_axis, summarize


ROOT = Path(__file__).resolve().parent
DEFAULT_CUES = [
    (0.0, "hold 0"),
    (8.0, "move +30"),
    (13.0, "hold +30"),
    (18.0, "return 0"),
    (23.0, "hold 0"),
    (28.0, "move -30"),
    (33.0, "hold -30"),
    (38.0, "return 0"),
]


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate a simple SpineSense twist-bench HTML visualization.")
    parser.add_argument("--input", type=Path, required=True, help="Raw serial log captured by capture_trial.py.")
    parser.add_argument("--parent", default="IMU1", help="Parent/reference IMU id. Default: IMU1.")
    parser.add_argument("--child", default="IMU2", help="Child/moving IMU id. Default: IMU2.")
    parser.add_argument("--axis", default="0,0,1", help="Twist axis. Default: 0,0,1.")
    parser.add_argument("--bias-seconds", type=float, default=8.0, help="Initial still window. Default: 8 s.")
    parser.add_argument("--tare-seconds", type=float, default=8.0, help="Initial tare window. Default: 8 s.")
    parser.add_argument("--target-deg", type=float, default=30.0, help="Target twist angle. Default: 30 deg.")
    parser.add_argument("--output", type=Path, help="Output HTML path.")
    args = parser.parse_args()

    axis = parse_axis(args.axis)
    pairs = load_pairs(args.input, args.parent, args.child, max_pair_dt_s=0.025)
    samples, detail = integrate_twist(pairs, axis, args.bias_seconds, args.tare_seconds)
    summary = summarize(
        samples,
        detail,
        parent=args.parent,
        child=args.child,
        axis=axis,
        drift_start_s=None,
        drift_end_s=None,
        return_window_seconds=2.0,
    )

    output = args.output or (ROOT / "plots" / f"{args.input.stem}_visual.html")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(render_html(args, samples, summary), encoding="utf-8")
    print(output)
    return 0


def render_html(args: argparse.Namespace, samples, summary: dict[str, object]) -> str:
    width, height = 1100, 560
    left, right, top, bottom = 78, 34, 54, 78
    plot_w = width - left - right
    plot_h = height - top - bottom

    xs = [sample.t_s for sample in samples]
    ys = [sample.twist_deg for sample in samples]
    target = float(args.target_deg)

    x_min, x_max = min(xs), max(xs)
    y_min = min(min(ys), -target) - 8
    y_max = max(max(ys), target) + 8
    if x_max <= x_min:
        x_max = x_min + 1
    if y_max <= y_min:
        y_max = y_min + 1

    def x_px(x: float) -> float:
        return left + (x - x_min) / (x_max - x_min) * plot_w

    def y_px(y: float) -> float:
        return top + (y_max - y) / (y_max - y_min) * plot_h

    points = " ".join(f"{x_px(x):.2f},{y_px(y):.2f}" for x, y in zip(xs, ys))
    target_lines = "\n".join(
        line_with_label(left, left + plot_w, y_px(value), label, cls)
        for value, label, cls in [
            (target, f"+{target:g} deg target", "target-pos"),
            (0.0, "0 deg", "zero"),
            (-target, f"-{target:g} deg target", "target-neg"),
        ]
    )
    cue_lines = "\n".join(
        f'<line x1="{x_px(t):.2f}" y1="{top}" x2="{x_px(t):.2f}" y2="{top + plot_h}" class="cue"/>'
        f'<text x="{x_px(t) + 4:.2f}" y="{top + 16}" class="cue-label">{html.escape(label)}</text>'
        for t, label in DEFAULT_CUES
        if x_min <= t <= x_max
    )
    grid_lines = "\n".join(
        f'<line x1="{left}" y1="{top + plot_h * frac:.2f}" x2="{left + plot_w}" y2="{top + plot_h * frac:.2f}" class="grid"/>'
        for frac in (0.25, 0.5, 0.75)
    )

    max_pos = max(ys)
    max_neg = min(ys)
    final = ys[-1]
    target_status = classify_target(max_pos, max_neg, target)

    cards = [
        ("Parent -> child", f"{summary['parent']} -> {summary['child']}"),
        ("Axis", str(summary["axis"])),
        ("Max +", f"{max_pos:.1f} deg"),
        ("Max -", f"{max_neg:.1f} deg"),
        ("Final", f"{final:.1f} deg"),
        ("Target read", target_status),
        ("Samples", str(summary["sample_count"])),
        ("Median Hz", fmt(summary["sample_rate_hz_median"])),
    ]
    card_html = "\n".join(
        f'<div class="card"><div class="k">{html.escape(k)}</div><div class="v">{html.escape(v)}</div></div>'
        for k, v in cards
    )

    title = f"{args.input.name} | relative twist {summary['parent']} -> {summary['child']}"
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{html.escape(title)}</title>
  <style>
    :root {{ color-scheme: light; font-family: Segoe UI, Arial, sans-serif; }}
    body {{ margin: 0; background: #f6f7f9; color: #1f2328; }}
    main {{ max-width: 1180px; margin: 0 auto; padding: 24px; }}
    h1 {{ font-size: 22px; margin: 0 0 16px; }}
    .cards {{ display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 10px; margin-bottom: 16px; }}
    .card {{ background: white; border: 1px solid #d9dee7; border-radius: 8px; padding: 10px 12px; }}
    .k {{ font-size: 12px; color: #687385; margin-bottom: 4px; }}
    .v {{ font-size: 17px; font-weight: 700; }}
    .plot {{ background: white; border: 1px solid #d9dee7; border-radius: 8px; overflow-x: auto; }}
    svg {{ display: block; width: 100%; min-width: 920px; height: auto; }}
    .grid {{ stroke: #e6e9ef; stroke-width: 1; }}
    .axis {{ stroke: #20242a; stroke-width: 1.4; }}
    .trace {{ fill: none; stroke: #155eef; stroke-width: 2.5; }}
    .target-pos, .target-neg {{ stroke: #d92d20; stroke-width: 1.5; stroke-dasharray: 7 5; }}
    .zero {{ stroke: #596579; stroke-width: 1.2; stroke-dasharray: 4 5; }}
    .cue {{ stroke: #f79009; stroke-width: 1; opacity: 0.55; }}
    text {{ fill: #1f2328; }}
    .small {{ font-size: 13px; fill: #596579; }}
    .cue-label {{ font-size: 11px; fill: #9a5b00; }}
    .line-label {{ font-size: 12px; fill: #5a6475; }}
    .note {{ margin-top: 14px; color: #4d5768; line-height: 1.45; }}
    code {{ background: #edf0f5; padding: 1px 4px; border-radius: 4px; }}
  </style>
</head>
<body>
<main>
  <h1>{html.escape(title)}</h1>
  <section class="cards">{card_html}</section>
  <section class="plot">
    <svg width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-label="relative twist plot">
      <rect width="100%" height="100%" fill="#ffffff"/>
      {grid_lines}
      {target_lines}
      {cue_lines}
      <line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_h}" class="axis"/>
      <line x1="{left}" y1="{top + plot_h}" x2="{left + plot_w}" y2="{top + plot_h}" class="axis"/>
      <polyline points="{points}" class="trace"/>
      <text x="{left + plot_w / 2:.1f}" y="{height - 24}" text-anchor="middle" class="small">time (s)</text>
      <text x="24" y="{top + plot_h / 2:.1f}" transform="rotate(-90 24 {top + plot_h / 2:.1f})" text-anchor="middle" class="small">relative twist (deg)</text>
      <text x="{left}" y="{top + plot_h + 24}" class="small">{x_min:.1f}s</text>
      <text x="{left + plot_w}" y="{top + plot_h + 24}" text-anchor="end" class="small">{x_max:.1f}s</text>
      <text x="{left - 10}" y="{top + 4}" text-anchor="end" class="small">{y_max:.0f}</text>
      <text x="{left - 10}" y="{top + plot_h}" text-anchor="end" class="small">{y_min:.0f}</text>
    </svg>
  </section>
  <p class="note">This is the gyro-only v0 visualization. If the trace does not return to <code>0 deg</code>, that can mean the block did not physically return to neutral, the selected axis is not the real pillar axis, or the first still window contained motion.</p>
</main>
</body>
</html>
"""


def line_with_label(x1: float, x2: float, y: float, label: str, cls: str) -> str:
    return (
        f'<line x1="{x1:.2f}" y1="{y:.2f}" x2="{x2:.2f}" y2="{y:.2f}" class="{cls}"/>'
        f'<text x="{x2 - 8:.2f}" y="{y - 5:.2f}" text-anchor="end" class="line-label">{html.escape(label)}</text>'
    )


def classify_target(max_pos: float, max_neg: float, target: float) -> str:
    pos_err = max_pos - target
    neg_err = abs(max_neg) - target
    return f"+ target err {pos_err:+.1f} deg / - target err {neg_err:+.1f} deg"


def fmt(value: object) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.1f}"
    return str(value)


if __name__ == "__main__":
    sys.exit(main())
