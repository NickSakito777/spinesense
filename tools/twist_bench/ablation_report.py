"""Render ablation_results.json into Markdown tables for the dissertation and KB report.

Read-only over the results file. Emits: a ranked subset table, a per-sensor-count summary,
the six-class recall breakdown for the subsets that matter, and the pre-specified contrasts.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

CLASSES = ["flexion", "extension", "left_bend", "right_bend", "left_twist", "right_twist"]
SHORT = {"flexion": "Flex", "extension": "Ext", "left_bend": "LBend",
         "right_bend": "RBend", "left_twist": "LTw", "right_twist": "RTw"}


def fmt(x: float) -> str:
    return f"{x:.4f}"


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Render ablation results as Markdown.")
    p.add_argument("--run-dir", type=Path, required=True)
    args = p.parse_args(argv)
    run_dir = args.run_dir.resolve()
    data = json.loads((run_dir / "ablation_results.json").read_text(encoding="utf-8"))
    subs = data["subsets"]
    out: list[str] = []

    real = {k: v for k, v in subs.items() if k != "legacy_frozen_block"}
    ranked = sorted(real.items(), key=lambda kv: -kv[1]["logistic"]["accuracy"])

    out.append("## 全 31 子集排名（Logistic LOSO，降序）\n")
    out.append("| # | 传感器子集 | k | 维度 | Logistic | RF | macro-F1 |")
    out.append("|---|---|---|---|---|---|---|")
    for i, (key, e) in enumerate(ranked, 1):
        out.append(
            f"| {i} | `{key}` | {e['n_sensors']} | {e['n_features']} | "
            f"{fmt(e['logistic']['accuracy'])} | {fmt(e['random_forest']['accuracy'])} | "
            f"{fmt(e['logistic']['macro_f1'])} |"
        )
    lg = subs["legacy_frozen_block"]
    out.append(
        f"| — | `legacy_frozen_block`（冻结 13 特征） | 3 | {lg['n_features']} | "
        f"{fmt(lg['logistic']['accuracy'])} | {fmt(lg['random_forest']['accuracy'])} | "
        f"{fmt(lg['logistic']['macro_f1'])} |"
    )

    out.append("\n## 按传感器数量汇总\n")
    out.append("| k | 子集数 | 最佳 Logistic | 最佳子集 | 最差 Logistic | 中位 |")
    out.append("|---|---|---|---|---|---|")
    for k in range(1, 6):
        grp = [(key, e) for key, e in real.items() if e["n_sensors"] == k]
        accs = sorted(e["logistic"]["accuracy"] for _, e in grp)
        best_key, best = max(grp, key=lambda kv: kv[1]["logistic"]["accuracy"])
        med = accs[len(accs) // 2] if len(accs) % 2 else (accs[len(accs) // 2 - 1] + accs[len(accs) // 2]) / 2
        out.append(
            f"| {k} | {len(grp)} | {fmt(best['logistic']['accuracy'])} | `{best_key}` | "
            f"{fmt(accs[0])} | {fmt(med)} |"
        )

    out.append("\n## 六类 recall（关键子集，Logistic）\n")
    focus = ["sacrum+lower+mid+upper+sternum", "sacrum+upper+sternum", "legacy_frozen_block"]
    focus += [k for k, _ in ranked if subs[k]["n_sensors"] == 1][:2]
    focus += [k for k, _ in ranked if subs[k]["n_sensors"] == 2][:1]
    seen: set[str] = set()
    focus = [f for f in focus if f in subs and not (f in seen or seen.add(f))]
    out.append("| 子集 | " + " | ".join(SHORT[c] for c in CLASSES) + " | 总体 |")
    out.append("|---" * (len(CLASSES) + 2) + "|")
    for key in focus:
        r = subs[key]["logistic"]["per_class_recall"]
        out.append(
            f"| `{key}` | " + " | ".join(f"{r[c]:.3f}" for c in CLASSES) +
            f" | {fmt(subs[key]['logistic']['accuracy'])} |"
        )

    out.append("\n## 预设配对对比（Wilcoxon signed-rank，13 名受试者）\n")
    out.append("| 对比 | 模型 | A | B | A 池化 | B 池化 | 中位差 | p |")
    out.append("|---|---|---|---|---|---|---|---|")
    for name, c in data["contrasts"].items():
        label, model = name.split("::")
        out.append(
            f"| {label} | {model} | `{c['a']}` | `{c['b']}` | {fmt(c['a_pooled'])} | "
            f"{fmt(c['b_pooled'])} | {c['median_delta']:+.4f} | {c['p_value']:.4f} |"
        )

    out.append(
        f"\nchance = {data['chance']:.4f}；多数类 = {data['majority']:.4f}；"
        f"n = {data['n_rows']} bouts / {data['n_subjects']} 人。"
    )

    text = "\n".join(out) + "\n"
    (run_dir / "ablation_tables.md").write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
