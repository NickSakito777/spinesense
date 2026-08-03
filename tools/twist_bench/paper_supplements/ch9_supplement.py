"""第 9 章补充分析 (2026-08-01)

三项补充结果,均由既有产物重新聚合,不重跑采集或坐标链:

  A. 跨人换算泛化 (leave-one-subject-out)
     输入 runs/corrected_bland_altman_2026-07-14_final2/paired_bout_peaks.csv
     统计口径复刻 corrected_bland_altman_plots.py 的 repeated_loa()
     (subject-balanced random-intercept variance decomposition)

  B. 块内相关系数的动作类结构
     输入 runs/mapping_repair_2026-07-13/C_corrected_uniform/validation/subjects/*.json
     该产物的 pooled_r 即正文 77 块相关系数的来源 (中位 0.920, 全距 0.390-0.991)

  C. 摆动分量的方向串扰与轴归属
     输入 ml_classify/features.csv (1,387 片段, 骶部至上段配对的 13 项特征)
     轴无关口径: 主分量与次大分量的能量占比, 因两条摆动分量的解剖归属跨参与者不固定

运行: python ch9_supplement.py
"""
from __future__ import annotations

import glob
import json
import math
import re
from collections import defaultdict
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]        # tools/twist_bench
RUNS = ROOT / "runs"
PAIRS = RUNS / "corrected_bland_altman_2026-07-14_final2" / "paired_bout_peaks.csv"
VALID = RUNS / "mapping_repair_2026-07-13" / "C_corrected_uniform" / "validation" / "subjects"
FEATS = ROOT / "ml_classify" / "features.csv"
OUT = ROOT / "runs" / "ch9_supplement"            # not published; created on run

ACTIONS = ["flexion", "extension", "left_bend", "right_bend", "left_twist", "right_twist"]
ZH = {
    "flexion": "前屈",
    "extension": "后伸",
    "left_bend": "左侧屈",
    "right_bend": "右侧屈",
    "left_twist": "左旋转",
    "right_twist": "右旋转",
}
BLOCK_ACTION = {"B1": "flexion", "B2": "extension", "B3": "left_bend",
                "B4": "right_bend", "B5": "left_twist", "B6": "right_twist"}


def repeated_loa(pairs):
    """逐字复刻 corrected_bland_altman_plots.py:repeated_loa 的偏倚与一致性界限口径。"""
    grouped = defaultdict(list)
    for subject, mocap, imu in pairs:
        grouped[subject].append(imu - mocap)
    arrays = [np.asarray(v, dtype=float) for v in grouped.values()]
    if not arrays:
        return None
    subject_means = np.asarray([float(np.mean(v)) for v in arrays])
    bias = float(np.mean(subject_means))
    within_ss = float(sum(np.sum(np.square(v - np.mean(v))) for v in arrays))
    within_df = sum(len(v) - 1 for v in arrays)
    within_var = within_ss / within_df if within_df > 0 else 0.0
    mean_var = float(np.var(subject_means, ddof=1)) if len(subject_means) > 1 else 0.0
    corrections = [float(np.var(v, ddof=1)) / len(v) for v in arrays if len(v) > 1]
    between_var = max(0.0, mean_var - (float(np.mean(corrections)) if corrections else 0.0))
    total_sd = math.sqrt(within_var + between_var)
    diffs = np.asarray([imu - mocap for _, mocap, imu in pairs])
    return {
        "n_subjects": len(grouped),
        "n_bouts": len(pairs),
        "bias_deg": bias,
        "within_subject_sd_deg": math.sqrt(within_var),
        "between_subject_sd_deg": math.sqrt(between_var),
        "lower_loa_deg": bias - 1.96 * total_sd,
        "upper_loa_deg": bias + 1.96 * total_sd,
        "rmse_deg": float(np.sqrt(np.mean(np.square(diffs)))),
        "mae_deg": float(np.mean(np.abs(diffs))),
    }


def part_a():
    d = pd.read_csv(PAIRS)
    out = {"baseline_within_block_loro": None, "loso_by_action": [], "loso_overall": None,
           "beta_between_subject_spread": []}

    scored = d.dropna(subset=["imu_loro_at_mocap_peak_deg"])
    base = repeated_loa([(r.subject, abs(r.mocap_peak_deg), abs(r.imu_loro_at_mocap_peak_deg))
                         for r in scored.itertuples()])
    out["baseline_within_block_loro"] = base

    rows = []
    for action in ACTIONS:
        sub = d[d.action == action]
        for subject in sorted(sub.subject.unique()):
            train = sub[sub.subject != subject]
            test = sub[sub.subject == subject]
            if len(train) < 3 or test.empty:
                continue
            beta, alpha = np.polyfit(train.imu_native_at_mocap_peak_deg.values,
                                     train.mocap_peak_deg.values, 1)
            pred = alpha + beta * test.imu_native_at_mocap_peak_deg.values
            for mocap, p in zip(test.mocap_peak_deg.values, pred):
                rows.append({"subject": subject, "action": action, "mocap": mocap,
                             "pred": p, "beta": beta, "alpha": alpha})
    loso = pd.DataFrame(rows)
    loso.to_csv(OUT / "loso_predictions.csv", index=False)

    for action in ACTIONS:
        g = loso[loso.action == action]
        st = repeated_loa([(r.subject, r.mocap, r.pred) for r in g.itertuples()])
        st["action"] = action
        out["loso_by_action"].append(st)
    out["loso_overall"] = repeated_loa(
        [(r.subject, abs(r.mocap), abs(r.pred)) for r in loso.itertuples()])

    per = d.groupby(["subject", "action"], as_index=False).loro_gain.median()
    for action in ACTIONS:
        g = per[per.action == action].loro_gain
        out["beta_between_subject_spread"].append({
            "action": action, "n_subjects": int(len(g)),
            "median": float(g.median()), "q1": float(g.quantile(0.25)),
            "q3": float(g.quantile(0.75)), "min": float(g.min()), "max": float(g.max()),
            "cv": float(g.std() / g.mean()),
        })
    return out


def part_b():
    rows = []
    for f in sorted(glob.glob(str(VALID / "T*.json"))):
        subject = re.search(r"(T\d+)_P", f).group(1)
        payload = json.loads(Path(f).read_text(encoding="utf-8"))

        def walk(node):
            if isinstance(node, dict):
                if "quality" in node and "corrected_metric" in node:
                    metric = node["corrected_metric"] or {}
                    rows.append({"subject": subject, "block": node.get("block"),
                                 "quality": node["quality"], "r": metric.get("pooled_r"),
                                 "n_reps": metric.get("n_reps")})
                for v in node.values():
                    walk(v)
            elif isinstance(node, list):
                for v in node:
                    walk(v)

        walk(payload)
    d = pd.DataFrame(rows).dropna(subset=["r"])
    d["action"] = d.block.str[:2].map(BLOCK_ACTION)
    d.to_csv(OUT / "block_pooled_r.csv", index=False)

    out = {"n_blocks": int(len(d)), "median": float(d.r.median()),
           "min": float(d.r.min()), "max": float(d.r.max()), "by_action": [],
           "by_quality": [], "low_blocks": []}
    for action in ACTIONS:
        g = d[d.action == action]
        out["by_action"].append({"action": action, "n": int(len(g)),
                                 "median": float(g.r.median()), "min": float(g.r.min()),
                                 "max": float(g.r.max()), "n_below_0.8": int((g.r < 0.8).sum())})
    for q, g in d.groupby("quality"):
        out["by_quality"].append({"quality": q, "n": int(len(g)), "median": float(g.r.median())})
    low = d[d.r < 0.8]
    out["n_below_0.8"] = int(len(low))
    out["below_0.8_action_counts"] = {ZH[k]: int(v) for k, v in
                                      low.action.value_counts().to_dict().items()}
    out["below_0.8_quality_counts"] = low.quality.value_counts().to_dict()
    out["low_blocks"] = low.nsmallest(10, "r")[
        ["subject", "block", "action", "quality", "r"]].to_dict("records")
    return out


def part_c():
    d = pd.read_csv(FEATS)
    frac = ["frac_x", "frac_y", "frac_tw"]
    d["dominant"] = d[frac].max(axis=1)
    d["second"] = d[frac].apply(lambda r: sorted(r)[-2], axis=1)
    d["dominant_axis"] = d[frac].idxmax(axis=1)
    swing = d[["frac_x", "frac_y"]]
    d["swing_ratio"] = swing.min(axis=1) / swing.max(axis=1)

    out = {"by_action": [], "dominant_axis_subjects": {}, "axis_note": None}
    for action in ACTIONS:
        g = d[d.label == action]
        per = g.groupby("subject")[["dominant", "second", "frac_tw", "swing_ratio"]].median()
        out["by_action"].append({
            "action": action, "n_segments": int(len(g)), "n_subjects": int(len(per)),
            "dominant_share": float(per.dominant.mean()),
            "second_share": float(per.second.mean()),
            "twist_share": float(per.frac_tw.mean()),
            "swing_ratio_subject_balanced": float(per.swing_ratio.mean()),
            "swing_ratio_min": float(per.swing_ratio.min()),
            "swing_ratio_max": float(per.swing_ratio.max()),
        })
    tab = d.groupby(["label", "dominant_axis"]).subject.nunique().unstack(fill_value=0)
    out["dominant_axis_subjects"] = tab.to_dict()

    flex = d[d.label == "flexion"].groupby("subject")[frac].median()
    out["axis_note"] = {
        "T02_dominant_axis": flex.loc["T02"].idxmax(),
        "others": flex.drop("T02").idxmax(axis=1).value_counts().to_dict(),
    }
    return out


def main():
    result = {"part_a_loso_generalisation": part_a(),
              "part_b_block_correlation_structure": part_b(),
              "part_c_swing_crosstalk": part_c()}
    (OUT / "ch9_supplement.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False, default=float) + "\n", encoding="utf-8")

    a = result["part_a_loso_generalisation"]
    print("=" * 72)
    print("A 跨人换算泛化")
    b0 = a["baseline_within_block_loro"]
    print(f"  块内留一重复(基线)  n={b0['n_bouts']}  偏倚={b0['bias_deg']:+.4f}°  "
          f"RMSE={b0['rmse_deg']:.4f}°  LoA=[{b0['lower_loa_deg']:.2f},{b0['upper_loa_deg']:.2f}]")
    ov = a["loso_overall"]
    print(f"  跨人换算(留一人)    n={ov['n_bouts']}  偏倚={ov['bias_deg']:+.4f}°  "
          f"RMSE={ov['rmse_deg']:.4f}°  LoA=[{ov['lower_loa_deg']:.2f},{ov['upper_loa_deg']:.2f}]")
    for st in a["loso_by_action"]:
        print(f"    {ZH[st['action']]:4s} n={st['n_bouts']:4d} 偏倚={st['bias_deg']:+7.3f}° "
              f"RMSE={st['rmse_deg']:6.3f}°")

    b = result["part_b_block_correlation_structure"]
    print("=" * 72)
    print(f"B 块内相关结构  n={b['n_blocks']} 中位={b['median']:.4f} "
          f"全距=[{b['min']:.4f},{b['max']:.4f}]")
    for row in b["by_action"]:
        print(f"    {ZH[row['action']]:4s} n={row['n']:2d} 中位={row['median']:.3f} "
              f"全距=[{row['min']:.3f},{row['max']:.3f}] r<0.8: {row['n_below_0.8']}")
    print(f"  r<0.8 共 {b['n_below_0.8']} 块, 动作分布 {b['below_0.8_action_counts']}")

    c = result["part_c_swing_crosstalk"]
    print("=" * 72)
    print("C 摆动分量串扰(轴无关口径)")
    for row in c["by_action"]:
        print(f"    {ZH[row['action']]:4s} 主分量={row['dominant_share']:.3f} "
              f"次大={row['second_share']:.3f} 摆动内比值={row['swing_ratio_subject_balanced']:.3f} "
              f"人间全距=[{row['swing_ratio_min']:.3f},{row['swing_ratio_max']:.3f}]")
    print(f"  轴归属: T02 前屈主轴={c['axis_note']['T02_dominant_axis']}, "
          f"其余 {c['axis_note']['others']}")
    print("=" * 72)
    print(f"已写出 {OUT/'ch9_supplement.json'}")


if __name__ == "__main__":
    main()
