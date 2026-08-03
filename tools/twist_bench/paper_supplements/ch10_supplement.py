"""第 10 章补充分析 (2026-08-02)

三项补充结果,均由既有产物重新聚合,不重跑采集、坐标链或锁定评估:

  A. 误判的结构 (集中度 / 幅值 / 质量分层 / 跨支路对照)
     输入 runs/locked_track_a_2026-07-21/primary/outer_predictions.csv (逐片段预测)
          ml_classify/features.csv (逐片段峰值与时长)
          runs/ch9_supplement_2026-08-01/block_pooled_r.csv (块级测量一致性)

  B. 特征承重 (leave-one-subject-out 分组置换重要性)
     输入 runs/allpairs_2026-07-21/features_model.csv (1,387 片段 x 130 特征)
     管线与超参逐项复刻 locked_track_a/core.py:FittedProcedure 与全 13 折选中的
     logistic:00 = {C: 0.01, family: l2}; 评分复刻 participant_first_macro_f1

  C. 集中失败块的方向特征诊断

复刻校验: 本脚本重跑 13 折 LOSO 得到的参与者等权宏平均 F1 与逐片段预测,
均与锁定运行逐位一致 (0.9544739; 64 个误判片段完全相同)。

运行: python ch10_supplement.py
"""
from __future__ import annotations

import json
from collections import Counter
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats
from sklearn.feature_selection import VarianceThreshold
from sklearn.impute import SimpleImputer
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import f1_score
from sklearn.preprocessing import StandardScaler

TB = Path(__file__).resolve().parents[1]          # tools/twist_bench
RUNS = TB / "runs"
HERE = RUNS / "ch10_supplement"                   # not published; created on run
FEATS = RUNS / "allpairs_2026-07-21" / "features_model.csv"
PEAKS = TB / "ml_classify" / "features.csv"
PRED = RUNS / "locked_track_a_2026-07-21" / "primary" / "outer_predictions.csv"
BLOCK_R = RUNS / "ch9_supplement_2026-08-01" / "block_pooled_r.csv"

SEED = 20260721
N_PERM = 20
ACTIONS = ["flexion", "extension", "left_bend", "right_bend", "left_twist", "right_twist"]
ZH = dict(zip(ACTIONS, ["前屈", "后伸", "左侧屈", "右侧屈", "左旋转", "右旋转"]))
PLANE = {"flexion": "sagittal", "extension": "sagittal",
         "left_bend": "frontal", "right_bend": "frontal",
         "left_twist": "transverse", "right_twist": "transverse"}
OPPOSITE = {"flexion": "extension", "extension": "flexion",
            "left_bend": "right_bend", "right_bend": "left_bend",
            "left_twist": "right_twist", "right_twist": "left_twist"}


def subject_only_weights(subjects: np.ndarray) -> np.ndarray:
    """复刻 core.py:compute_training_weights(scheme='subject_only')。"""
    _, inverse, counts = np.unique(subjects, return_inverse=True, return_counts=True)
    raw = 1.0 / counts[inverse].astype(np.float64)
    return raw / float(np.mean(raw))


def macro_f1_fixed6(y_true, y_pred) -> float:
    """复刻 core.py 的 fixed6_macro_f1: 六类固定标签集上的等权平均。"""
    return float(np.mean(f1_score(y_true, y_pred, labels=list(range(6)),
                                  average=None, zero_division=0.0)))


def fit_fold(X, y, w):
    imp = SimpleImputer(strategy="median")
    var = VarianceThreshold(threshold=0.0)
    sca = StandardScaler()
    A = sca.fit_transform(var.fit_transform(imp.fit_transform(X)))
    est = LogisticRegression(C=0.01, solver="lbfgs", l1_ratio=0.0, max_iter=10000,
                             tol=1e-4, class_weight=None, random_state=SEED)
    est.fit(A, y, sample_weight=w)
    return est, (lambda M: sca.transform(var.transform(imp.transform(M))))


def main() -> None:
    f = pd.read_csv(FEATS)
    pk = pd.read_csv(PEAKS)
    pr = pd.read_csv(PRED)
    pr = pr[pr.model == "logistic"].sort_values("source_row_index").reset_index(drop=True)
    assert (f.subject.values == pr.outer_test_subject.values).all()
    assert (f.y.values == pr.y_true.values).all()
    assert (pk.subject.values == f.subject.values).all()

    fcols = [c for c in f.columns if c.startswith("pair_")]
    X = f[fcols].to_numpy(dtype=np.float64)
    y = f.y.to_numpy()
    subj = f.subject.to_numpy()
    subjects = sorted(set(subj))
    out: dict = {}

    # ---------- 复刻校验 + Part B 的折内模型 ----------
    pair_names = sorted({c.split("__")[0] for c in fcols})
    fam_names = sorted({c.split("__")[1] for c in fcols})
    groups = {f"pair::{p}": [i for i, c in enumerate(fcols) if c.split("__")[0] == p]
              for p in pair_names}
    groups.update({f"family::{k}": [i for i, c in enumerate(fcols) if c.split("__")[1] == k]
                   for k in fam_names})
    groups["block::direction"] = [i for i, c in enumerate(fcols)
                                  if c.split("__")[1].startswith(("sign_", "dir_"))]
    groups["block::magnitude"] = [i for i, c in enumerate(fcols)
                                  if c.split("__")[1].startswith(("frac_", "logr_", "tw_dom"))]

    per_subject, preds = {}, np.empty_like(y)
    drops = {g: [] for g in groups}
    for s in subjects:
        te = subj == s
        tr = ~te
        est, tf = fit_fold(X[tr], y[tr], subject_only_weights(subj[tr]))
        Xte, yte = X[te], y[te]
        base = macro_f1_fixed6(yte, est.predict(tf(Xte)))
        preds[te] = est.predict(tf(Xte))
        per_subject[s] = base
        rng = np.random.default_rng(SEED)
        for gname, idx in groups.items():
            vals = []
            for _ in range(N_PERM):
                Xp = Xte.copy()
                Xp[:, idx] = Xp[rng.permutation(Xp.shape[0])][:, idx]
                vals.append(base - macro_f1_fixed6(yte, est.predict(tf(Xp))))
            drops[gname].append(float(np.mean(vals)))

    replicated = float(np.mean(list(per_subject.values())))
    out["replication_check"] = {
        "participant_equal_macro_f1": replicated,
        "locked_value": 0.9544739220062376,
        "identical_to_locked_predictions": bool((preds == pr.y_pred.to_numpy()).all()),
        "n_errors": int((preds != y).sum()),
        "per_subject_macro_f1": {k: round(v, 6) for k, v in per_subject.items()},
    }

    # ---------- Part A: 误判结构 ----------
    d = pd.DataFrame({
        "subject": subj, "block": pk.block.values, "label_true": f.label.values,
        "label_pred": [ACTIONS[i] for i in preds], "quality": f.quality.values,
        "correct": preds == y, "amp": np.abs(pk.mocap_peak.values),
        "dur_s": pk.dur_s.values,
    })
    d["err_type"] = np.where(d.correct, "correct",
                             np.where(d.label_pred == d.label_true.map(OPPOSITE),
                                      "within_plane_opposite", "cross_plane"))
    d["amp_z"] = d.groupby(["subject", "label_true"]).amp.transform(
        lambda v: (v - v.median()) / (v.std(ddof=1) if v.std(ddof=1) > 0 else np.nan))
    err = d[~d.correct]

    qual = []
    for q, g in d.groupby("quality"):
        qual.append({"quality": q, "n_bouts": int(len(g)),
                     "n_errors": int((~g.correct).sum()),
                     "error_rate": float((~g.correct).mean())})

    blk = d.groupby(["subject", "block"]).agg(
        n=("correct", "size"), n_err=("correct", lambda v: int((~v).sum())),
        action=("label_true", "first"), quality=("quality", "first")).reset_index()
    blk["err_rate"] = blk.n_err / blk.n
    top_blocks = blk.sort_values("n_err", ascending=False).head(5)

    out["part_a_error_structure"] = {
        "n_bouts": int(len(d)), "n_errors": int(len(err)),
        "error_rate": float(len(err) / len(d)),
        "error_type_counts": {k: int(v) for k, v in Counter(err.err_type).items()},
        "amplitude": {
            "correct_median_deg": float(d[d.correct].amp.median()),
            "error_median_deg": float(err.amp.median()),
            "correct_median_z": float(d[d.correct].amp_z.median()),
            "error_median_z": float(err.amp_z.median()),
            "note": "z 为参与者×动作类内标准化; 中位数接近零表示误判片段的幅值与正确片段无系统差异",
        },
        "by_quality": qual,
        "concentration": {
            "top2_blocks": [{"subject": r.subject, "block": r.block, "action": r.action,
                             "quality": r.quality, "n": int(r.n), "n_err": int(r.n_err),
                             "err_rate": float(r.err_rate)}
                            for r in top_blocks.itertuples() if r.n_err >= 10],
            "share_of_all_errors_top2": float(
                top_blocks.n_err.head(2).sum() / len(err)),
            "by_subject": {s: {"n": int(g.n.sum()), "n_err": int(g.n_err.sum())}
                           for s, g in blk.groupby("subject")},
        },
    }

    # 跨支路对照
    br = pd.read_csv(BLOCK_R)
    m = blk.merge(br[["subject", "block", "r", "n_reps"]], on=["subject", "block"], how="inner")

    def contrast(mm):
        lo, hi = mm[mm.r < 0.8], mm[mm.r >= 0.8]
        if not len(lo) or not len(hi):
            return None
        table = [[int(lo.n_err.sum()), int(lo.n.sum() - lo.n_err.sum())],
                 [int(hi.n_err.sum()), int(hi.n.sum() - hi.n_err.sum())]]
        odds, p = stats.fisher_exact(table)
        return {"n_blocks_low": int(len(lo)), "n_blocks_high": int(len(hi)),
                "err_rate_low_r": float(lo.n_err.sum() / lo.n.sum()),
                "err_rate_high_r": float(hi.n_err.sum() / hi.n.sum()),
                "odds_ratio": float(odds), "fisher_p": float(p)}

    sp_block = stats.spearmanr(m.r, m.err_rate)
    out["part_a_cross_branch"] = {
        "n_blocks_joined": int(len(m)),
        "spearman_r_vs_error_rate": {"rho": float(sp_block.statistic),
                                     "p": float(sp_block.pvalue)},
        "dichotomised_all": contrast(m),
        "dichotomised_excluding_T09": contrast(m[m.subject != "T09"]),
        "dichotomised_excluding_T09_T03": contrast(m[~m.subject.isin(["T09", "T03"])]),
        "note": ("二分对比在全队列上显著, 但排除 T09 与 T03 后消失 (OR≈1); "
                 "因此这是两个动作块的集中失败, 不构成测量一致性对分类误判的一般性预测"),
    }

    # ---------- Part B: 特征承重 ----------
    imp_rows = []
    for g, v in drops.items():
        kind, name = g.split("::")
        imp_rows.append({"kind": kind, "group": name,
                         "mean_drop": float(np.mean(v)), "sd": float(np.std(v, ddof=1)),
                         "min": float(np.min(v)), "max": float(np.max(v))})
    imp = pd.DataFrame(imp_rows).sort_values(["kind", "mean_drop"], ascending=[True, False])
    imp.to_csv(HERE / "feature_importance.csv", index=False)
    out["part_b_feature_load"] = {
        "method": (f"leave-one-subject-out 分组置换, 每折每组 {N_PERM} 次置换; "
                   "分数下降以参与者等权宏平均 F1 计"),
        "baseline": replicated,
        "by_pair": imp[imp.kind == "pair"].to_dict("records"),
        "by_family": imp[imp.kind == "family"].to_dict("records"),
        "by_block": imp[imp.kind == "block"].to_dict("records"),
    }

    # ---------- Part C: 集中失败块的方向特征 ----------
    dirs = [c for c in fcols if c.split("__")[1].startswith("dir_")]
    fd = f.assign(block=pk.block.values, correct=preds == y)
    flips = []
    for (s, b), g in fd.groupby(["subject", "block"]):
        act = g.label.iloc[0]
        oth = fd[(fd.label == act) & (fd.subject != s)]
        nf = 0
        for c in dirs:
            a, bb = g[c].median(), oth[c].median()
            if np.isfinite(a) and np.isfinite(bb) and abs(a) > 0.1 and abs(bb) > 0.1 \
                    and np.sign(a) != np.sign(bb):
                nf += 1
        flips.append({"subject": s, "block": b, "action": act, "n": int(len(g)),
                      "n_err": int((~g.correct).sum()), "n_dir_flipped": nf})
    fl = pd.DataFrame(flips)
    fl["err_rate"] = fl.n_err / fl.n
    fl.to_csv(HERE / "direction_flips.csv", index=False)
    sp_flip = stats.spearmanr(fl.n_dir_flipped, fl.err_rate)

    within = {}
    for s, b in [("T09", "B1"), ("T03", "B5")]:
        g = fd[(fd.subject == s) & (fd.block == b)]
        ge, gc = g[~g.correct], g[g.correct]
        diffs = [(c, float(ge[c].median() - gc[c].median())) for c in dirs
                 if np.isfinite(ge[c].median()) and np.isfinite(gc[c].median())]
        diffs.sort(key=lambda x: -abs(x[1]))
        within[f"{s}_{b}"] = {"n_err": int(len(ge)), "n_correct": int(len(gc)),
                              "top_direction_gaps": [
                                  {"feature": c.replace("pair_", ""), "delta": round(v, 3)}
                                  for c, v in diffs[:5]]}
    out["part_c_direction_diagnostic"] = {
        "cross_block_flip_vs_error": {"spearman_rho": float(sp_flip.statistic),
                                      "p": float(sp_flip.pvalue), "n_blocks": int(len(fl))},
        "within_block_error_vs_correct": within,
        "note": ("跨参与者的方向特征符号本就不稳定, 与误判率无关 (rho≈0); "
                 "集中失败出现在块内部, 误判片段的方向分量相对同块正确片段系统性反号"),
    }

    (HERE / "ch10_supplement.json").write_text(
        json.dumps(out, indent=1, ensure_ascii=False), encoding="utf-8")
    d.to_csv(HERE / "bout_level_rows.csv", index=False)
    m.to_csv(HERE / "block_level_join.csv", index=False)
    print(f"复刻 = {replicated:.7f} (锁定 0.9544739); 误判 {int((preds != y).sum())}; "
          f"逐片段一致 {bool((preds == pr.y_pred.to_numpy()).all())}")
    print(f"产物写入 {HERE}")


if __name__ == "__main__":
    main()
