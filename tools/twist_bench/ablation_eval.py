"""Exhaustive sensor-subset ablation over the 6-class movement classifier.

Evaluates all 31 non-empty subsets of {sacrum, lower, mid, upper, sternum} plus the frozen
``legacy`` fused block, using Leave-One-Subject-Out over 13 subjects with pooled held-out
predictions. Reads the wide per-channel table from ``ablation_build.py``; no subset needs a
feature rebuild, only a column selection.

ANTI-LEAKAGE
------------
X is built exclusively from ``<channel>__<feature>`` columns of the subset's own channels.
Subject, label, quality and bout metadata never enter X. StandardScaler is refit inside each
training fold via a Pipeline. Hyperparameters are fixed; nothing is tuned against a test fold.

DIMENSION CONFOUND
------------------
A k-sensor subset carries 13*(k-1) features (13 for k=1), so larger subsets get more columns.
Any "more sensors is better" result is therefore partly a capacity effect. Two mitigations are
reported rather than hidden: logistic regression is regularized and comparatively insensitive
to added columns, and a random forest is reported alongside it. The write-up must name the
confound.

PRE-SPECIFIED CONTRASTS
-----------------------
Paired Wilcoxon signed-rank over the 13 per-subject accuracies, for a small fixed set of
contrasts chosen before seeing results. All-pairs testing across 31 subsets is deliberately
avoided.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import platform
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.stats import wilcoxon
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import accuracy_score, confusion_matrix, f1_score, recall_score
from sklearn.model_selection import LeaveOneGroupOut
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import ablation_build as ab  # noqa: E402

CLASSES = ["flexion", "extension", "left_bend", "right_bend", "left_twist", "right_twist"]
LABELS = list(range(6))
META_PREFIXES = ("subject", "trial_id", "mapping_", "block", "label", "y", "quality",
                 "bout_", "n_samples", "dur_s")

# Fixed before any result was inspected. Each entry is (name, subset_a, subset_b); the test
# asks whether A differs from B across the 13 per-subject accuracies.
CONTRASTS = [
    ("five_vs_best_single", ("sacrum", "lower", "mid", "upper", "sternum"), None),
    ("five_vs_frozen_three", ("sacrum", "lower", "mid", "upper", "sternum"),
     ("sacrum", "upper", "sternum")),
    ("frozen_three_vs_best_pair", ("sacrum", "upper", "sternum"), None),
    ("five_vs_four_no_lower", ("sacrum", "lower", "mid", "upper", "sternum"),
     ("sacrum", "mid", "upper", "sternum")),
    ("five_vs_four_no_mid", ("sacrum", "lower", "mid", "upper", "sternum"),
     ("sacrum", "lower", "upper", "sternum")),
]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def subset_key(subset: tuple[str, ...]) -> str:
    return "+".join(r for r in ab.ROOT_ORDER if r in subset)


def build_models() -> dict:
    return {
        "logistic": lambda: Pipeline([
            ("scaler", StandardScaler()),
            ("clf", LogisticRegression(max_iter=5000, class_weight="balanced")),
        ]),
        "random_forest": lambda: RandomForestClassifier(
            n_estimators=400, class_weight="balanced", random_state=0
        ),
    }


def loso(make_model, X, y, groups):
    """Pooled held-out predictions plus per-subject accuracy, index-tracked."""
    logo = LeaveOneGroupOut()
    pred = np.full(len(y), -1, dtype=int)
    for tr, te in logo.split(X, y, groups):
        m = make_model()
        m.fit(X[tr], y[tr])
        pred[te] = m.predict(X[te])
    assert (pred >= 0).all(), "every row must be held out exactly once"

    per_subject = {
        str(s): float(accuracy_score(y[groups == s], pred[groups == s]))
        for s in sorted(set(groups))
    }
    return {
        "accuracy": float(accuracy_score(y, pred)),
        "macro_f1": float(f1_score(y, pred, labels=LABELS, average="macro")),
        "per_class_recall": {
            CLASSES[i]: float(v) for i, v in enumerate(
                recall_score(y, pred, labels=LABELS, average=None, zero_division=0)
            )
        },
        "per_subject_accuracy": per_subject,
        "confusion_matrix": confusion_matrix(y, pred, labels=LABELS).tolist(),
    }


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Exhaustive sensor-subset ablation.")
    p.add_argument("--run-dir", type=Path, required=True,
                   help="Directory produced by ablation_build.py.")
    args = p.parse_args(argv)
    run_dir = args.run_dir.resolve()

    manifest_path = run_dir / "ablation_manifest.json"
    features_path = run_dir / "channel_features.csv"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    actual = sha256_file(features_path)
    if manifest["outputs"]["channel_features.csv"] != actual:
        raise SystemExit(f"stale/tampered channel_features.csv: {actual}")

    out_path = run_dir / "ablation_results.json"
    if out_path.exists():
        raise SystemExit(f"refusing to overwrite existing results: {out_path}")

    df = pd.read_csv(features_path)
    y = df["y"].to_numpy(int)
    groups = df["subject"].to_numpy()

    leaked = [c for c in df.columns if c.startswith(META_PREFIXES) and "__" in c]
    assert not leaked, f"metadata column carries a feature suffix: {leaked}"
    print(f"rows={len(df)}  subjects={df['subject'].nunique()}  "
          f"channels={len(manifest['channels'])}")

    models = build_models()
    results: dict[str, dict] = {}

    # All 31 non-empty subsets, plus the frozen fused block as a same-pipeline reference.
    subsets = [
        tuple(c)
        for k in range(1, len(ab.ROOT_ORDER) + 1)
        for c in itertools.combinations(ab.ROOT_ORDER, k)
    ]
    jobs = [(subset_key(s), ab.subset_channels(s), len(s)) for s in subsets]
    jobs.append(("legacy_frozen_block", ["legacy"], 3))

    for key, chans, n_sensors in jobs:
        cols = [f"{c}__{f}" for c in chans for f in ab.FEAT_NAMES]
        missing = [c for c in cols if c not in df.columns]
        if missing:
            raise SystemExit(f"missing feature columns for {key}: {missing[:3]}")
        X = df[cols].to_numpy(float)
        entry = {
            "n_sensors": n_sensors,
            "channels": chans,
            "n_features": len(cols),
        }
        for mname, factory in models.items():
            entry[mname] = loso(factory, X, y, groups)
        results[key] = entry
        print(f"  {key:44s} k={n_sensors} d={len(cols):3d}  "
              f"logit={entry['logistic']['accuracy']:.4f}  "
              f"rf={entry['random_forest']['accuracy']:.4f}", flush=True)

    # ---- pre-specified paired contrasts on per-subject accuracy (n=13) ----
    def acc_vec(key: str, model: str) -> np.ndarray:
        ps = results[key][model]["per_subject_accuracy"]
        return np.array([ps[s] for s in sorted(ps)])

    def best_with_k(k: int, model: str) -> str:
        cands = [key for key, e in results.items()
                 if e["n_sensors"] == k and key != "legacy_frozen_block"]
        return max(cands, key=lambda key: results[key][model]["accuracy"])

    contrasts = {}
    for name, a_sub, b_sub in CONTRASTS:
        for model in models:
            a_key = subset_key(a_sub)
            if b_sub is None:
                b_key = best_with_k(1, model) if "single" in name else best_with_k(2, model)
            else:
                b_key = subset_key(b_sub)
            va, vb = acc_vec(a_key, model), acc_vec(b_key, model)
            if np.allclose(va, vb):
                stat, pval = float("nan"), 1.0
            else:
                stat, pval = wilcoxon(va, vb)
            contrasts[f"{name}::{model}"] = {
                "a": a_key, "b": b_key,
                "a_pooled": results[a_key][model]["accuracy"],
                "b_pooled": results[b_key][model]["accuracy"],
                "a_subject_mean": float(va.mean()),
                "b_subject_mean": float(vb.mean()),
                "median_delta": float(np.median(va - vb)),
                "wilcoxon_stat": float(stat),
                "p_value": float(pval),
                "n_subjects": int(len(va)),
            }

    payload = {
        "run_type": "sensor_subset_ablation_results",
        "source_manifest_sha256": sha256_file(manifest_path),
        "channel_features_sha256": actual,
        "evaluator_sha256": sha256_file(Path(__file__).resolve()),
        "analysis_mode": manifest["analysis_mode"],
        "n_rows": int(len(df)),
        "n_subjects": int(df["subject"].nunique()),
        "chance": 1.0 / 6.0,
        "majority": float(np.bincount(y).max() / len(y)),
        "root_order": ab.ROOT_ORDER,
        "subsets": results,
        "contrasts": contrasts,
        "confounds": {
            "reference_frame": manifest["reference_frame_confound"],
            "dimension": (
                "Feature count is 13*(k-1) for k>1 and 13 for k=1, so subset size and model "
                "capacity move together; a monotone accuracy-vs-k trend is not by itself "
                "evidence that a placement carries independent information."
            ),
        },
        "environment": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "pandas": pd.__version__,
        },
    }
    out_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"\nwrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
