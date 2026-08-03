"""Promote the all-pairs (10-relation, 130-feature) scheme to a formal dataset + results.

Produces the same artifact shape as the frozen mapping-repair runs so this can stand as a
headline candidate rather than an exploratory side result:

  features_model.csv      leak-safe model matrix (identity/label columns + 130 features)
  dataset_manifest.json   run type, placement provenance, registry + output hashes
  results_allpairs.json   full LOSO metrics for logistic and random forest + permutation null

Source of truth is the ablation channel table built by ``ablation_build.py``, which itself
reuses ``dataset_adapter.subject_blocks`` as the frozen authority for raw segments,
clocks, protocol windows and detected bouts. No bout is re-detected and no raw file is touched.

Scheme definition: all C(5,2) = 10 ordered sensor pairs, each contributing the same 13
scale-invariant, axis-agnostic features the frozen pipeline uses. Ordering within a pair is
inferior -> superior along ``ablation_build.ROOT_ORDER``.

Dimensionality note: 130 features over 1387 rows (~10.7:1). Logistic relies on its default L2
penalty; the permutation null is the guard against the ratio being too generous.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import (accuracy_score, confusion_matrix, f1_score,
                             precision_score, recall_score)
from sklearn.model_selection import LeaveOneGroupOut
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import ablation_build as ab      # noqa: E402
import placement_maps as pm      # noqa: E402

CLASSES = ["flexion", "extension", "left_bend", "right_bend", "left_twist", "right_twist"]
LABELS = list(range(6))
ID_COLS = ["subject", "trial_id", "mapping_version", "mapping_sha256", "y", "label", "quality"]

# Columns that must never reach X: MoCap ground truth, raw peaks, and bout timing.
FORBIDDEN = ("mocap_peak", "_peak_x", "_peak_y", "_peak_tw",
             "bout_index", "bout_start_s", "bout_end_s", "block", "n_samples", "dur_s")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def all_pair_channels() -> list[str]:
    return [f"pair_{p}_{c}"
            for i, p in enumerate(ab.ROOT_ORDER)
            for c in ab.ROOT_ORDER[i + 1:]]


def loso_full(make_model, X, y, groups) -> dict:
    logo = LeaveOneGroupOut()
    pred = np.full(len(y), -1, dtype=int)
    for tr, te in logo.split(X, y, groups):
        m = make_model()
        m.fit(X[tr], y[tr])
        pred[te] = m.predict(X[te])
    assert (pred >= 0).all(), "every row must be held out exactly once"
    return {
        "accuracy": float(accuracy_score(y, pred)),
        "macro_f1": float(f1_score(y, pred, labels=LABELS, average="macro")),
        "per_class_recall": {CLASSES[i]: float(v) for i, v in enumerate(
            recall_score(y, pred, labels=LABELS, average=None, zero_division=0))},
        "per_class_precision": {CLASSES[i]: float(v) for i, v in enumerate(
            precision_score(y, pred, labels=LABELS, average=None, zero_division=0))},
        "confusion_matrix": confusion_matrix(y, pred, labels=LABELS).tolist(),
        "per_subject_accuracy": {str(s): float(accuracy_score(y[groups == s], pred[groups == s]))
                                 for s in sorted(set(groups))},
    }


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Promote the all-pairs scheme to a formal dataset.")
    p.add_argument("--ablation-dir", type=Path, required=True)
    p.add_argument("--out-dir", type=Path, required=True)
    p.add_argument("--permutations", type=int, default=200)
    args = p.parse_args(argv)

    abl_dir = args.ablation_dir.resolve()
    out_dir = args.out_dir.resolve()
    if out_dir.exists() and any(out_dir.iterdir()):
        raise SystemExit(f"--out-dir must be new or empty; refusing to overwrite {out_dir}")

    src_manifest_path = abl_dir / "ablation_manifest.json"
    src_features_path = abl_dir / "channel_features.csv"
    src_manifest = json.loads(src_manifest_path.read_text(encoding="utf-8"))
    actual = sha256_file(src_features_path)
    if src_manifest["outputs"]["channel_features.csv"] != actual:
        raise SystemExit(f"stale/tampered source channel table: {actual}")

    registry_hash = sha256_file(Path(pm.DEFAULT_CONFIG_PATH))
    if src_manifest["mapping_registry_sha256"] != registry_hash:
        raise SystemExit("placement registry hash drifted since the channel table was built")

    df = pd.read_csv(src_features_path)
    chans = all_pair_channels()
    if len(chans) != 10:
        raise SystemExit(f"expected 10 pair channels, got {len(chans)}")
    feat_cols = [f"{c}__{f}" for c in chans for f in ab.FEAT_NAMES]
    if len(feat_cols) != 130:
        raise SystemExit(f"expected 130 features, got {len(feat_cols)}")
    missing = [c for c in feat_cols if c not in df.columns]
    if missing:
        raise SystemExit(f"missing feature columns: {missing[:5]}")

    # Fail closed on any leak-prone column reaching the model matrix.
    bad = [c for c in feat_cols if any(f in c for f in FORBIDDEN)]
    if bad:
        raise SystemExit(f"forbidden column pattern in feature list: {bad}")
    if set(feat_cols) & set(ID_COLS):
        raise SystemExit("identity column present in feature list")

    out_dir.mkdir(parents=True, exist_ok=True)
    model_df = df[ID_COLS + feat_cols].copy()
    features_path = out_dir / "features_model.csv"
    model_df.to_csv(features_path, index=False)

    placements = {}
    for trial_id, grp in df.groupby("trial_id"):
        versions = set(grp["mapping_version"])
        hashes = set(grp["mapping_sha256"])
        if len(versions) != 1 or len(hashes) != 1:
            raise SystemExit(f"inconsistent placement provenance within {trial_id}")
        resolved = pm.resolve_placement(trial_id=str(trial_id))
        if resolved.canonical_sha256 != hashes.pop():
            raise SystemExit(f"placement hash mismatch vs registry for {trial_id}")
        placements[str(trial_id)] = {
            "mapping_version": versions.pop(),
            "mapping_sha256": resolved.canonical_sha256,
            "mapping_status": resolved.status,
            "placement_map": dict(resolved.role_to_imu),
        }
        if resolved.status not in {"confirmed", "inferred_high"}:
            raise SystemExit(f"non-production placement: {trial_id}={resolved.status}")

    X = model_df[feat_cols].to_numpy(float)
    y = model_df["y"].to_numpy(int)
    groups = model_df["subject"].to_numpy()

    logit = lambda: Pipeline([("scaler", StandardScaler()),
                              ("clf", LogisticRegression(max_iter=5000, class_weight="balanced"))])
    rf = lambda: RandomForestClassifier(n_estimators=400, class_weight="balanced", random_state=0)

    results = {"logistic": loso_full(logit, X, y, groups),
               "random_forest": loso_full(rf, X, y, groups)}

    logo = LeaveOneGroupOut()
    rng = np.random.default_rng(0)
    null = np.empty(args.permutations)
    for k in range(args.permutations):
        yp = rng.permutation(y)
        pred = np.full(len(yp), -1, dtype=int)
        for tr, te in logo.split(X, yp, groups):
            m = logit()
            m.fit(X[tr], yp[tr])
            pred[te] = m.predict(X[te])
        null[k] = accuracy_score(yp, pred)
    obs = results["logistic"]["accuracy"]
    pval = float((np.sum(null >= obs) + 1) / (args.permutations + 1))

    manifest = {
        "run_type": "allpairs_headline_candidate",
        "scheme": "all C(5,2)=10 sensor pairs x 13 scale-invariant features",
        "channels": chans,
        "features_per_channel": ab.FEAT_NAMES,
        "n_features": len(feat_cols),
        "n_rows": int(len(model_df)),
        "n_subjects": int(model_df["subject"].nunique()),
        "analysis_mode": src_manifest["analysis_mode"],
        "source_channel_table": str(src_features_path),
        "source_channel_table_sha256": actual,
        "source_manifest_sha256": sha256_file(src_manifest_path),
        "frozen_authority": src_manifest["frozen_authority"],
        "mapping_registry": str(pm.DEFAULT_CONFIG_PATH),
        "mapping_registry_sha256": registry_hash,
        "placements": placements,
        "builder_sha256": sha256_file(Path(__file__).resolve()),
        "outputs": {"features_model.csv": sha256_file(features_path)},
        "environment": {"python": platform.python_version(), "numpy": np.__version__,
                        "pandas": pd.__version__},
        "dimensionality_note": (
            f"{len(feat_cols)} features over {len(model_df)} rows "
            f"({len(model_df) / len(feat_cols):.1f}:1). Logistic uses sklearn's default L2 "
            "penalty; the permutation null is the guard on this ratio."
        ),
    }
    (out_dir / "dataset_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n",
                                                   encoding="utf-8")

    payload = {
        "run_type": "allpairs_headline_candidate_results",
        "dataset_manifest_sha256": sha256_file(out_dir / "dataset_manifest.json"),
        "features_model_sha256": sha256_file(features_path),
        "n_rows": int(len(model_df)),
        "n_subjects": int(model_df["subject"].nunique()),
        "chance": 1.0 / 6.0,
        "majority": float(np.bincount(y).max() / len(y)),
        "results": results,
        "permutation": {"n": args.permutations, "observed": obs,
                        "null_mean": float(null.mean()), "null_max": float(null.max()),
                        "p_value": pval},
    }
    (out_dir / "results_allpairs.json").write_text(json.dumps(payload, indent=2) + "\n",
                                                    encoding="utf-8")

    print(f"rows={len(model_df)} subjects={model_df['subject'].nunique()} features={len(feat_cols)}")
    for name, r in results.items():
        print(f"[{name}] acc={r['accuracy']:.4f} macroF1={r['macro_f1']:.4f}")
        print("  recall: " + "  ".join(f"{c[:5]}={v:.3f}" for c, v in r["per_class_recall"].items()))
    print(f"permutation: observed={obs:.4f} null_mean={null.mean():.4f} p={pval:.4f}")
    print(f"wrote {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
